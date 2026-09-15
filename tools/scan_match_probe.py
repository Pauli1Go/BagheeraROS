#!/usr/bin/env python3
"""Score a live LiDAR scan against a saved occupancy map.

Reads a scan dump (JSON from /scan, frame lidar_link) plus a PGM/YAML map and
answers two questions without any GUI:

  1. How well does the scan match at a given (x, y, yaw)?
  2. Where on the map does it match best, and is that answer unambiguous?

Many near-equally good matches far apart mean the map is ghosted or
self-similar. A single sharp peak means the map is fine and the problem sits
in the localization parameters instead.
"""

import argparse
import json

import numpy as np


def load_pgm(path):
    """Load a binary PGM (P5) into a 2D uint8 array, row 0 = bottom."""
    with open(path, "rb") as handle:
        blob = handle.read()

    fields, idx = [], 0
    while len(fields) < 4:
        while blob[idx:idx + 1].isspace():
            idx += 1
        if blob[idx:idx + 1] == b"#":
            while blob[idx:idx + 1] != b"\n":
                idx += 1
            continue
        start = idx
        while not blob[idx:idx + 1].isspace():
            idx += 1
        fields.append(blob[start:idx])

    magic, width, height = fields[0], int(fields[1]), int(fields[2])
    if magic != b"P5":
        raise ValueError(f"expected binary PGM (P5), got {magic!r}")
    image = np.frombuffer(blob[idx + 1:idx + 1 + width * height], dtype=np.uint8)
    # PGM row 0 is the top of the image; ROS map row 0 is the bottom.
    return image.reshape(height, width)[::-1]


def load_map_yaml(path):
    """Minimal ROS map YAML reader: scalars plus the origin triple."""
    values = {}
    with open(path) as handle:
        for line in handle:
            if ":" not in line or line.strip().startswith("#"):
                continue
            key, raw = line.split(":", 1)
            raw = raw.split("#")[0].strip()
            if raw.startswith("[") and raw.endswith("]"):
                values[key.strip()] = [float(v) for v in raw.strip("[]").split(",")]
            else:
                try:
                    values[key.strip()] = float(raw)
                except ValueError:
                    pass
    return values


def dilate(mask):
    out = mask.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            out |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    return out


def distance_to_occupied(occupied, resolution, max_steps=14):
    """Distance in metres from every cell to the nearest occupied cell."""
    distance = np.where(occupied, 0.0, np.inf)
    grown = occupied.copy()
    for step in range(1, max_steps + 1):
        grown = dilate(grown)
        distance = np.where(np.isfinite(distance), distance,
                            np.where(grown, step * resolution, np.inf))
    return distance


def scan_to_base_points(scan, base_yaw, subsample):
    """Beam endpoints of the scan expressed in the base_link frame."""
    ranges = np.asarray(scan["ranges"], dtype=float)
    angles = scan["angle_min"] + scan["angle_increment"] * np.arange(len(ranges))
    valid = (np.isfinite(ranges) & (ranges >= scan["range_min"])
             & (ranges < scan["range_max"] - 1e-3))
    ranges, angles = ranges[valid][::subsample], angles[valid][::subsample]

    lidar_xy = np.stack([ranges * np.cos(angles), ranges * np.sin(angles)], axis=1)
    cos_y, sin_y = np.cos(base_yaw), np.sin(base_yaw)
    rotation = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
    return lidar_xy @ rotation.T


def transform(points, x, y, yaw):
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    return np.stack([
        x + points[:, 0] * cos_y - points[:, 1] * sin_y,
        y + points[:, 0] * sin_y + points[:, 1] * cos_y,
    ], axis=1)


def hit_fraction(world_points, distance, resolution, origin, tolerance):
    cols = ((world_points[:, 0] - origin[0]) / resolution).astype(np.int32)
    rows = ((world_points[:, 1] - origin[1]) / resolution).astype(np.int32)
    height, width = distance.shape
    inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    local = np.full(len(world_points), np.inf)
    np.copyto(local, distance[rows.clip(0, height - 1), cols.clip(0, width - 1)],
              where=inside)
    return float(np.mean(np.clip(1.0 - local / tolerance, 0.0, 1.0)))


def global_search(points, distance, resolution, origin, yaws, step, tolerance,
                  chunk=20_000):
    height, width = distance.shape
    xs = origin[0] + resolution * np.arange(width)
    ys = origin[1] + resolution * np.arange(height)
    grid_x, grid_y = np.meshgrid(xs, ys)
    grid_x, grid_y = grid_x.ravel(), grid_y.ravel()
    stride = max(1, int(round(step / resolution)))
    grid_x, grid_y = grid_x[::stride], grid_y[::stride]

    all_scores = np.empty((len(yaws), len(grid_x)))
    for index, yaw in enumerate(yaws):
        rotated = transform(points, 0.0, 0.0, yaw)
        scores = np.empty(len(grid_x))
        for start in range(0, len(grid_x), chunk):
            stop = min(start + chunk, len(grid_x))
            cols = ((grid_x[start:stop, None] + rotated[None, :, 0] - origin[0])
                    / resolution).astype(np.int32)
            rows = ((grid_y[start:stop, None] + rotated[None, :, 1] - origin[1])
                    / resolution).astype(np.int32)
            inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
            local = np.full(cols.shape, np.inf)
            np.copyto(local,
                      distance[rows.clip(0, height - 1), cols.clip(0, width - 1)],
                      where=inside)
            scores[start:stop] = np.mean(
                np.clip(1.0 - local / tolerance, 0.0, 1.0), axis=1)
        all_scores[index] = scores
    return grid_x, grid_y, all_scores


def refine(points, distance, resolution, origin, seed, tolerance,
           span=0.4, step=0.02, yaw_span_deg=8.0, yaw_step_deg=1.0):
    """Fine search around a coarse peak; returns (x, y, yaw, score)."""
    x_seed, y_seed, yaw_seed, _ = seed
    xs = np.arange(x_seed - span, x_seed + span + 1e-9, step)
    ys = np.arange(y_seed - span, y_seed + span + 1e-9, step)
    yaws = np.arange(yaw_seed - np.deg2rad(yaw_span_deg),
                     yaw_seed + np.deg2rad(yaw_span_deg) + 1e-9,
                     np.deg2rad(yaw_step_deg))
    grid_x, grid_y = np.meshgrid(xs, ys)
    grid_x, grid_y = grid_x.ravel(), grid_y.ravel()
    best = (x_seed, y_seed, yaw_seed, -1.0)
    for yaw in yaws:
        rotated = transform(points, 0.0, 0.0, yaw)
        for start in range(0, len(grid_x), 5000):
            stop = min(start + 5000, len(grid_x))
            cols = ((grid_x[start:stop, None] + rotated[None, :, 0] - origin[0])
                    / resolution).astype(np.int32)
            rows = ((grid_y[start:stop, None] + rotated[None, :, 1] - origin[1])
                    / resolution).astype(np.int32)
            height, width = distance.shape
            inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
            local = np.full(cols.shape, np.inf)
            np.copyto(local,
                      distance[rows.clip(0, height - 1), cols.clip(0, width - 1)],
                      where=inside)
            scores = np.mean(np.clip(1.0 - local / tolerance, 0.0, 1.0), axis=1)
            index = int(np.argmax(scores))
            if scores[index] > best[3]:
                best = (float(grid_x[start + index]), float(grid_y[start + index]),
                        float(yaw), float(scores[index]))
    return best


def top_peaks(grid_x, grid_y, all_scores, yaws, count, min_separation):
    flat = all_scores.ravel()
    peaks = []
    for index in np.argsort(flat)[::-1]:
        yaw_index, position_index = divmod(int(index), len(grid_x))
        x, y = float(grid_x[position_index]), float(grid_y[position_index])
        if all((x - px) ** 2 + (y - py) ** 2 > min_separation ** 2
               for px, py, _, _ in peaks):
            peaks.append((x, y, float(yaws[yaw_index]), float(flat[index])))
        if len(peaks) >= count:
            break
    return peaks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--yaml", required=True)
    parser.add_argument("--base-yaw-deg", type=float, default=-85.5,
                        help="lidar_link yaw relative to base_link, from TF")
    parser.add_argument("--subsample", type=int, default=5)
    parser.add_argument("--tolerance", type=float, default=0.30)
    parser.add_argument("--step", type=float, default=0.15)
    parser.add_argument("--yaw-step-deg", type=float, default=10.0)
    parser.add_argument("--pose", action="append", default=[],
                        help="extra pose to score, formatted x,y,yaw_deg")
    parser.add_argument("--fine-at", default=None,
                        help="also search finely around x,y,yaw_deg")
    parser.add_argument("--ascii", default=None,
                        help="render the map plus scan around x,y,yaw_deg")
    parser.add_argument("--ascii-span", type=float, default=8.0)
    parser.add_argument("--fine-span", type=float, default=0.4)
    parser.add_argument("--fine-step", type=float, default=0.02)
    parser.add_argument("--fine-yaw-span-deg", type=float, default=8.0)
    parser.add_argument("--fine-yaw-step-deg", type=float, default=1.0)
    args = parser.parse_args()

    meta = load_map_yaml(args.yaml)
    resolution = meta.get("resolution", 0.05)
    origin = meta.get("origin", [0.0, 0.0, 0.0])[:2]

    image = load_pgm(args.map)
    occupancy = (255.0 - image.astype(float)) / 255.0
    occupied = occupancy > meta.get("occupied_thresh", 0.65)
    free = occupancy < meta.get("free_thresh", 0.196)
    distance = distance_to_occupied(occupied, resolution)

    with open(args.scan) as handle:
        scan = json.load(handle)
    points = scan_to_base_points(scan, np.deg2rad(args.base_yaw_deg), args.subsample)

    print(f"map  : {image.shape[1]}x{image.shape[0]} cells @ {resolution} m = "
          f"{image.shape[1] * resolution:.1f} x {image.shape[0] * resolution:.1f} m, "
          f"origin {origin}")
    print(f"cells: occupied {100 * occupied.mean():.1f}%, free {100 * free.mean():.1f}%, "
          f"unknown {100 * (~occupied & ~free).mean():.1f}%")
    print(f"scan : {len(points)} beams used of {len(scan['ranges'])}, frame {scan['frame_id']}")

    for pose_text in args.pose:
        x, y, yaw_deg = (float(value) for value in pose_text.split(","))
        score = hit_fraction(transform(points, x, y, np.deg2rad(yaw_deg)),
                             distance, resolution, origin, args.tolerance)
        print(f"score at ({x:7.2f}, {y:7.2f}, {yaw_deg:6.1f} deg) = {score:.3f}")

    if args.fine_at:
        fine_x, fine_y, fine_yaw = (float(v) for v in args.fine_at.split(","))
        best = refine(points, distance, resolution, origin,
                      (fine_x, fine_y, np.deg2rad(fine_yaw), 0.0), args.tolerance,
                      span=args.fine_span, step=args.fine_step,
                      yaw_span_deg=args.fine_yaw_span_deg,
                      yaw_step_deg=args.fine_yaw_step_deg)
        print(f"fine search around ({fine_x:.2f}, {fine_y:.2f}, {fine_yaw:.1f} deg) "
              f"-> ({best[0]:.3f}, {best[1]:.3f}, {np.rad2deg(best[2]):.2f} deg) = {best[3]:.3f}")

    yaws = np.deg2rad(np.arange(0, 360, args.yaw_step_deg))
    grid_x, grid_y, all_scores = global_search(points, distance, resolution, origin,
                                               yaws, args.step, args.tolerance)
    peaks = top_peaks(grid_x, grid_y, all_scores, yaws, 8, 1.0)
    print("\nbest matches on the map (x, y, yaw_deg, hit fraction):")
    for x, y, yaw, score in peaks:
        print(f"  ({x:7.2f}, {y:7.2f}, {np.rad2deg(yaw):6.1f})  {score:.3f}")

    best = peaks[0][3]
    rivals = [peak for peak in peaks[1:] if peak[3] > best - 0.10]
    verdict = "AMBIGUOUS map" if rivals else "sharp single match"
    print(f"\nbest {best:.3f}; {len(rivals)} rival location(s) within 0.10 -> {verdict}")

    refined = refine(points, distance, resolution, origin, peaks[0], args.tolerance)
    print(f"refined best: ({refined[0]:.3f}, {refined[1]:.3f}, {np.rad2deg(refined[2]):.2f} deg) "
          f"= {refined[3]:.3f}")

    mirrored = points * np.array([1.0, -1.0])
    mirror_scores = [hit_fraction(transform(mirrored, x, y, yaw), distance,
                                 resolution, origin, args.tolerance)
                     for x, y, yaw, _ in peaks[:3]]
    print("mirrored-scan control at the top-3 poses: "
          + ", ".join(f"{score:.3f}" for score in mirror_scores))

    render_ascii(occupied, free, points, refined[:3], resolution, origin,
                 args.ascii_span)
    if args.ascii:
        ascii_x, ascii_y, ascii_yaw = (float(v) for v in args.ascii.split(","))
        render_ascii(occupied, free, points, (ascii_x, ascii_y, np.deg2rad(ascii_yaw)),
                     resolution, origin, args.ascii_span)


def render_ascii(occupied, free, points, pose, resolution, origin,
                 half_width=8.0, columns=110):
    """Text render of the map around a pose with the scan drawn on top."""
    height, width = occupied.shape
    rows = max(1, int(columns * half_width * 2 / (half_width * 2) / 2.1))
    xs = np.linspace(pose[0] - half_width, pose[0] + half_width, columns)
    ys = np.linspace(pose[1] - half_width, pose[1] + half_width, rows)
    grid_x, grid_y = np.meshgrid(xs, ys)
    cols = ((grid_x - origin[0]) / resolution).astype(int)
    lines = ((grid_y - origin[1]) / resolution).astype(int)
    inside = (cols >= 0) & (cols < width) & (lines >= 0) & (lines < height)

    canvas = np.full((rows, columns), " ")
    canvas[inside & free[lines.clip(0, height - 1), cols.clip(0, width - 1)]] = "."
    canvas[inside & occupied[lines.clip(0, height - 1), cols.clip(0, width - 1)]] = "#"

    world = transform(points, pose[0], pose[1], pose[2])
    beam_cols = ((world[:, 0] - xs[0]) / (xs[1] - xs[0])).round().astype(int)
    beam_rows = ((world[:, 1] - ys[0]) / (ys[1] - ys[0])).round().astype(int)
    keep = ((beam_cols >= 0) & (beam_cols < columns)
            & (beam_rows >= 0) & (beam_rows < rows))
    canvas[beam_rows[keep], beam_cols[keep]] = "o"

    origin_col = int(round((pose[0] - xs[0]) / (xs[1] - xs[0])))
    origin_row = int(round((pose[1] - ys[0]) / (ys[1] - ys[0])))
    if 0 <= origin_col < columns and 0 <= origin_row < rows:
        canvas[origin_row, origin_col] = "R"

    header = (f"map around ({pose[0]:.2f}, {pose[1]:.2f}, {np.rad2deg(pose[2]):.1f} deg), "
              f"+/-{half_width:.1f} m, y increases upwards, R = robot")
    print("\n" + header[0])
    for row in range(rows - 1, -1, -1):
        print("  " + "".join(canvas[row]))


if __name__ == "__main__":
    main()
