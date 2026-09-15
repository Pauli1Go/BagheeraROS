"""Fisheye calibration for the Bagheera front camera (OV5647, NoIR).

Usage (on a machine with OpenCV, e.g. the Mac)::

    pip install opencv-python numpy pyyaml
    python3 tools/calibrate_fisheye.py --images /tmp/calib \
        --pattern 9x6 --square 0.024 --out src/bagheera_base/config/camera_fisheye.yaml

Board: 10x7 squares of 24 mm -> 9x6 inner corners. Images are the already
180-degree-rotated /camera/image_raw frames (1920x1080, bgr8).

Writes a ROS camera_info YAML (distortion_model: equidistant) that can be
referenced from sensors.yaml via camera_info_url, plus undistorted previews
next to the output file. Prints per-image detection results and a corner
coverage summary so missing FOV areas become visible.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

try:
    import yaml
except ImportError:  # minimal fallback writer
    yaml = None


def find_corners(path: Path, pattern: tuple[int, int]):
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        return None, None, f"{path.name}: unreadable"
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(img, pattern, flags)
    if not ok:
        return None, img.shape[::-1], f"{path.name}: no 9x6 pattern"
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
    cv2.cornerSubPix(img, corners, (3, 3), (-1, -1), criteria)
    corners = corners.reshape(-1, 1, 2).astype(np.float64)
    return corners, img.shape[::-1], f"{path.name}: ok"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True)
    ap.add_argument("--pattern", default="9x6")
    ap.add_argument("--square", type=float, default=0.024)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    cols, rows = (int(v) for v in args.pattern.split("x"))
    pattern = (cols, rows)
    objp = np.zeros((rows * cols, 1, 3), np.float64)
    objp[:, 0, :2] = (
        np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square
    )

    objpoints, imgpoints, sizes = [], [], set()
    coverage = np.zeros((3, 4), np.int32)  # coarse 4x3 FOV coverage grid
    for path in sorted(Path(args.images).glob("img_*.png")):
        corners, size, msg = find_corners(path, pattern)
        print(msg)
        if corners is None:
            continue
        sizes.add(size)
        objpoints.append(objp)
        imgpoints.append(corners)
        h, w = corners[:, 0, 1], corners[:, 0, 0]
        img_w, img_h = size
        coverage[
            np.clip((h / img_h * 3).astype(int), 0, 2)[:, None],
            np.clip((w / img_w * 4).astype(int), 0, 3),
        ] += 1

    print(f"\nusable: {len(objpoints)} images, sizes: {sorted(sizes)}")
    print("corner coverage (rows top-bottom x cols left-right):")
    print(coverage)
    if len(objpoints) < 15:
        raise SystemExit("too few usable images (need >= 15)")
    if len(sizes) != 1:
        raise SystemExit("mixed image sizes, calibrate per resolution")
    img_w, img_h = sorted(sizes)[0]

    K = np.eye(3)
    K[0, 0] = K[1, 1] = 1000.0
    K[0, 2], K[1, 2] = img_w / 2.0, img_h / 2.0
    D = np.zeros((4, 1))
    fisheye_mod = cv2.fisheye
    calib_flags = getattr(fisheye_mod, "CALIB_RECOMPUTE_EXTRINSIC", None)
    if calib_flags is None:  # OpenCV >= 5 moved them to the cv2 namespace
        calib_flags = cv2.CALIB_RECOMPUTE_EXTRINSIC + cv2.CALIB_FIX_SKEW
    else:
        calib_flags = (
            fisheye_mod.CALIB_RECOMPUTE_EXTRINSIC + fisheye_mod.CALIB_FIX_SKEW
        )
    rms, K, D, _, _ = cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        (img_w, img_h),
        K,
        D,
        flags=calib_flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
    )
    print(f"\nrms reprojection error: {rms:.4f} px")
    print("K:\n", K)
    print("D (k1..k4):", D.ravel())

    P = np.zeros((3, 4))
    P[:3, :3] = K
    info = {
        "image_width": img_w,
        "image_height": img_h,
        "camera_name": "ov5647_fisheye",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [float(v) for v in K.reshape(-1)],
        },
        "distortion_model": "equidistant",
        "distortion_coefficients": {
            "rows": 1,
            "cols": 4,
            "data": [float(v) for v in D.ravel()],
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [float(v) for v in P.reshape(-1)],
        },
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as handle:
        if yaml is not None:
            yaml.safe_dump(info, handle, sort_keys=False)
        else:
            handle.write(str(info))
    print(f"wrote {out}")

    # undistorted previews for the first usable image
    first = sorted(Path(args.images).glob("img_*.png"))[0]
    img = cv2.imread(str(first))
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (img_w, img_h), np.eye(3), balance=0.0
    )
    undist = cv2.fisheye.undistortImage(img, K, D, Knew=new_K)
    preview = out.parent / "calib_undistorted_preview.png"
    cv2.imwrite(str(preview), undist)
    print(f"wrote {preview}")


if __name__ == "__main__":
    main()
