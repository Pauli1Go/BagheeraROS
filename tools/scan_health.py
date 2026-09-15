#!/usr/bin/env python3
"""Report which sectors of a dumped /scan are actually usable.

Usage: python3.11 tools/scan_health.py --scan /tmp/scan_probe.json
"""

import argparse
import json

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan", required=True)
    parser.add_argument("--bins", type=int, default=36)
    args = parser.parse_args()

    with open(args.scan) as handle:
        scan = json.load(handle)

    ranges = np.asarray(scan["ranges"], dtype=float)
    angles = np.rad2deg(scan["angle_min"] + scan["angle_increment"] * np.arange(len(ranges)))
    usable = (np.isfinite(ranges) & (ranges >= scan["range_min"])
              & (ranges < scan["range_max"] - 1e-3))
    dead = ~usable

    print(f"beams {len(ranges)}, usable {usable.sum()} ({100 * usable.mean():.1f}%), "
          f"dead {dead.sum()} ({100 * dead.mean():.1f}%)")
    if dead.sum():
        print(f"dead range values: {np.nanmin(ranges[dead]):.3f} .. {np.nanmax(ranges[dead]):.3f} m "
              f"(range_min {scan['range_min']})")
        print(f"usable distances : {ranges[usable].min():.2f} .. {ranges[usable].max():.2f} m, "
              f"mean {ranges[usable].mean():.2f} m")

    edges = np.linspace(-180.0, 180.0, args.bins + 1)
    dead_hist, _ = np.histogram(angles[dead], bins=args.bins, range=(-180.0, 180.0))
    usable_hist, _ = np.histogram(angles[usable], bins=args.bins, range=(-180.0, 180.0))
    print("\nsector (deg)      dead / usable")
    for index in range(args.bins):
        if dead_hist[index] or usable_hist[index]:
            bar = "#" * int(round(30 * usable_hist[index] / max(1, usable_hist.max())))
            print(f"  {edges[index]:+7.0f}..{edges[index + 1]:+5.0f} : "
                  f"{dead_hist[index]:4d} / {usable_hist[index]:4d}  {bar}")

    runs, index = [], 0
    while index < len(dead):
        if dead[index]:
            start = index
            while index + 1 < len(dead) and dead[index + 1]:
                index += 1
            runs.append((start, index))
        index += 1
    big = [run for run in runs if run[1] - run[0] + 1 >= 5]
    print(f"\ndead runs >= 5 beams: {len(big)} (of {len(runs)} total)")
    for start, stop in big:
        print(f"  angle {angles[start]:+7.1f} .. {angles[stop]:+7.1f} deg "
              f"({stop - start + 1} beams)")


if __name__ == "__main__":
    main()
