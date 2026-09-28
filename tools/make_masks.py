"""Create the keepout and localization-exclusion masks for a saved map.

Nav2 loads both masks at boot (nav2_navigation.yaml); without them the mask
servers fail to activate. This writes all-free masks with exactly the map's
size, resolution and origin. Paint forbidden areas black (0) afterwards in any
image editor that keeps 8-bit greyscale PGM:

- keepout_mask.pgm: Nav2 never plans or drives into black cells.
- localization_exclusion_mask.pgm: AMCL poses inside black cells are rejected
  (for example the far side of a glass wall where the robot cannot be).

Stdlib only, runs on the host:
    python3 tools/make_masks.py maps/current.yaml
Existing masks are kept unless --force is given.
"""

from __future__ import annotations

import argparse
from pathlib import Path

MASKS = ("keepout_mask", "localization_exclusion_mask")
FREE = 254


def read_map_yaml(path: Path) -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition(":")
        if separator and not line.startswith((" ", "#")):
            values[key.strip()] = value.strip()
    return values


def read_pgm_size(path: Path) -> tuple[int, int]:
    tokens: list[bytes] = []
    with path.open("rb") as handle:
        while len(tokens) < 3:
            line = handle.readline()
            if not line:
                raise ValueError(f"{path}: truncated PGM header")
            tokens.extend(line.split(b"#", 1)[0].split())
    if tokens[0] != b"P5":
        raise ValueError(f"{path}: expected binary PGM (P5), got {tokens[0]!r}")
    return int(tokens[1]), int(tokens[2])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("map_yaml", type=Path, help="e.g. maps/current.yaml")
    parser.add_argument("--force", action="store_true", help="overwrite existing masks")
    args = parser.parse_args()

    values = read_map_yaml(args.map_yaml)
    image = args.map_yaml.resolve().parent / values["image"]
    width, height = read_pgm_size(image)
    directory = args.map_yaml.parent

    for name in MASKS:
        pgm = directory / f"{name}.pgm"
        yaml = directory / f"{name}.yaml"
        if (pgm.exists() or yaml.exists()) and not args.force:
            print(f"kept existing {pgm.name} (use --force to overwrite)")
            continue
        pgm.write_bytes(f"P5\n{width} {height}\n255\n".encode() + bytes([FREE]) * (width * height))
        yaml.write_text(
            f"image: {pgm.name}\n"
            "mode: trinary\n"
            f"resolution: {values['resolution']}\n"
            f"origin: {values['origin']}\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n",
            encoding="utf-8",
        )
        print(f"wrote {pgm} and {yaml.name} ({width}x{height})")


if __name__ == "__main__":
    main()
