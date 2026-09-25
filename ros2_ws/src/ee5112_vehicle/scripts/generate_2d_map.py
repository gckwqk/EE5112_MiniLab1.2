#!/usr/bin/env python3
# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Generate a wall-only ROS occupancy map from the supplied MiniLab JSON.

Python standard library only. Source JSON is read without modification.
Developed with AI coding assistance.
"""

import argparse
import json
import math
from pathlib import Path


def generate(specs_path, output_dir, resolution=0.01):
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Resolution must be a finite positive number.')
    arena = json.loads(Path(specs_path).read_text(encoding='utf-8'))['map']
    width_m, height_m = map(float, arena['overall_m'])
    width = round(width_m / resolution)
    height = round(height_m / resolution)
    if width < 1 or height < 1 or not (
        math.isclose(width * resolution, width_m, abs_tol=1e-9)
        and math.isclose(height * resolution, height_m, abs_tol=1e-9)
    ):
        raise ValueError('Resolution must divide both arena dimensions exactly.')
    if width * height > 25_000_000:
        raise ValueError('Requested grid exceeds 25 million cells.')

    # PGM: black=occupied, white=free. ROS map origin is bottom-left,
    # whereas the first image row is the top (maximum world y).
    pixels = bytearray([255]) * (width * height)
    walls = arena['outer_walls_axis_aligned'] + arena['inner_walls_axis_aligned']
    for wall in walls:
        x, y, w, h = (float(wall[k]) for k in ('x', 'y', 'w', 'h'))
        if not all(math.isfinite(v) for v in (x, y, w, h)):
            raise ValueError('Wall coordinates must be finite.')
        if w <= 0 or h <= 0 or min(x, y) < 0 or x+w > width_m+1e-9 or y+h > height_m+1e-9:
            raise ValueError(f'Invalid or out-of-bounds wall: {wall}')
        # Occupy every cell overlapping a wall; epsilon avoids floating-point
        # expansion at exactly aligned boundaries. At 1 cm all supplied edges
        # are exact grid boundaries, preserving the prescribed door widths.
        x0 = max(0, math.floor(x / resolution + 1e-9))
        x1 = min(width, math.ceil((x+w) / resolution - 1e-9))
        y0 = max(0, math.floor(y / resolution + 1e-9))
        y1 = min(height, math.ceil((y+h) / resolution - 1e-9))
        for gy in range(y0, y1):
            start = (height - 1 - gy) * width + x0
            pixels[start:start + x1-x0] = bytes(x1-x0)

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = output_dir / 'arena_map.pgm'
    yaml_path = output_dir / 'arena_map.yaml'
    pgm_path.write_bytes(f'P5\n{width} {height}\n255\n'.encode('ascii') + pixels)
    yaml_path.write_text(
        'image: arena_map.pgm\n'
        'mode: trinary\n'
        f'resolution: {resolution:.12g}\n'
        'origin: [0.0, 0.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n', encoding='utf-8'
    )
    print(f'Created {pgm_path} and {yaml_path}')
    print(f'{width} x {height} cells; {resolution:g} m/cell; {len(walls)} walls')
    print('Wall-only localisation map; blocks are intentionally excluded.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('specs', type=Path, help='Path to MiniLab1.2_platform_specs_5112.json')
    parser.add_argument('--output-dir', type=Path, default=Path('maps'))
    parser.add_argument('--resolution', type=float, default=0.01, help='Metres per cell (default: 0.01)')
    args = parser.parse_args()
    generate(args.specs, args.output_dir, args.resolution)


if __name__ == '__main__':
    main()
