"""Generate the Nav2 occupancy maps (.pgm + .yaml) directly from zones.yaml.

The maps are rasterised from the same geometry as the Gazebo world, so there
is no SLAM step and no mismatch between the maps and the simulation.

Two maps are written:
  <out>.yaml               navigation map, fence drawn as a solid wall
  <out>_localization.yaml  AMCL map, fence drawn as posts only, because the
                           lidar sees through the chain-link mesh

Usage:
    ros2 run perimeter_defense_world generate_map --out maps/perimeter_base
"""

from __future__ import annotations

import argparse
import math
import os
from typing import List, Optional

import numpy as np

from perimeter_defense_world.geometry import BaseLayout, Rect, Segment, load_layout

FREE = 254
OCCUPIED = 0


class OccupancyRaster:
    """Boolean grid in map coordinates, row 0 is the south edge."""

    def __init__(self, width_m: float, height_m: float, resolution: float) -> None:
        self.resolution = resolution
        self.cols = int(round(width_m / resolution))
        self.rows = int(round(height_m / resolution))
        self.occupied = np.zeros((self.rows, self.cols), dtype=bool)

    def _index_range(self, lo: float, hi: float, limit: int) -> range:
        i0 = max(0, int(math.floor(lo / self.resolution)))
        i1 = min(limit, int(math.ceil(hi / self.resolution)))
        return range(i0, i1)

    def fill_rect(self, x0: float, y0: float, x1: float, y1: float) -> None:
        cols = self._index_range(min(x0, x1), max(x0, x1), self.cols)
        rows = self._index_range(min(y0, y1), max(y0, y1), self.rows)
        if len(cols) and len(rows):
            self.occupied[rows.start:rows.stop, cols.start:cols.stop] = True

    def fill_segment(self, seg: Segment, thickness: float) -> None:
        # Fence sides are axis-aligned, so a thick segment is a rectangle.
        half = max(thickness, 2.0 * self.resolution) / 2.0
        (ax, ay), (bx, by) = seg.a, seg.b
        self.fill_rect(min(ax, bx) - half, min(ay, by) - half,
                       max(ax, bx) + half, max(ay, by) + half)

    def to_pgm_array(self) -> np.ndarray:
        # PGM row 0 is the top (north) edge.
        img = np.where(self.occupied, OCCUPIED, FREE).astype(np.uint8)
        return np.flipud(img)


def rasterize(layout: BaseLayout, resolution: float, solid_fence: bool = True) -> OccupancyRaster:
    grid = OccupancyRaster(layout.width, layout.height, resolution)
    if solid_fence:
        for seg in layout.fence_segments():
            grid.fill_segment(seg, layout.fence_thickness)
    else:
        half = max(layout.post_size, resolution) / 2.0
        for x, y in layout.fence_posts():
            grid.fill_rect(x - half, y - half, x + half, y + half)
    obstacle: Rect
    for obstacle in layout.obstacles():
        grid.fill_rect(obstacle.min[0], obstacle.min[1], obstacle.max[0], obstacle.max[1])
    return grid


def write_pgm(path: str, img: np.ndarray) -> None:
    rows, cols = img.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n# FOB-3 map generated from zones.yaml\n{cols} {rows}\n255\n'.encode())
        f.write(img.tobytes())


def write_maps(layout: BaseLayout, out_prefix: str, resolution: float = 0.05) -> List[str]:
    """Write the navigation and the localization map, return the yaml paths."""
    return [write_map(layout, out_prefix, resolution, solid_fence=True),
            write_map(layout, out_prefix + '_localization', resolution, solid_fence=False)]


def write_map(layout: BaseLayout, out_prefix: str, resolution: float = 0.05,
              solid_fence: bool = True) -> str:
    """Write <out_prefix>.pgm and <out_prefix>.yaml, return the yaml path."""
    grid = rasterize(layout, resolution, solid_fence)
    os.makedirs(os.path.dirname(os.path.abspath(out_prefix)), exist_ok=True)
    pgm_path = out_prefix + '.pgm'
    yaml_path = out_prefix + '.yaml'
    write_pgm(pgm_path, grid.to_pgm_array())
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(
            f'image: {os.path.basename(pgm_path)}\n'
            'mode: trinary\n'
            f'resolution: {resolution}\n'
            'origin: [0.0, 0.0, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n'
        )
    return yaml_path


def main(argv: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--zones', default=None, help='path to zones.yaml')
    parser.add_argument('--out', required=True, help='output prefix, without extension')
    parser.add_argument('--resolution', type=float, default=0.05)
    args = parser.parse_args(argv)
    for path in write_maps(load_layout(args.zones), args.out, args.resolution):
        print(f'map written: {path}')


if __name__ == '__main__':
    main()
