"""Procedural textures for the FOB-3 world.

Textures are generated with a fixed seed so the output is reproducible. The
ground texture is drawn from zones.yaml (roads, building pads, outside scrub),
which makes the top-down render look like an aerial photo of the same layout.
"""

from __future__ import annotations

import os
from typing import Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

from perimeter_defense_world.geometry import BaseLayout

RGB = Tuple[int, int, int]


def _noise(h: int, w: int, rng: np.random.Generator, scales=(64, 16, 4)) -> np.ndarray:
    """Multi-octave value noise in [0, 1]."""
    out = np.zeros((h, w), dtype=np.float32)
    total = 0.0
    for i, s in enumerate(scales):
        gh, gw = max(2, h // s + 2), max(2, w // s + 2)
        coarse = rng.random((gh, gw)).astype(np.float32)
        img = Image.fromarray((coarse * 255).astype(np.uint8)).resize((w, h), Image.BICUBIC)
        weight = 1.0 / (i + 1)
        out += np.asarray(img, dtype=np.float32) / 255.0 * weight
        total += weight
    return out / total


def _tint(base: RGB, noise: np.ndarray, amount: float) -> np.ndarray:
    arr = np.empty(noise.shape + (3,), dtype=np.float32)
    for c in range(3):
        arr[..., c] = base[c] * (1.0 - amount + 2.0 * amount * noise)
    return np.clip(arr, 0, 255).astype(np.uint8)


def ground_texture(layout: BaseLayout, px_per_m: float, seed: int = 7) -> Image.Image:
    rng = np.random.default_rng(seed)
    w = int(layout.width * px_per_m)
    h = int(layout.height * px_per_m)
    grass = _tint((104, 112, 72), _noise(h, w, rng, (96, 24, 6)), 0.22)
    img = Image.fromarray(grass, 'RGB')

    def to_px(x: float, y: float) -> Tuple[float, float]:
        return (x * px_per_m, (layout.height - y) * px_per_m)

    # Compacted dirt inside the fence.
    dirt = Image.fromarray(_tint((128, 118, 92), _noise(h, w, rng, (80, 20, 5)), 0.18), 'RGB')
    mask = Image.new('L', (w, h), 0)
    md = ImageDraw.Draw(mask)
    (fx0, fy0), (fx1, fy1) = layout.fence_min, layout.fence_max
    md.rectangle([to_px(fx0 + 0.3, fy1 - 0.3), to_px(fx1 - 0.3, fy0 + 0.3)], fill=200)
    mask = mask.filter(ImageFilter.GaussianBlur(px_per_m * 0.6))
    img = Image.composite(dirt, img, mask)

    # Gravel roads.
    gravel = Image.fromarray(_tint((150, 146, 136), _noise(h, w, rng, (12, 4, 2)), 0.25), 'RGB')
    road_mask = Image.new('L', (w, h), 0)
    rd = ImageDraw.Draw(road_mask)
    road_px = int(layout.road_width * px_per_m)
    for line in layout.roads:
        pts = [to_px(*p) for p in line]
        rd.line(pts, fill=255, width=road_px, joint='curve')
        for p in pts:
            r = road_px / 2
            rd.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill=255)
    road_mask = road_mask.filter(ImageFilter.GaussianBlur(px_per_m * 0.15))
    img = Image.composite(gravel, img, road_mask)

    # Concrete pads around buildings.
    pad = Image.fromarray(_tint((160, 158, 150), _noise(h, w, rng, (32, 8, 2)), 0.1), 'RGB')
    pad_mask = Image.new('L', (w, h), 0)
    pd = ImageDraw.Draw(pad_mask)
    for b in layout.buildings:
        pd.rectangle([to_px(b.min[0] - 0.6, b.max[1] + 0.6), to_px(b.max[0] + 0.6, b.min[1] - 0.6)],
                     fill=230)
    img = Image.composite(pad, img, pad_mask)

    return img.filter(ImageFilter.SMOOTH)


def wall_texture(kind: str, size: int = 512, seed: int = 11) -> Image.Image:
    rng = np.random.default_rng(seed + sum(ord(c) for c in kind))
    if kind == 'concrete':
        arr = _tint((168, 165, 156), _noise(size, size, rng, (64, 16, 3)), 0.12)
        img = Image.fromarray(arr, 'RGB')
        d = ImageDraw.Draw(img)
        for y in range(0, size, size // 6):  # panel joints
            d.line([(0, y), (size, y)], fill=(130, 128, 120), width=2)
        return img
    if kind in ('metal', 'container_green', 'container_tan'):
        base = {'metal': (150, 154, 156), 'container_green': (78, 92, 64),
                'container_tan': (164, 140, 98)}[kind]
        arr = _tint(base, _noise(size, size, rng, (48, 12, 3)), 0.08).astype(np.float32)
        ribs = (np.sin(np.linspace(0, 40 * np.pi, size)) * 0.12 + 1.0)[None, :, None]
        return Image.fromarray(np.clip(arr * ribs, 0, 255).astype(np.uint8), 'RGB')
    if kind == 'bunker':
        arr = _tint((112, 110, 98), _noise(size, size, rng, (32, 8, 2)), 0.2)
        return Image.fromarray(arr, 'RGB')
    if kind == 'sandbags':
        arr = _tint((176, 158, 116), _noise(size, size, rng, (8, 4, 2)), 0.2)
        img = Image.fromarray(arr, 'RGB')
        d = ImageDraw.Draw(img)
        rows = 6
        for r in range(rows):
            y = r * size // rows
            d.line([(0, y), (size, y)], fill=(120, 106, 76), width=3)
            off = (size // 8) * (r % 2)
            for x in range(off, size, size // 4):
                d.line([(x, y), (x, y + size // rows)], fill=(130, 116, 84), width=2)
        return img
    if kind == 'vehicle':
        arr = _tint((84, 90, 64), _noise(size, size, rng, (64, 16, 4)), 0.1)
        return Image.fromarray(arr, 'RGB')
    if kind == 'tower':
        arr = _tint((96, 84, 64), _noise(size, size, rng, (16, 4, 2)), 0.2)
        return Image.fromarray(arr, 'RGB')
    raise ValueError(f'unknown texture kind {kind}')


TEXTURE_KINDS = ('concrete', 'metal', 'container_green', 'container_tan', 'bunker',
                 'sandbags', 'vehicle', 'tower')


def write_textures(layout: BaseLayout, out_dir: str, ground_px_per_m: float = 40.0) -> None:
    os.makedirs(out_dir, exist_ok=True)
    # Gazebo maps the texture onto the top face of the ground box rotated 90 deg
    # clockwise (checked with a quadrant calibration image), so store it rotated
    # 90 deg counter-clockwise to appear north-up in the simulation.
    ground = ground_texture(layout, ground_px_per_m)
    ground.transpose(Image.Transpose.ROTATE_90).save(os.path.join(out_dir, 'ground.png'),
                                                     optimize=True)
    for kind in TEXTURE_KINDS:
        wall_texture(kind).save(os.path.join(out_dir, f'{kind}.png'), optimize=True)
