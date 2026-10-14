"""Procedural textures for the FOB-3 world.

Textures are generated with a fixed seed so the output is reproducible. The
base ground and the surrounding countryside are drawn from zones.yaml (slab
roads, building pads, farmland, county road, canal), which makes the top-down
render look like an aerial photo of the same layout.
"""

from __future__ import annotations

import os
from typing import Optional, Tuple

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


def _slab_texture(w: int, h: int, px_per_m: float, rng: np.random.Generator,
                  slab_m: float = 3.0) -> Image.Image:
    """Grey concrete slabs with dark joints on a world-aligned grid."""
    arr = _tint((158, 156, 148), _noise(h, w, rng, (24, 6, 2)), 0.12)
    img = Image.fromarray(arr, 'RGB')
    d = ImageDraw.Draw(img)
    step = slab_m * px_per_m
    x = 0.0
    while x < w:
        d.line([(x, 0), (x, h)], fill=(96, 94, 88), width=2)
        x += step
    y = float(h % step)
    while y < h:
        d.line([(0, y), (w, y)], fill=(96, 94, 88), width=2)
        y += step
    return img


# Crop colours (RGB) and row stripe strength for the fields around the base.
CROPS = (
    ((92, 122, 54), 0.10),    # young wheat
    ((190, 170, 98), 0.08),   # ripe wheat
    ((68, 98, 42), 0.18),     # maize
    ((118, 124, 48), 0.20),   # sunflower
    ((112, 88, 64), 0.22),    # ploughed soil
    ((176, 158, 108), 0.12),  # stubble
    ((182, 178, 72), 0.06),   # rapeseed
)


def terrain_texture(layout: BaseLayout, px_per_m: float, seed: Optional[int] = None) -> Image.Image:
    """Countryside around the base: farmland strips, roads, canal, field tracks."""
    sur = layout.surroundings
    (x0, y0) = tuple(sur['bounds']['min'])
    (x1, y1) = tuple(sur['bounds']['max'])
    rng = np.random.default_rng(sur.get('farmland_seed', 21) if seed is None else seed)
    w, h = int((x1 - x0) * px_per_m), int((y1 - y0) * px_per_m)

    def to_px(x: float, y: float) -> Tuple[float, float]:
        return ((x - x0) * px_per_m, (y1 - y) * px_per_m)

    img = Image.fromarray(_tint((104, 112, 70), _noise(h, w, rng, (64, 16, 4)), 0.2), 'RGB')

    # Long north-south strips split into parcels, each with its own crop.
    xx = np.arange(w, dtype=np.float32)[None, :] / px_per_m
    x = x0
    while x < x1:
        width = float(rng.uniform(9.0, 20.0))
        y = y0
        while y < y1:
            length = float(rng.uniform(25.0, 60.0))
            color, stripes = CROPS[int(rng.integers(len(CROPS)))]
            c0, c1 = int((x - x0) * px_per_m), int((min(x + width, x1) - x0) * px_per_m)
            r0, r1 = int((y1 - min(y + length, y1)) * px_per_m), int((y1 - y) * px_per_m)
            if c1 > c0 and r1 > r0:
                patch = _tint(color, _noise(r1 - r0, c1 - c0, rng, (32, 8, 2)), 0.12)
                period = 0.75
                rows = 1.0 + stripes * np.sin(2 * np.pi * xx[:, c0:c1] / period)
                patch = np.clip(patch.astype(np.float32) * rows[..., None], 0, 255).astype(np.uint8)
                img.paste(Image.fromarray(patch, 'RGB'), (c0, r0))
            y += length + 1.5   # field track between parcels
        x += width + 0.8        # narrow boundary between strips

    # Mown grass strip around the base (the base ground box covers the inside).
    gx0, gy1 = to_px(-4.0, layout.height + 4.0)
    gx1, gy0 = to_px(layout.width + 4.0, -2.0)
    gw, gh = int(gx1 - gx0), int(gy0 - gy1)
    grass = _tint((106, 112, 72), _noise(gh, gw, rng, (48, 12, 3)), 0.2)
    img.paste(Image.fromarray(grass, 'RGB'), (int(gx0), int(gy1)))

    d = ImageDraw.Draw(img)
    # Irrigation canal with grassy banks.
    canal = sur.get('canal')
    if canal:
        cx, cw = canal['x'], canal['width']
        d.rectangle([to_px(cx - cw / 2 - 1.5, y1), to_px(cx + cw / 2 + 1.5, y0)], fill=(96, 110, 66))
        d.rectangle([to_px(cx - cw / 2, y1), to_px(cx + cw / 2, y0)], fill=(52, 72, 66))
    # County road: asphalt, white edge lines and a dashed centre line.
    road = sur.get('county_road')
    if road:
        ry, rw = road['y'], road['width']
        d.rectangle([to_px(x0, ry + rw / 2 + 1.0), to_px(x1, ry - rw / 2 - 1.0)], fill=(128, 120, 96))
        d.rectangle([to_px(x0, ry + rw / 2), to_px(x1, ry - rw / 2)], fill=(64, 66, 68))
        lw = max(1, int(0.15 * px_per_m))
        for edge in (ry + rw / 2 - 0.3, ry - rw / 2 + 0.3):
            d.line([to_px(x0, edge), to_px(x1, edge)], fill=(220, 220, 214), width=lw)
        xd = x0
        while xd < x1:
            d.line([to_px(xd, ry), to_px(xd + 3.0, ry)], fill=(220, 220, 214), width=lw)
            xd += 9.0
    # Access road from the county road to the main gate.
    access = sur.get('access_road')
    if access and road:
        ax, aw = access['x'], access['width']
        d.rectangle([to_px(ax - aw / 2, layout.fence_min[1]), to_px(ax + aw / 2, road['y'] + road['width'] / 2)],
                    fill=(70, 72, 72))
    return img.filter(ImageFilter.SMOOTH)


def ground_texture(layout: BaseLayout, px_per_m: float, seed: int = 7) -> Image.Image:
    rng = np.random.default_rng(seed)
    w = int(layout.width * px_per_m)
    h = int(layout.height * px_per_m)
    grass = _tint((112, 110, 74), _noise(h, w, rng, (96, 24, 6)), 0.24)
    img = Image.fromarray(grass, 'RGB')

    def to_px(x: float, y: float) -> Tuple[float, float]:
        return (x * px_per_m, (layout.height - y) * px_per_m)

    # Mown grass and bare patches inside the fence.
    dirt = Image.fromarray(_tint((118, 116, 78), _noise(h, w, rng, (80, 20, 5)), 0.22), 'RGB')
    mask = Image.new('L', (w, h), 0)
    md = ImageDraw.Draw(mask)
    (fx0, fy0), (fx1, fy1) = layout.fence_min, layout.fence_max
    md.rectangle([to_px(fx0 + 0.3, fy1 - 0.3), to_px(fx1 - 0.3, fy0 + 0.3)], fill=200)
    mask = mask.filter(ImageFilter.GaussianBlur(px_per_m * 0.6))
    img = Image.composite(dirt, img, mask)

    # Concrete slab roads (typical of Romanian military sites), plus the access
    # road from the county road to the main gate on the south fence.
    slabs = _slab_texture(w, h, px_per_m, rng)
    road_mask = Image.new('L', (w, h), 0)
    rd = ImageDraw.Draw(road_mask)
    road_px = int(layout.road_width * px_per_m)
    for line in layout.roads:
        pts = [to_px(*p) for p in line]
        rd.line(pts, fill=255, width=road_px)
        for p in pts:
            r = road_px / 2
            rd.rectangle([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill=255)
    access = layout.surroundings.get('access_road')
    if access:
        half = access['width'] / 2.0
        rd.rectangle([to_px(access['x'] - half, layout.fence_min[1]),
                      to_px(access['x'] + half, 0.0)], fill=255)
    img = Image.composite(slabs, img, road_mask)

    # Concrete pads around buildings.
    pad = Image.fromarray(_tint((160, 158, 150), _noise(h, w, rng, (32, 8, 2)), 0.1), 'RGB')
    pad_mask = Image.new('L', (w, h), 0)
    pd = ImageDraw.Draw(pad_mask)
    for b in layout.buildings:
        pd.rectangle([to_px(b.min[0] - 0.6, b.max[1] + 0.6), to_px(b.max[0] + 0.6, b.min[1] - 0.6)],
                     fill=230)
    img = Image.composite(pad, img, pad_mask)

    # Helipad: concrete disc, painted circle and H.
    if layout.helipad:
        (hx, hy), r = layout.helipad
        d = ImageDraw.Draw(img)
        cx, cy = to_px(hx, hy)
        rp = r * px_per_m
        d.ellipse([cx - rp, cy - rp, cx + rp, cy + rp], fill=(150, 148, 140))
        ring = rp * 0.85
        d.ellipse([cx - ring, cy - ring, cx + ring, cy + ring], outline=(230, 226, 210),
                  width=max(2, int(px_per_m * 0.12)))
        hw, hh, t = rp * 0.32, rp * 0.45, max(2, int(px_per_m * 0.16))
        for x in (cx - hw, cx + hw - t):
            d.rectangle([x, cy - hh, x + t, cy + hh], fill=(230, 226, 210))
        d.rectangle([cx - hw, cy - t / 2, cx + hw, cy + t / 2], fill=(230, 226, 210))
    return img.filter(ImageFilter.SMOOTH)


def wall_texture(kind: str, size: int = 512, seed: int = 11) -> Image.Image:
    rng = np.random.default_rng(seed + sum(ord(c) for c in kind))
    if kind == 'concrete':
        arr = _tint((176, 162, 128), _noise(size, size, rng, (64, 16, 3)), 0.12)
        img = Image.fromarray(arr, 'RGB')
        d = ImageDraw.Draw(img)
        for y in range(0, size, size // 6):  # panel joints
            d.line([(0, y), (size, y)], fill=(130, 128, 120), width=2)
        return img
    if kind in ('metal', 'container_green', 'container_tan'):
        base = {'metal': (104, 112, 86), 'container_green': (78, 92, 64),
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
    if kind == 'hesco':
        # Sand-filled fabric cages with a wire grid.
        arr = _tint((170, 152, 110), _noise(size, size, rng, (16, 6, 2)), 0.18)
        img = Image.fromarray(arr, 'RGB')
        d = ImageDraw.Draw(img)
        for i in range(0, size, size // 8):
            d.line([(i, 0), (i, size)], fill=(110, 100, 78), width=2)
            d.line([(0, i), (size, i)], fill=(110, 100, 78), width=2)
        return img
    if kind in ('plaster_cream', 'plaster_ochre'):
        base = {'plaster_cream': (214, 204, 172), 'plaster_ochre': (204, 172, 112)}[kind]
        img = Image.fromarray(_tint(base, _noise(size, size, rng, (64, 16, 3)), 0.06), 'RGB')
        d = ImageDraw.Draw(img)
        d.rectangle([0, int(size * 0.86), size, size], fill=(120, 118, 110))   # plinth
        n = 5
        ww, wh = size / (n * 1.9), size * 0.32
        for k in range(n):
            cx = size * (k + 0.5) / n
            box = [cx - ww / 2, size * 0.30, cx + ww / 2, size * 0.30 + wh]
            d.rectangle(box, fill=(238, 236, 228))
            d.rectangle([box[0] + 5, box[1] + 5, box[2] - 5, box[3] - 5], fill=(58, 70, 82))
            d.line([(cx, box[1] + 5), (cx, box[3] - 5)], fill=(238, 236, 228), width=3)
        return img
    if kind == 'roof_tile':
        img = Image.fromarray(_tint((148, 66, 44), _noise(size, size, rng, (16, 4, 2)), 0.14), 'RGB')
        d = ImageDraw.Draw(img)
        for y in range(0, size, size // 24):
            d.line([(0, y), (size, y)], fill=(96, 40, 28), width=2)
        return img
    if kind in ('roof_tin_green', 'hangar_metal'):
        base = {'roof_tin_green': (72, 104, 74), 'hangar_metal': (118, 124, 108)}[kind]
        arr = _tint(base, _noise(size, size, rng, (48, 12, 3)), 0.06).astype(np.float32)
        ribs = (np.sin(np.linspace(0, 48 * np.pi, size)) * 0.12 + 1.0)[None, :, None]
        return Image.fromarray(np.clip(arr * ribs, 0, 255).astype(np.uint8), 'RGB')
    if kind == 'bunker_roof':
        arr = _tint((96, 108, 62), _noise(size, size, rng, (32, 8, 2)), 0.25)
        return Image.fromarray(arr, 'RGB')
    if kind == 'flag_ro':
        img = Image.new('RGB', (size, size))
        d = ImageDraw.Draw(img)
        for k, col in enumerate(((0, 43, 127), (252, 209, 22), (206, 17, 38))):
            d.rectangle([k * size // 3, 0, (k + 1) * size // 3, size], fill=col)
        return img
    if kind == 'gate':
        img = Image.new('RGB', (size, size), (54, 72, 52))
        d = ImageDraw.Draw(img)
        for x in range(0, size, size // 16):
            d.rectangle([x, 0, x + 6, size], fill=(36, 48, 36))
        return img
    if kind == 'camo_net':
        # Woodland pattern: blobs of three colours over olive.
        img = Image.new('RGB', (size, size), (86, 96, 62))
        d = ImageDraw.Draw(img)
        for color in ((60, 70, 44), (120, 110, 76), (40, 44, 34)):
            for _ in range(60):
                x, y = rng.integers(0, size, 2)
                rx, ry = rng.integers(size // 40, size // 12, 2)
                d.ellipse([x - rx, y - ry, x + rx, y + ry], fill=color)
        return img.filter(ImageFilter.GaussianBlur(1.5))
    raise ValueError(f'unknown texture kind {kind}')


TEXTURE_KINDS = ('concrete', 'metal', 'container_green', 'container_tan', 'bunker',
                 'sandbags', 'vehicle', 'tower', 'hesco', 'camo_net', 'plaster_cream',
                 'plaster_ochre', 'roof_tile', 'roof_tin_green', 'hangar_metal', 'bunker_roof',
                 'flag_ro', 'gate')


def write_textures(layout: BaseLayout, out_dir: str, ground_px_per_m: float = 40.0) -> None:
    os.makedirs(out_dir, exist_ok=True)
    # Gazebo maps the texture onto the top face of the ground box rotated 90 deg
    # clockwise (checked with a quadrant calibration image), so store it rotated
    # 90 deg counter-clockwise to appear north-up in the simulation.
    ground = ground_texture(layout, ground_px_per_m)
    ground.transpose(Image.Transpose.ROTATE_90).save(os.path.join(out_dir, 'ground.png'),
                                                     optimize=True)
    if layout.surroundings:
        terrain = terrain_texture(layout, 10.0)
        terrain.transpose(Image.Transpose.ROTATE_90).save(os.path.join(out_dir, 'terrain.png'),
                                                          optimize=True)
    for kind in TEXTURE_KINDS:
        wall_texture(kind).save(os.path.join(out_dir, f'{kind}.png'), optimize=True)
