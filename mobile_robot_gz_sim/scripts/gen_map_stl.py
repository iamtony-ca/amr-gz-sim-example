#!/usr/bin/env python3
"""Regenerate the gz world mesh (map.stl) from the Nav2 occupancy map (pgm).

The Nav2 map (depot.yaml + depot_edit.pgm) is the single source of truth:
AMCL, the costmaps and the planners all use it. map.stl is only the gz
physics/sensor world. Whenever the two disagree (e.g. the pgm was edited, or
a new map was dropped in), regenerate the STL from the pgm with this tool so
the two are metrically identical by construction.

Each occupied pixel becomes a wall cell, extruded 0..WALL_HEIGHT m. Cells are
centred on the occupied-cell reference point (origin + index*resolution) that
nav2_amcl uses, so the lidar sees walls exactly where AMCL expects them.
Only faces exposed to a non-occupied neighbour are emitted (keeps it light).

Usage:
    python3 gen_map_stl.py [map.yaml] [out.stl]

Defaults (resolved relative to this script's package):
    map.yaml -> ../maps/depot.yaml
    out.stl  -> ../models/map/meshes/map.stl
"""
import os
import struct
import sys

WALL_HEIGHT = 2.0  # metres; must exceed the lidar height (0.19 m)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def parse_map_yaml(path):
    """Minimal parser for the flat Nav2 map yaml (no PyYAML dependency)."""
    cfg = {}
    for line in open(path):
        line = line.split('#', 1)[0].strip()
        if ':' not in line:
            continue
        key, val = line.split(':', 1)
        cfg[key.strip()] = val.strip()
    image = cfg['image'].strip('"\'')
    resolution = float(cfg['resolution'])
    origin = [float(v) for v in cfg['origin'].strip('[]').split(',')]
    occupied_thresh = float(cfg.get('occupied_thresh', 0.65))
    negate = int(cfg.get('negate', 0))
    return image, resolution, origin, occupied_thresh, negate


def read_pgm(path):
    """Read a binary (P5) pgm into (width, height, bytes)."""
    data = open(path, 'rb').read()
    assert data[:2] == b'P5', f'{path} is not a binary (P5) pgm'
    i, toks = 2, []
    while len(toks) < 3:                       # width, height, maxval
        while data[i] in b' \t\r\n':
            i += 1
        if data[i:i + 1] == b'#':              # skip comment line
            while data[i] not in b'\r\n':
                i += 1
            continue
        j = i
        while data[j] not in b' \t\r\n':
            j += 1
        toks.append(int(data[i:j]))
        i = j
    w, h, _ = toks
    px = data[i + 1:i + 1 + w * h]             # one whitespace, then pixels
    assert len(px) == w * h, f'pixel count {len(px)} != {w}*{h}'
    return w, h, px


def generate(map_yaml, out_stl):
    image, res, origin, occ_thresh, negate = parse_map_yaml(map_yaml)
    ox, oy = origin[0], origin[1]
    pgm_path = os.path.join(os.path.dirname(map_yaml), image)
    W, H, px = read_pgm(pgm_path)

    # map_server occupancy: negate=0 -> p=(255-val)/255 ; negate=1 -> p=val/255.
    # A cell is occupied when p > occupied_thresh.
    if negate:
        def is_occ(v):
            return v > 255.0 * occ_thresh
    else:
        def is_occ(v):
            return v < 255.0 * (1.0 - occ_thresh)

    def occ(c, r):
        return 0 <= c < W and 0 <= r < H and is_occ(px[r * W + c])

    print(f'pgm  : {pgm_path}  {W}x{H} @ {res} m  -> {W*res:.2f} x {H*res:.2f} m')

    tris = []  # each: (normal, v0, v1, v2)

    def quad(n, a, b, c, d):
        tris.append((n, a, b, c))
        tris.append((n, a, c, d))

    nocc = 0
    for r in range(H):
        rb = H - 1 - r                          # row index from the bottom
        for c in range(W):
            if not is_occ(px[r * W + c]):
                continue
            nocc += 1
            # Centre the cell box on the occupied-cell reference point
            # (origin + index*res) used by nav2_amcl's likelihood field.
            x0 = ox + c * res - res / 2.0
            y0 = oy + rb * res - res / 2.0
            x1, y1, Z = x0 + res, y0 + res, WALL_HEIGHT
            if not occ(c - 1, r):
                quad((-1, 0, 0), (x0, y1, 0), (x0, y0, 0), (x0, y0, Z), (x0, y1, Z))
            if not occ(c + 1, r):
                quad((1, 0, 0), (x1, y0, 0), (x1, y1, 0), (x1, y1, Z), (x1, y0, Z))
            if not occ(c, r + 1):               # world -y neighbour
                quad((0, -1, 0), (x0, y0, 0), (x1, y0, 0), (x1, y0, Z), (x0, y0, Z))
            if not occ(c, r - 1):               # world +y neighbour
                quad((0, 1, 0), (x1, y1, 0), (x0, y1, 0), (x0, y1, Z), (x1, y1, Z))
            quad((0, 0, 1), (x0, y0, Z), (x1, y0, Z), (x1, y1, Z), (x0, y1, Z))

    with open(out_stl, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(tris)))
        for n, a, b, c in tris:
            f.write(struct.pack('<3f', *n))
            f.write(struct.pack('<3f', *a))
            f.write(struct.pack('<3f', *b))
            f.write(struct.pack('<3f', *c))
            f.write(struct.pack('<H', 0))

    print(f'occupied cells: {nocc}   triangles: {len(tris)}')
    print(f'wrote: {out_stl}')


def main():
    map_yaml = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PKG, 'maps', 'depot.yaml')
    out_stl = sys.argv[2] if len(sys.argv) > 2 else os.path.join(PKG, 'models', 'map', 'meshes', 'map.stl')
    generate(os.path.abspath(map_yaml), os.path.abspath(out_stl))


if __name__ == '__main__':
    main()
