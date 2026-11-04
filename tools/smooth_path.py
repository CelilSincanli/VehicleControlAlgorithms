#!/usr/bin/env python3
"""Make a map's reference path curvature-continuous (clothoid corner transitions).

map_01 and map_03 corners were filleted with a straight line meeting a circular
arc: curvature jumps from 0 to 1/R at the tangent point, which a rate-limited
steering actuator cannot follow. This tool keeps the route and replaces every
curvature step by a linear ramp (a clothoid) of length SMOOTH:

  1. resample the path finely and compute its curvature profile kappa(s)
  2. box-filter kappa(s) over SMOOTH metres -> linear curvature ramps
  3. re-integrate heading and position from the original start pose
  4. distribute the small end-point error linearly so start and goal stay exact
  5. resample every 1 m and check against both vehicles (tools/path_tools.py)

Usage: python3 tools/smooth_path.py map_01 [--smooth 8] [--check-only]
"""
import argparse, json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from path_tools import REPO, check_path, resample, write_path

DS = 0.05


def densify(pts, ds=DS):
    out = []
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.dist(a, b) / ds))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n)]
    out.append(pts[-1])
    return out


def smooth(path, smooth_len):
    dense = densify(path)
    heads = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(dense, dense[1:])]
    lens = [math.dist(a, b) for a, b in zip(dense, dense[1:])]
    kappa = [0.0]
    for i in range(1, len(heads)):
        d = (heads[i] - heads[i - 1] + math.pi) % (2 * math.pi) - math.pi
        kappa.append(d / max(lens[i], 1e-9))
    n, w = len(kappa), max(1, int(round(smooth_len / DS)))
    pref = [0.0]
    for k in kappa: pref.append(pref[-1] + k)
    ks = [(pref[min(n, i + w // 2)] - pref[max(0, i - w // 2)]) / (min(n, i + w // 2) - max(0, i - w // 2)) for i in range(n)]
    pts = [dense[0]]; psi = heads[0]; x, y = dense[0]
    for i in range(n):
        if i > 0: psi += ks[i] * lens[i]
        x += math.cos(psi) * lens[i]; y += math.sin(psi) * lens[i]
        pts.append((x, y))
    ex, ey = path[-1][0] - pts[-1][0], path[-1][1] - pts[-1][1]
    m = len(pts) - 1
    pts = [(px + ex * i / m, py + ey * i / m) for i, (px, py) in enumerate(pts)]
    return resample(pts, 1.0, end=path[-1]), (ex, ey)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("map")
    ap.add_argument("--smooth", type=float, default=8.0, help="clothoid transition length [m]")
    ap.add_argument("--check-only", action="store_true")
    a = ap.parse_args()
    d = os.path.join(REPO, "data/maps", a.map)
    mp = json.load(open(os.path.join(d, "map.json")))
    pj = json.load(open(os.path.join(d, "path.json")))
    path = [(w['x'], w['y']) for w in pj['waypoints']]
    if a.check_only:
        sys.exit(0 if check_path(path, mp) else 1)
    print("original:"); check_path(path, mp)
    new, err = smooth(path, a.smooth)
    print(f"smoothed (closure correction {err[0]:+.3f}, {err[1]:+.3f} m):")
    if not check_path(new, mp):
        raise SystemExit("constraints not met - path NOT written")
    write_path(d, new, mp.get('name', a.map), f"clothoid-smoothed (tools/smooth_path.py --smooth {a.smooth:g})")
    print("wrote", os.path.join(d, "path.json"))
