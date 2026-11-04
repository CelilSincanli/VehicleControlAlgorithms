#!/usr/bin/env python3
"""Regenerate data/maps/map_02/path.json (Sine Wave Map) as a drivable weave.

The original path was a true sine (amplitude 24 m, period 54 m) whose tightest bend
(~3 m) was tighter than Car 01 can turn, and whose curvature changed faster than the
rate-limited steering actuator can follow.

Construction (all in metres; maps are authored in metres):
  1. Same course: start (6, 60) -> over baffle 1 -> under baffle 2 -> over 3 -> under 4 -> goal (114, 60).
     Crest/valley apexes sit at the baffle centres (x = 19.5, 46.5, 73.5, 100.5).
  2. Heading profile made of straights and arcs of radius R (solved so the apexes land
     exactly on the centres at the chosen heights).
  3. Heading box-filtered over SMOOTH units: curvature then ramps linearly into each arc
     (clothoid transition) instead of stepping, so the path is curvature-continuous.
  4. Re-integrated, end error distributed linearly, resampled every 1.0 unit.
  5. Verified by tools/path_tools.py: tightest radius vs both vehicles' turning radius and
     body clearance of BOTH vehicles in metres (body rectangle as the simulation uses it).

Usage: python3 tools/generate_map02_path.py [--check-only]
"""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from path_tools import check_path, write_path

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP = os.path.join(REPO, "data/maps/map_02")
R = 13.0          # arc radius [m]
SMOOTH = 6.0      # clothoid transition length [m]
Y_CREST, Y_VALLEY = 77.0, 43.0
START, GOAL = (6.0, 60.0), (114.0, 60.0)
CRESTS, VALLEYS = (19.5, 73.5), (46.5, 100.5)
DS = 0.02


def solve(dx, dy, r):
    """Straight at heading th followed by an arc of radius r turning back to 0:
    dx = L cos th + r sin th, dy = L sin th + r (1 - cos th). Returns (th, L)."""
    lo, hi = 1e-4, math.pi / 2 - 1e-4
    f = lambda th: (dx - r * math.sin(th)) * math.tan(th) + r * (1 - math.cos(th)) - dy
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if f(lo) * f(mid) <= 0: hi = mid
        else: lo = mid
    th = 0.5 * (lo + hi)
    L = (dx - r * math.sin(th)) / math.cos(th)
    if L < 0: raise SystemExit(f"infeasible: dx={dx}, dy={dy}, R={r}")
    return th, L


def heading_profile():
    """List of (length, curvature) segments, starting heading psi0."""
    th_s, L_s = solve(CRESTS[0] - START[0], Y_CREST - START[1], R)           # start -> crest 1
    half = (VALLEYS[0] - CRESTS[0]) / 2, (Y_CREST - Y_VALLEY) / 2               # apex -> mid-point
    th_m, L_m = solve(half[0], half[1], R)
    segs = [(L_s, 0.0), (R * th_s, -1 / R)]                                     # climb, crest arc (1st half)
    apexes = [CRESTS[0], VALLEYS[0], CRESTS[1], VALLEYS[1]]
    for i in range(3):                                                          # crest->valley->crest->valley
        down = (i % 2 == 0)
        k = -1 / R if down else 1 / R
        segs += [(R * th_m, k), (2 * L_m, 0.0), (R * th_m, -k)]
    th_e, L_e = solve(GOAL[0] - VALLEYS[1], GOAL[1] - Y_VALLEY, R)               # valley 2 -> goal
    segs += [(R * th_e, 1 / R), (L_e, 0.0)]
    return th_s, segs


def build():
    psi0, segs = heading_profile()
    kappa = []
    for L, k in segs:
        kappa += [k] * max(1, int(round(L / DS)))
    n = len(kappa)
    # box filter on curvature == linear curvature ramps (clothoids) of length SMOOTH
    w = int(round(SMOOTH / DS))
    pref = [0.0]
    for k in kappa: pref.append(pref[-1] + k)
    ks = [(pref[min(n, i + w // 2)] - pref[max(0, i - w // 2)]) / (min(n, i + w // 2) - max(0, i - w // 2)) for i in range(n)]
    # integrate
    pts = [START]; psi = psi0; x, y = START
    for k in ks:
        psi += k * DS
        x += math.cos(psi) * DS; y += math.sin(psi) * DS
        pts.append((x, y))
    ex, ey = GOAL[0] - pts[-1][0], GOAL[1] - pts[-1][1]
    pts = [(px + ex * i / (len(pts) - 1), py + ey * i / (len(pts) - 1)) for i, (px, py) in enumerate(pts)]
    # resample every 1.0 unit of arc length
    out = [pts[0]]; acc = 0.0
    for a, b in zip(pts, pts[1:]):
        acc += math.dist(a, b)
        if acc >= 1.0: out.append(b); acc = 0.0
    if math.dist(out[-1], GOAL) < 0.5: out[-1] = GOAL
    else: out.append(GOAL)
    return out, (ex, ey)


def check(path):
    return check_path(path, json.load(open(os.path.join(MAP, "map.json"))))


if __name__ == "__main__":
    if "--check-only" in sys.argv:
        p = [(w['x'], w['y']) for w in json.load(open(os.path.join(MAP, "path.json")))['waypoints']]
        sys.exit(0 if check(p) else 1)
    path, err = build()
    print(f"closure correction applied: ({err[0]:+.3f}, {err[1]:+.3f}) units")
    if not check(path): raise SystemExit("constraints not met - path NOT written")
    write_path(MAP, path, "Sine Wave Map", "arc-clothoid weave (tools/generate_map02_path.py)")
    print("wrote", os.path.join(MAP, "path.json"))
