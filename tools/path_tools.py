"""Shared helpers for the map path tools. Maps are authored in metres.

check_path(): verifies a path against both shipped vehicles exactly the way
the simulation sees them:
  * tightest bend (Menger curvature over +-2 waypoints) >= 1.5 x the vehicle's
    minimum turning radius (minimum_turning_radius is authoritative, capped by
    the steering travel, as in vehicle::TireAngleLimits)
  * body rectangle (rear-axle referenced, as sim::BodyCorners) keeps at least
    MIN_CLEARANCE metres from every obstacle and stays inside the world.
"""
import json, math, os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MIN_CLEARANCE = 0.8   # [m]
RADIUS_MARGIN = 1.5   # path radius / vehicle turning radius


def load_vehicles():
    out = []
    for f in ("vehicle01.json", "vehicle02.json"):
        out.append(json.load(open(os.path.join(REPO, "data/vehicle", f))))
    return out


def vehicle_min_radius(v):
    steer = math.radians(v['max_steering_wheel_angle'] / v['steering_wheel_to_tire_angle_ratio'])
    by_radius = math.atan(v['wheelbase'] / v['minimum_turning_radius']) if v.get('minimum_turning_radius', 0) > 0 else steer
    return v['wheelbase'] / math.tan(min(steer, by_radius))


def menger(a, b, c):
    ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
    cr = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return 2 * cr / (ab * bc * ca) if ab * bc * ca > 0 else 0.0


def footprint(x, y, h, v):
    front = v['wheelbase'] + v['front_overhang']
    rear = v['rear_overhang']
    half = v['wheel_tread'] / 2 + max(v['left_overhang'], v['right_overhang'])
    c, s = math.cos(h), math.sin(h)
    return [(x + f * c - l * s, y + f * s + l * c) for f, l in ((front, half), (front, -half), (-rear, -half), (-rear, half))]


def quad_rect_gap(q, ob):
    """Largest separating gap between a quad and an AABB (<= 0 means overlap)."""
    rect = [(ob['x'], ob['y']), (ob['x'] + ob['width'], ob['y']),
            (ob['x'] + ob['width'], ob['y'] + ob['height']), (ob['x'], ob['y'] + ob['height'])]
    best = -1e9
    for ax in [(1, 0), (0, 1), (q[1][0] - q[0][0], q[1][1] - q[0][1]), (q[3][0] - q[0][0], q[3][1] - q[0][1])]:
        n = math.hypot(*ax); ax = (ax[0] / n, ax[1] / n)
        pa = [p[0] * ax[0] + p[1] * ax[1] for p in q]
        pb = [p[0] * ax[0] + p[1] * ax[1] for p in rect]
        best = max(best, max(min(pb) - max(pa), min(pa) - max(pb)))
    return best


def check_path(path, map_json, verbose=True):
    ks = [menger(path[i - 2], path[i], path[i + 2]) for i in range(2, len(path) - 2)]
    kmax = max(abs(k) for k in ks)
    dk = max(abs(ks[i + 1] - ks[i]) for i in range(len(ks) - 1))
    if verbose:
        print(f"  waypoints={len(path)}  tightest radius={1/kmax:.2f} m  max curvature step/waypoint={dk:.4f} 1/m")
    ok = True
    W, H = map_json['world']['width'], map_json['world']['height']
    for v in load_vehicles():
        rmin = vehicle_min_radius(v)
        gap = 1e9
        for i in range(len(path) - 1):
            h = math.atan2(path[i + 1][1] - path[i][1], path[i + 1][0] - path[i][0])
            q = footprint(path[i][0], path[i][1], h, v)
            for ob in map_json['obstacles']:
                gap = min(gap, quad_rect_gap(q, ob))
            # the run ends within 1 m of the goal (sim::CheckTermination), so the
            # world-edge test stops there, as it does in the simulation
            if math.dist(path[i], path[-1]) > 1.0 and (min(p[0] for p in q) < 0 or min(p[1] for p in q) < 0
                                                     or max(p[0] for p in q) > W or max(p[1] for p in q) > H):
                gap = min(gap, -1.0)
        good = gap >= MIN_CLEARANCE and 1 / kmax >= RADIUS_MARGIN * rmin
        if verbose:
            print(f"  {v['name']}: path radius {1/kmax:.2f} m vs turning radius {rmin:.2f} m ({kmax and 1/kmax/rmin:.2f}x)"
                  f"  min body clearance {gap:.2f} m  {'OK' if good else 'FAIL'}")
        ok &= good
    return ok


def resample(pts, spacing=1.0, end=None):
    out = [pts[0]]; acc = 0.0
    for a, b in zip(pts, pts[1:]):
        acc += math.dist(a, b)
        if acc >= spacing:
            out.append(b); acc = 0.0
    if end is not None:
        if math.dist(out[-1], end) < 0.5 * spacing: out[-1] = end
        else: out.append(end)
    return out


def write_path(map_dir, path, map_name, algorithm):
    json.dump({"map_name": map_name, "algorithm": algorithm,
               "waypoints": [{"x": round(x, 3), "y": round(y, 3)} for x, y in path]},
              open(os.path.join(map_dir, "path.json"), "w"), indent=2)
