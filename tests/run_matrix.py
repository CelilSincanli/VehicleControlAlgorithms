#!/usr/bin/env python3
"""Run the scenario matrix and sweeps with the sim_harness binary.

Usage: python3 tests/run_matrix.py BUILD_DIR OUT_DIR [EXPERIMENTS]
  BUILD_DIR    directory containing the sim_harness executable
  OUT_DIR      where results.jsonl (+ traces/ for E1) is written
  EXPERIMENTS  comma list, default E1,E2,E3,E4,E5,E6,E7
    E1 baseline (3 maps x 2 vehicles x 6 algorithms x 2 fidelities, traces kept)
    E2 requested speed 2/5/8/12 m/s          E3 frame time 144/30/15/10 fps + jitter
    E4 algorithm rebuild at t = 8 s          E5 fidelity switch at t = 8 s
    E6 1-10 um start offsets (sensitivity)   E7 uncapped Dynamic speed sweep (calibration)
A run is clean when it ends with "outcome": "goal reached".
"""
import itertools, json, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
H = os.path.join(os.path.abspath(sys.argv[1]), "sim_harness")
OUT = os.path.abspath(sys.argv[2])
ONLY = set((sys.argv[3] if len(sys.argv) > 3 else "E1,E2,E3,E4,E5,E6,E7").split(","))
os.makedirs(os.path.join(OUT, "traces"), exist_ok=True)

MAPS = ["map_01", "map_02", "map_03"]
VEH = ["vehicle01", "vehicle02"]
ALGOS = ["Pure Pursuit", "Adaptive Pure Pursuit", "LQR", "Stanley", "MPC", "MPPI"]
FID = ["kin", "dyn"]

jobs = []
def add(tag, m, v, a, f, extra, trace=False):
    args = [H, "--map", f"{REPO}/data/maps/{m}", "--vehicle", f"{REPO}/data/vehicle/{v}.json",
            "--algo", a, "--fidelity", f, "--tag", tag] + extra
    if trace:
        args += ["--trace", os.path.join(OUT, "traces", f"{tag}__{m}__{v}__{a.replace(' ', '_')}__{f}.csv")]
    jobs.append(args)

for m, v, a, f in itertools.product(MAPS, VEH, ALGOS, FID):
    if "E1" in ONLY: add("E1_baseline", m, v, a, f, [], trace=True)
    if "E2" in ONLY:
        for s in (2, 5, 8, 12): add(f"E2_speed_{s}", m, v, a, f, ["--speed", str(s)])
    if "E3" in ONLY:
        for fps in (144, 30, 15, 10): add(f"E3_fps_{fps}", m, v, a, f, ["--frame-dt", str(1.0 / fps)])
        add("E3_jitter", m, v, a, f, ["--jitter", "0.008,0.05", "--seed", "7"])
    if "E4" in ONLY: add("E4_rebuild8s", m, v, a, f, ["--rebuild-at", "8"])
    if "E5" in ONLY: add("E5_switch8s", m, v, a, f, ["--switch-fidelity-at", "8"])
    if "E6" in ONLY:
        for y0 in ("1e-6", "-1e-6", "1e-5", "-1e-5"): add(f"E6_y0_{y0}", m, v, a, f, ["--y0", y0])
for m, v, a in itertools.product(MAPS, VEH, ALGOS):
    if "E7" in ONLY:
        for s in (1, 1.5, 2, 2.5, 3, 3.5, 4, 5, 6, 8): add(f"E7_nocap_{s}", m, v, a, "dyn", ["--speed", str(s), "--no-cap"])

def run(args):
    p = subprocess.run(args, capture_output=True, text=True, timeout=1800)
    line = p.stdout.strip().splitlines()[-1] if p.stdout.strip() else ""
    d = json.loads(line) if line.startswith("{") else {"error": p.stderr[-300:], "rc": p.returncode}
    d["cmd"] = " ".join(args[1:])
    return d

with ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as ex, open(os.path.join(OUT, "results.jsonl"), "w") as fo:
    for d in ex.map(run, jobs):
        fo.write(json.dumps(d) + "\n")
print(f"{len(jobs)} runs -> {OUT}/results.jsonl")
