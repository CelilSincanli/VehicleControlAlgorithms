# Audit harness (moved)

The audit tools used for the reports in this folder now live with the other
tests and are built by CMake:

| Then (audit v1/v2)                     | Now                                   |
|----------------------------------------|---------------------------------------|
| `docs/audit/harness/sim_harness.cpp`   | `tests/sim_harness.cpp` (target `sim_harness`) |
| `docs/audit/harness/edge_tests.cpp`    | `tests/edge_tests.cpp` (target `edge_tests`, run by `ctest`) |
| `docs/audit/harness/run_matrix.py`     | `tests/run_matrix.py`                 |

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
ctest --test-dir build --output-on-failure
python3 tests/run_matrix.py build results          # all experiments (E1..E7)
```

The harness now uses the same simulation rules as the GUI (`src/sim/sim_rules.cpp`)
instead of a copy, so the v1/v2 report numbers are not directly comparable with
runs of the current code (maps are now in metres, runs stop on collision).
