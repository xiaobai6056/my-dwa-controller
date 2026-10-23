# Dynamic Benchmark V0.2 implementation plan

Direct implementation authorized; preserve all current user modifications and V0.1 records.

- [x] Inspect current repository, UI entry point, source/binary differences and raw records.
- [x] Freeze initial existing files and current plugin; new runs are current-workspace diagnostics, not V0.1.
- [x] Add independent single-run GUI/headless wrapper with original scene/motion/runner unchanged, custom RViz config from observed topics; verify two D01 GUI runs.
- [x] Offline T0..T7 rules, native/reception time distinction, raw range/grid association, velocity/XY/gap/timeline plots for D01 all six records (no new formal suite).
- [x] Inspect installed Humble WallRate and upstream matching ControllerServer 1.1.20; preserve official source references.
- [x] If needed measure both plugins using external ELF interposition: unchanged original call forwarded exactly once, monotonic wall and thread CPU, mmap memory records (no per-cycle file I/O), existing MyDWA stage timing captured, calibrate overhead. Separate diagnostic runs and binary hashes.
- [x] Run tests, source/record integrity checks, raw-to-chart cross checks and independent review; create REPORT_V0_2.md and reproducible commands.

Files: run_once.py/run_once.sh, launch_scene.sh, dynamic.rviz (GUI and orchestration); analyze.py (offline events/figures); perf_probe.cpp/build_probe.sh/read_perf.py (external observation only if necessary); cmd_probe.py (native receipt frequencies); tests in test_diagnostics.py; results only under diagnostics/outputs and diagnostics/runs. Existing V0.1 scripts not modified. No commit/push or parameter/scene edits.