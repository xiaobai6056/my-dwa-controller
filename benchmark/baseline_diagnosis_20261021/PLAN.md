# MyDWA baseline diagnosis implementation plan

> For agentic workers: use subagent-driven-development for independent offline analysis and verification. The user approved this scope on 2026-10-21. Add files only inside this directory; do not commit, reset, rebuild the controller, change navigation parameters, or overwrite historical records.

**Goal:** Diagnose obstacle-score decisions and historical Costmap information with auditable measurements, without changing the algorithm.

**Architecture:** Reuse ControllerDiagnostics, the existing ABI observation probe, static/dynamic runners and all valid historical trials. Reconstruct trajectories offline, validate their endpoints and map statistics, and compute descriptive Top-K and unique-cell statistics. Compare stamped maps in odom with rolling-window registration; keep inflation and unknown/coverage changes separate. Record raw topics only when missing evidence requires a new trial.

**Tech stack:** ROS2 Humble/Nav2 1.1.20, existing Gazebo Classic scenes, Python/numpy/matplotlib, rosbag2 sqlite3, existing libdecision.so.

## 1. Preserve and inventory
- [x] Freeze all pre-existing project/script/install files and the original Git diff/status in evidence/before.json.
- [x] Inventory scenario coverage, historical raw streams, library/source hashes, timestamp semantics and data gaps; save outputs/inventory.json.
- [x] Select all first repeats by an explicit rule, retaining other repeats and failures for context.

## 2. Score reconstruction
- [x] Add tests for sampled versus unique-cell Top-K, map-boundary checks, exact Euler reconstruction and weighted terms before implementation.
- [x] Run tests and record expected missing-function failures.
- [x] Implement scoring.py and analyze_scores.py. Reconstruct every scored candidate, compare endpoint/mean/max with original observations, and export candidate/cycle tables.
- [x] Export chosen, fastest, slowest-best, least-obstacle and runner-up candidates with weighted O/S/G/P/H contributions and selection provenance. Top-K is descriptive only: no new score or policy is executed.

## 3. Map history
- [x] Add and first fail tests for rolling-window translation, duplicates, unknown cells, non-integral grid shifts, dilation-only changes and frame/time discontinuities.
- [x] Implement temporal.py and analyze_maps.py, using actual message stamps, common odom coordinates, overlap masks and lethal-only occupancy changes.
- [x] Align maps with odometry, GT receipt-clock uncertainty, motion start, conditional brake windows and physical contact stamps; reject unsupported alignments rather than inventing transforms.
- [x] Export stamp/receipt gaps, duplicate statistics, change coordinates, motion evidence and timing figures.

## 4. Targeted acquisition
- [x] Reuse complete historical D01 t2/t4 candidate observations and first repeats at t2/3/4/5/6 plus D02 maps.
- [x] For missing candidate/raw-topic evidence, use original S04/S05/D01/D02 scenes, the same selected Release plugin and unchanged YAML, with the existing observation probe. Do not change scene geometry or physics.
- [x] Reuse launch/collector/cleanup routines through a thin adapter. Save exact commands, effective scene, runtime plugin maps/hash, configs, raw rosbag and logs in fresh run directories. Record only on a free isolated domain.
- [x] Check observer overhead, map timestamps/TF coverage and navigation validity; preserve failed attempts and repeat only a diagnosed infrastructure failure.

## 5. Delivery and audit
- [x] Run unit tests, probe pass-through calibration, reconstruction checks and append-only integrity audit.
- [x] Write outputs/*.json, CSV(.gz), figures and REPORT.md answering the five requested questions, with facts/measurements/hypotheses separated.
- [x] Provide README commands and record unresolved limitations. No success claim based only on an old report.

## Completion evidence

- Six fresh valid acquisitions; no infrastructure failure or outcome-selective retry.
- Eight score conditions, 888517 scored and21045 rejected reconstructable candidates, zero mismatches. Old exact-pose omissions remain explicit.
- 26 stamped-map streams /611 maps; 40 offline unit tests green.
- Independent numerical/spec review corrected terminal runner-up ordering, fail-closed admission and one causal overstatement.
- Audit checks5252 old files and the original Git HEAD/diff/status unchanged; six bag sqlite quick checks pass.
