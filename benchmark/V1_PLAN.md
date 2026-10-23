# Static benchmark V1 — approved mixed-map design, 2026-10-19

User approved stock S01/S04/S06 plus generated S02/S03/S05 coordinates in scenarios.yaml.
No changes to C++ or controller/GoalChecker/planner/costmap algorithm parameters.

- [x] Inspect stock map, collision geometry and parameter fairness.
- [x] Define metric fixtures: segment distances, irregular-time integration, angle wrapping,
  missing data, direction hysteresis; implement standalone metrics.py.
- [x] Generate corridor SDF and PGM from exactly the same rectangle geometry; verify
  every occupied cell corresponds to a Gazebo box. Preserve stock ODE physics.
- [x] Extend run_trial.py opt-in with raw plans, feedback, BT/action statuses and full
  runtime configuration capture. Preserve default CLI and existing tests.
- [x] Run fresh headless S01/S02/S06 pilots, verify path shapes and terminal rotation
  attribution, then validate remaining scenes. Save failures as well as successes.
- [x] Add a thin restart-per-trial scheduler, controller/scenario/repetition selectors,
  bounded startup/cleanup, fairness audit and aggregation of every scheduled attempt.
- [x] Run three repeats per usable scene and controller; aggregate counts, failure reasons,
  means/sample SD/counts with no statistical-significance claims.
- [x] Review boundary cases, verify original tests/core hashes, document commands,
  definitions, actual results and explicitly unavailable clearance/collision metrics.

Reference for end-to-end lateral metrics is the first published map-frame plan, frozen
for the trial. All subsequent plans and behavior events remain available for attribution.
Time-integrated statistics use actual source timestamps, skip missing/non-increasing or
overlong intervals, and report covered duration. Generated corridor geometry establishes
map/world consistency, not a validated full robot safety footprint. Safety metrics remain
null with an explicit reason. Optional followPath tests can be added only after A works.

Formal run: static_v1_20261019, 35/36 succeeded; MyDWA S04/1 wall timeout,
all attempts retained. Offline source_window_v2 removes pre-planning cached samples
without altering raw records. Final analysis source snapshots and per-stream coverage
are preserved. B/followPath and validated safety geometry remain explicit future work.
