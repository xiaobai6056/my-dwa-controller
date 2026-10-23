# Minimal single-trial benchmark — 2026-10-19

Scope: existing stock TurtleBot3 world, one NavigateToPose action, manual reset.
No controller C++ or scoring changes. The workspace has no Git repository.

Design: keep UI startup intact; headless sources the overlay and selects an
existing MyDWA/DWB YAML. A collector node owns TF, sensor subscriptions and
bounded preflight service calls. BasicNavigator owns the action in one worker;
the main thread uses monotonic wall time for sampling, timeout and cancellation
grace, including when simulation time stops. It never substitutes endpoint
distance for an action result. One configured FollowPath plugin plus the runtime
BT binding establishes which controller is used.

- [x] Add failing stdlib tests for start checks, angle wrapping, stale/missing
  data, clock reset/stall and bounded action supervision.
- [x] Implement benchmark/run_trial.py and pass those tests.
- [x] Fix only the headless entry point; add Python runtime dependencies.
- [x] Launch MyDWA headless, inspect live endpoints/QoS/lifecycle/TF, run one
  navigation, inspect saved CSV/JSON and action result.
- [x] Exercise refusal, timeout/cancellation and paused clock paths; run existing
  C++ and startup-cleanup tests; optionally run DWB after a clean restart.
- [x] Write exact reset, launch, execution and inspection commands in README;
  record evidence and remaining limits. Leave the simulation stopped.

Results: trajectory.csv contains map/base_link pose, separate source/receipt
timestamps, odom child-frame measured twist, raw controller and smoothed commands,
freshness fields and explicit missing-data flags. summary.json contains action
status, monotonic and simulated durations, start/goal errors, data-quality counts,
runtime parameters, topic endpoints and copied config hashes. Failed preflight
also produces a summary and header-only CSV. Existing result directories are
never overwritten. Reset remains an external step before preflight, providing
the insertion point for future batch trials.

Acceptance: mydwa_006, dwb_001, timeout_002, clock_stall_002 and the two refusal
trials. Independent review corrections cover collector-exception cancellation
and TF wall freshness. All 12 benchmark tests pass. Existing C++/startup tests
pass; full lint retains the pre-existing obstacle_evaluator.cpp formatting issue.
Python lints exclude generated results/evidence to avoid checking archived
upstream copies; only this lint registration is added to CMake.
