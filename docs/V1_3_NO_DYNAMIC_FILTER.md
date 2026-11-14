# v1.3 — no dynamic candidate filter

This is an engineering simplification based on the 2026-11-03 ablation, not a
collision-prediction correctness fix. The six pairs / twelve valid A/B runs did
not establish a clear navigation benefit from filtering. D02 changed many
candidates without proving a benefit; disabling filtering introduced no new
observed collisions or clear navigation-performance regression in that sample.
D01-t5 collided in both groups. Some minimum-clearance metrics traded off.
Removal does not solve dynamic avoidance or establish equal safety.

## Control flow

1. Sample the original reachable DWA velocities and apply terminal-goal limits.
2. Use MotionPredictor to form nominal, one-control-period-then-stop backup, and
   immediate-brake trajectories. Keep the stopping horizon and Costmap checks.
3. Score the feasible nominal trajectories with the existing weights and tie order.
4. Continue ScanTracker, full/partial surface motion and source-time observations.
5. Let CrossingYield admit bounded approach or latch zero output and release it
   from its existing observation conditions. Median quasi-static handoff and
   trusted path-motion re-entry remain in place.
6. With no feasible candidate, output a zero target. Keep the immediate-brake
   completeness/Costmap emergency diagnostic; a zero target is not instant rest.

DynamicFilter, DynamicTrajectoryEvaluator and DynamicRiskEvaluator and their
exclusive tests/exports are removed. There is no replacement runtime switch.
Eigen and sensor_msgs remain required by retained partial motion and ScanTracker.
PathConflictEvaluator remains used by trusted-motion re-entry and is independent
of the deleted candidate filter.

## Diagnostics

`Stoppable decision` has schema_version=2. It retains sampled/static-feasible/
accepted counts, static and horizon rejection counts, navigation/brake/emergency
mode, pre-Yield candidate, final output, actual velocity/pose, stop time and static
fallback status. Cluster records are now `[velocity_valid,vx,vy,point_count]`.
Prediction availability/state/reason, known/unknown point counters, dynamic and
observed-geometry rejection counts, selected dynamic clearance and dynamic
fallback fields are removed. `global_dynamic_active` still describes the
CrossingYield/median mode; it does not enable a trajectory filter.

## Limits and rollback

Unknown or stale Scan data no longer independently rejects otherwise admissible
candidates; existing pending Yield risk still holds. Costmap is the retained
candidate collision gate. Missing motion evidence is never filled with zero to
claim a stationary obstacle. Unobservable motion, stale geometry, late crossing
intent, and limited sampled/point coverage remain unproven safety boundaries.
No velocities, margins, scenes, scoring weights or new avoidance policy changed.

Keep `v1.2-m-handover` as the recoverable previous implementation. The new tag is
`v1.3-no-dynamic-filter`; checkout and rebuild/source the selected version before
running it. Neither tag switching nor source edits change an already loaded plugin.
