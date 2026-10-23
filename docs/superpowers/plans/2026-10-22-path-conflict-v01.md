# Path Conflict V0.1 implementation plan

Goal: opt-in Costmap path conflict detection and conservative speed governance,
without changing the existing DWA score or persistent Nav2 speed limit.

## Checked interfaces

Humble; PathHandler already returns the current local path in the Costmap frame.
ObstacleEvaluator uses Costmap2D::worldToMap/getCost and has no snapshot utility.
Reuse Costmap2D's copy constructor and path_geometry's projection/extraction.
Copy once under the source mutex; monitor and candidate evaluation share that
immutable snapshot outside the mutex. The disabled path does no monitoring.

## Decisions

- Monitor: lethal=254 only; Inner uses the configured padded circumscribed radius,
  Outer adds a margin. Reproject both historical maps onto the CURRENT path.
  Match cells in world coordinates, require aligned resolution/origin lattice,
  compare common known coverage; newly visible/unknown cells cannot imply motion.
- A different observed map state requires a changed cost in overlapping cells;
  rolling coverage alone and identical control-loop snapshots do not count.
  This is content change, NOT a map update timestamp or a sensor age estimate.
  Keep the adjacent pair plus an optional positive reference, expire the comparison after 1 second.
- Outer expansion: at least two new lethal cells on a side, one-cell inward
  centroid shift dominating longitudinal shift. Mere static presence is not risk.
- Governor: Normal/Limited/Braking/Wait/Recovering/DwaFallback. Stop only if
  v*reaction_time + v*v/(2*brake_acceleration) fits before core minus robot radius
  and margin. Otherwise retain DWA. WAIT requires actual stopped velocity and
  reachable zero. Three distinct, fully observed clear states release the latch;
  duplicates or unknown coverage never advance recovery.
- Merge transient caps after terminal policy, clamp to the reachable lower bound;
  preserve setSpeedLimit and existing scorer normalization. Terminal yaw control
  takes priority once its original terminal phase is active.
- Enable at configure (default false). Do not reset risk on same-goal replans; reset for a different goal.
  Reset on deactivate/cleanup. No dynamic parameter callback or extra publisher.

## Work and checks

- [x] Write gtests for static walls, inward expansion, rolling/unknown/duplicate
  maps, path changes, stopping distance, unsafe fallback and clear hysteresis.
  Compile public API with inert stubs, observe expected assertion failures.
- [x] Implement PathConflictMonitor and RiskGovernor independently. Run their
  gtests; extend edge cases before fixing failures.
- [x] Integrate Controller snapshot, per-cycle cap, diagnostics and disabled
  characterization oracle; test external speed limits and terminal behavior.
- [x] Build isolated Release prefix and run existing C++/Python regression suite.
- [x] One closed-loop run per D01 2/4/6 and S04/S05, original scenes and parameters
  except feature enable. Save new manifests, exact commands, params, plugin hash,
  and all outcomes, including failures. No tuning against these outcomes.
- [x] Review safety/state transitions, document results/limits and switch-off
  rollback. Verify prior benchmark/config files are unchanged.

Initial feature parameters are centralized in PathConflictConfig/RiskGovernorConfig:
lookahead 1.5m; Outer extra width 0.33m; warning speed 0.12m/s; braking 0.3m/s²
(bounded by acc_lim_x); reaction delay 0.3s plus controller period; stop margin
0.1m; clear states 3. Existing stopped threshold is reused. Evidence thresholds
are fixed V0.1 constants, not a tuning surface.

Completion notes: first five smokes preserved, prefix geometry correction followed by two targeted D01 smokes; final folded-path safety fix has functional/unit validation only. D01 still collides, no parameter search. Complete CTest has three style-target failures; see benchmark/path_conflict_v01_20261022/README.md.
