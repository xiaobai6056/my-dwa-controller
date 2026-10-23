# D01 avoidability and decision diagnosis — execution record

- [x] Freeze5007 existing files/Git diff/install/plugin/model fingerprints; all match on final audit.
- [x] Derive full conservative planar collision enclosure from actual Nav2-default waffle.model, including shape/link transforms.
- [x] Tested offline hold-v/w and brake-wait across all24 old2–5s records at0.1s decision spacing. Ideal2.5 vs coordinated wheels1.0, latency.15, margins.03/.12; numericaldt.01. Windows explicitly conditional.
- [x] External original-function forwarding fixture calibrated (normal return, exception, count, scores, map, chosen trajectory). Copied actual costmap inside existing lock, full candidate feasibility/score records, complete chosen trajectory. No Controller rebuild.
- [x] Two valid actual Nav2 Release diagnostic runs(start2/4), 115+141cycles; runtime plugin/probe maps and SHA verified; all cycle consistency checks pass.
- [x] Three Gazebo-only real-command validation runs at two representative timings: early2s brake4.0 and early4s brake4.6 safe stop-wait-continue; late4s brake6.0 stops then is hit6.81. Not Nav2 success-rate data.
- [x] Retain two infrastructure failures separately: clockQoS readiness failure and probe linker failure. Corrected tooling only; no selective controller-result retries.
- [x] Figures, per-run/decision CSV+JSON, raw logs, controlled stop comparison, bounded interpretations and one next research direction.
- [x] 63Python tests,9Release CTests,compileall,shell syntax,diff check,5007-file audit, read-only scientific/code review.
- [x] Report/README/reproduction scripts/new local export. No commit/push.

Unresolved and explicitly disclosed: full synchronous observer write/flush overhead not measured; GT lacks source stamps; .12m is sensitivity under assumed timing bounds, not calibrated certificate; exact latest physical braking deadline and global unavoidability unknown; all candidate map costs were captured but independent grid recalculation covers selected trajectories only.