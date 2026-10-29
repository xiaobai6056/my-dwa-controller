# Dynamic proposal, four-frame admission, locked reference

Scope: isolated offline prototype only, formal f7b6a45 untouched. Parent a072e77.
No guide weights changed and no rewards added. The inherited hard gates stay byte-identical.

- [x] Test reference geometry with .10m extra clearance and .10m beyond exit, too-narrow/walled routes, fresh source de-duplication, four-frame stopped admission, lock persistence, reversal/loss stop, rear passage and safe original-path recovery.
- [x] Use one small reference lifecycle module (waiting/locked/done), not another risk tracker. Existing Exclusion owns the motion direction/association and updates geometry; existing CrossingYield owns its event, occupancy/clear and latch.
- [x] Generate current scene rays for original CrossingYield; full scene envelope/known identity remain a declared oracle observation. Never replay historical Scan at a changed pose.
- [x] A holds the original stop latch until original risk release. B may override that output hold only with explicit four-frame permission; do not mutate CrossingYield private latch/release. Reference release is independent from risk release.
- [x] While waiting compute points only on fresh observations. On lock use the latest point; reevaluate atan2 from current robot every cycle. Abort on invalid evidence, unsafe fallback or no admissible action. Keep the original hard gate on nominal, backup and queued backup.
- [x] After actual rear passage, drop the point only if an ordinary-score action is safe and does not increase original-path distance (1e-9 tolerance; actual release strictly improves it). No forced event release and no automatic re-lock in the same event.
- [x] Common fixed end time 16s for both trials, enough to observe a parking-release baseline passing. Record first milestones while continuing safety monitoring.
- [x] Run 4s A/B once; only if B shows meaningful pre-8.8 rear displacement and earlier full passage, recheck 6/8 with matching A. Otherwise preserve a negative result without tuning.
- [x] Independent review, all raw telemetry and parameter/archive preservation; do not enter Gazebo or runtime development automatically.

Result: first-stage screen negative. No 6/8 s run, tuning, runtime integration or Gazebo.

Result: first-stage screen negative. No 6/8 s run, tuning, runtime integration or Gazebo.
