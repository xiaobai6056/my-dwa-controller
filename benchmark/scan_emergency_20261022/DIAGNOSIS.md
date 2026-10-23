# Offline root causes and minimal repair plan

Source hashes match the prior final_manifest.json; existing dirty work preserved.
Raw input: prior round3_D01_t2 bag, original tracks/risk/actor CSV, launch log and plots.

1. Visible support changes: elapsed5.12 xmax=1.706, elapsed5.72 xmax=1.370,
   while xmin stays1.334..1.336; centroid vx~-0.30 with confidence~0.71.
   Y bounds both progress in +Y, supporting real transverse motion. A small
   centroid regression residual is not a physical motion validity test.
2. Protection starts elapsed2.6, Braking cap0.245, available0.446 > stop0.204.
   At3.1 a changing predicted boundary invalidates stopping and preserves0.185.
3. Full old swept approach+crossing patch (sampled528 cells, finite rays only)
   first becomes clear after protection at10.12. All393 post12 scans pass this
   geometric audit. The prior path patch alone is already empty at2.32 BEFORE
   crossing; therefore path emptiness alone cannot release an approaching hazard.
4. After12s all679 logged risk records have scan_unresolved=1 and
   corridor_known=0. The global moving_seen expiry flag has no clearing path.
   Full forward-corridor coverage also depends on the moving map window endpoint,
   although the original crossing region is observed. These are independent gates.

Minimal design: keep existing three modules and DWA. Tracker adds explicit motion
validity from paired visible support, and retains rays for finite free-space
proof. Ordinary cluster expiry has no permanent risk ownership. Monitor filters
for nearby transverse motion and owns ONE fixed crossing/approach rectangle with
no identity requirement; preserve it through unknown/missing observations. Clear
only when ALL sampled swept-region cells have fresh finite-ray visibility and
crossing-corridor Costmap cells are covered and non-colliding. Governor alone
counts distinct clear scans and controls braking/wait/recovery. Explicit release
on confirmed Normal removes the region; no timer clears a hazard. Freeze the
initial conflict location so visibility changes cannot drag the parking target.

- [ ] Write failing support-switch, longitudinal/static exclusion, lost-ID and
  ray occlusion/NaN/stale/unknown-map/region-clear tests; governor distinct scan
  confirmation, stale recovery and plugin orchestration regressions.
- [ ] Implement smallest Tracker/Monitor/Governor changes, Controller only wires
  release feedback/diagnostics. Preserve scan-disabled behavior and DWA scoring.
- [ ] Reuse old bag alignment to create read-only replay inputs and compare actual
  tracker/monitor/governor outputs before/after; no live publishers or Gazebo.
- [ ] Separate build_scan_emergency/install_scan_emergency/log_scan_emergency.
- [ ] All functional tests before S05 then D01_t2 (one each); D01_t4 only if needed
  to distinguish an unresolved cause. Preserve each version, params and outcome.
- [ ] Complete CTest and scoped format results separately, source/plugin hash
  correspondence, protected file hash audit, final report and known limitations.

No threshold sweep, Benchmark changes, startup-time logic or timeout clearance.
The user explicitly requested this analysis first and then autonomous execution.
