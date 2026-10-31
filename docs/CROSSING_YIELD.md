# Dynamic control

## Default configuration

The shipped `config/nav2_params.yaml` enables Scan Tracker and Crossing Yield
under `/controller_server`'s `FollowPath` plugin. DynamicFilter is constructed
with Scan Tracker and has no separate enable parameter:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      scan_tracker:
        enabled: true
      crossing_yield:
        enabled: true
```

Use the complete Nav2 parameter file, the configured `/scan` topic, simulation
time when applicable, and scan-time TF to the local costmap frame. Without usable
Scan evidence the enabled filter returns a zero target. Rebuild and source an
installation matching the source before running; changing source configuration
does not refresh an existing plugin binary. Callers that omit the YAML switches
retain the Controller's existing false parameter fallbacks.

To restore static selection, explicitly override both switches:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      scan_tracker:
        enabled: false
      crossing_yield:
        enabled: false
```

The policy maintains crossing risk, permits approach only with the existing
braking admission and 0.20 m longitudinal reserve plus sampling allowance, and
latches a zero target on rejection. Normal risk release or the existing
four-second observed-empty fallback unlocks the risk stop. It does not implement
dynamic detours. Velocity, acceleration, scoring, prediction/braking, progress
checker and behavior tree parameters are unchanged by enabling the switches.

## Quasi-static handover

When both existing dynamic switches are enabled, a latched crossing stop also
observes its continuously associated scan points with a C++ median window.
Original scan-order fragments at the full-scan seam are joined for this view
using the Tracker's configured adjacency distance; clustering and motion
estimation remain unchanged. No object search or rebinding is performed after
association loss.

The window requires at least eight distinct source scans spanning two seconds,
with a maximum pairwise distance of 0.05 m between their coordinate-wise medians.
Invalid points, a broken association, source gaps or age above 0.35 s, robot
motion, and trusted object speed at or above 0.05 m/s prevent permission.
Range excess slides out of the window rather than resetting its history.

The Controller uses raw stamped odometry from the existing `odom_topic` in the
local costmap frame, with the configured base or `base_footprint` child frame.
It waits for odometry to cover each Scan, then checks only causal samples:
linear speed magnitude below 0.01 m/s and angular speed below 0.05 rad/s for
0.5 s, with scan-relative and current age no greater than 0.35 s.
The bounded one-second/256-record stop-state buffer refuses a query if its
causal sample was evicted; a later usable Scan can resume observation.
Planning velocity and zero commands are not used as stop evidence.

Permission clears CrossingYield and bypasses DynamicFilter for all candidate,
observed-geometry and braking-fallback checks in the same control cycle.
Ordinary Costmap collision checks, prediction, braking limits and scoring remain
active. Tracker continues to observe. A newer, associated full velocity of at
least 0.05 m/s with a current forward-path conflict restores dynamic handling;
the existing stateless path-conflict query supplies this positive risk test.
Other detected full-motion or existing partial-crossing risks veto handover.
Unknown velocity is never filled with zero to establish a stop.

**Lifecycle limitation:** the mode belongs to the plugin activation, not a
NavigateToPose session. `setPlan()` does not reset it, because replanning may
call that method repeatedly. A new navigation request, including the same goal
after cancellation, can inherit ordinary mode until new trusted conflict motion
is observed or the plugin is deactivated/reactivated. This integration does not
identify top-level sessions or alter Nav2 lifecycle management. Lifecycle and
ROS-clock regression reset the mode, window and stop evidence to dynamic.

Median stability is quasi-static position evidence, not proof of physical
stillness or safety. Slow/tangential unobservable motion, changing visible
subsets, association loss and multiple unknown obstacles remain limitations.
There is no three-second timeout degradation, extra recovery algorithm or
dynamic detour planner. The original 0.20 m approach reserve is unchanged.

## Historical regression evidence and limits

The 2026-10-29 regression ran S01-S06 once, then repeated S05 twice, and ran
D01/D02 three times each. All 14 dynamic-configuration samples reached the goal
without recorded Gazebo contacts. Four same-source static A/B samples
(S04/S05/S06/D02) also reached the goal without recorded contacts. One static
admission attempt failed before goal dispatch because the check expected
inactive Tracker parameters to exist; it was retained and retried once after
correcting that check.

- D01 used the standard four-second onset. Each run triggered and released
  Crossing Yield, stopping physically for about 6.3-6.6 seconds before resuming.
- D02 took 36.3-39.1 seconds, did not trigger Crossing Yield and relied on normal
  Nav2 recovery. Feedback recovery counts were 10-16; forward motion resumed
  2.1-4.0 seconds after the actor physically cleared the corridor. The static
  control also used recovery (38.7 seconds, count 15).
- S06's last post-success pose sample had about 0.254 rad yaw error versus the
  0.25 rad tolerance. Arrival does not establish steady-state tolerance.
- In an earlier S05 trial, two isolated Scan returns gave negative raw
  observed-radius clearance without matching the saved wall geometry. Their
  origin remains unresolved.

The default is enabled at the maintainer's request with these limits retained.
These few successful samples do not prove general stability, all-observation
clearance or a D02 Yield release. Raw bags, logs, exact configurations and hashes
are maintained separately under experiment IDs
`dynamic_regression_main_20261029` and `crossing_yield_reserve020_20261029`;
they are not distributed in this repository. Maintained unit tests exercise
enabled defaults without Scan and explicit static opt-out. Optional frozen-input
migration checks are documented in [test/migration/README.md](../test/migration/README.md).
