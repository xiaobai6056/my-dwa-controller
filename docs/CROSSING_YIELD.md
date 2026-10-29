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

## Regression evidence and limits

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
