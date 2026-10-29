# Dynamic crossing yield integration

Source: `v2-crossing-yield-reserve020`, exact candidate commit
`818150910ac83a7b0dfda91cf80c95e56fce23ed`.
The candidate and old static `main` have unrelated roots. This integration uses
one source snapshot commit on the old `main`, rather than an unrelated-history
merge. Candidate source/configuration, plugin registration, build files and all
maintained tests were copied byte-for-byte at the initial integration. Historical experiment reports,
compressed logs, generated build/install directories and simulation data are
excluded. Four small surface-motion fixtures are required by the unit suite.
The optional `test/migration` replay sources remain; their historical evidence
is available in the original experiment directories, outside this main tree.

## Default Nav2 configuration

The shipped `config/nav2_params.yaml` now enables Scan Tracker, DynamicFilter
and Crossing Yield under `/controller_server`'s `FollowPath` plugin:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "my_dwa_controller::MyDWAController"
      scan_tracker:
        enabled: true
      crossing_yield:
        enabled: true
```

DynamicFilter is constructed with Scan Tracker; it has no separate enable
parameter. Load the complete shipped Nav2 YAML, with the existing `/scan` topic
and scan-time TF to the local costmap frame. Without usable Scan evidence the
enabled filter returns a zero target. Topic, velocity/acceleration limits,
scoring weights, prediction/braking parameters, progress checker and behavior
tree settings are unchanged. Plugin callers omitting these YAML switches retain
the existing false parameter fallbacks; this change is to the shipped Nav2
configuration, not to the Controller implementation.

To opt back into static selection, override both switches in a complete user
configuration or parameter overlay:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      scan_tracker:
        enabled: false
      crossing_yield:
        enabled: false
```

The 2026-10-29 regression used source `f7b6a45` with exactly these two switches
enabled: S01-S06 once each, with S05 repeated twice; D01/D02 three times each.
All 14 dynamic-configuration trials reached the goal without recorded Gazebo
contacts. Four same-source static A/B trials (S04/S05/S06/D02) also reached the
goal without recorded contacts. One static admission attempt failed before
goal dispatch because its check expected inactive Tracker parameters to be
declared; it was retained and retried once after correcting only that check.

D01 used the standard 4-second onset: all three trials triggered and released
Crossing Yield, with about 6.3-6.6 seconds of physical stopping before resuming.
D02 reached its goal in 36.3-39.1 seconds but did not trigger Crossing Yield;
it relied on normal Nav2 recovery, with feedback recovery counts of 10-16 and
2.1-4.0 seconds to resume after the actor physically cleared the corridor.
The static D02 control also used recovery (38.7 seconds, count 15). These
observations do not establish a D02 Yield release or general stability.
S06's last post-success pose sample had about 0.254 rad yaw error versus the
0.25 rad tolerance; arrival status does not establish steady-state tolerance.
The default is enabled by maintainer request with these limitations retained.

Raw logs, bags, exact configurations, plugin mappings/hashes and summary CSV
remain outside Git in
`/home/user/ros2_experiments/dynamic_regression_main_20261029`.
The YAML-loading test covers enabled defaults and zero output without Scan;
the original sampling/acceleration expectation remains covered with explicit
static opt-out. No scenario definitions, thresholds or algorithms are changed.
Rebuild and source a matching installation before running: a source or remote
configuration update alone does not refresh an existing plugin binary.

The strategy maintains crossing risk, permits approach only while a complete
candidate-after-one-cycle brake satisfies the 0.20 m longitudinal reserve plus
the existing sampling allowance, and latches zero target on the first rejection.
Only normal risk release or the existing four-second observed-empty fallback
unlocks that risk stop. It does not implement dynamic detours.

## Verification and known issue

Build and full CTest run in a separate directory and install prefix. Controller
unit tests load the actual registered plugin through pluginlib; scan tests cover
both disabled/default behavior and explicit dynamic behavior. Candidate parity
is verified for `src/`, `include/`, `config/`, `test/`, CMake, package and plugin XML.
The standalone migration replay helper can be compiled against the isolated
installation without including its large recorded inputs in this repository.

The candidate's single D01 and turnback runs completed without contact,
post-latch acceleration or negative pending-risk prediction. S05 navigation and
contact checks also completed, but two isolated Scan returns in one frame gave
a negative raw observed-radius clearance. Their exact origin is unresolved;
they did not match the saved wall geometry. Do not interpret the integration or
these single trials as all-observation-metrics clearance or a general safety
proof. No new Gazebo trials or parameter tuning were part of the initial integration;
the later default-configuration regression is documented above.
Full raw evidence remains at:
`/home/user/ros2_experiments/crossing_yield_reserve020_20261029`.

## Recovery

Tags retained in the formal repository:

- `pre-crossing-yield-main`: exact old main `1bad3b66aa64384d5955f1933dcfb5c1017dcd62`.
- `pre-crossing-yield-formal-checkout-20261029`: previously checked-out formal
  version `c667a957eb15389ff0d0cf2bd91a518337af528c`.
- `v2-crossing-yield-reserve020`: exact source candidate, including provenance history.
- main-crossing-yield-integration-20261029: the source integration commit.
- main-crossing-yield-integration-notes-20261029: the installation/rollback documentation follow-up.

With a clean worktree, inspect either previous source without moving main:

```bash
git switch -c rollback/pre-crossing-yield-main pre-crossing-yield-main
# Alternatively, recover the previously active formal source:
git switch -c rollback/formal-c667a95 pre-crossing-yield-formal-checkout-20261029
```

To undo this integration on clean main while preserving history:

```bash
git switch main
git revert --no-commit main-crossing-yield-integration-notes-20261029 main-crossing-yield-integration-20261029
git commit -m "Revert crossing yield main integration"
```

Resolve any conflicts with subsequent work before continuing a revert. Do not
force-reset or discard local changes. No install command rewrites the formal plugin binary. The existing formal
installation uses source symlinks: three headers and package.xml now expose the
new checkout, while its plugin binary remains the old version. A later explicit
deployment/rebuild must synchronize that installation; it is not done here. Retained Git bundles, build/test logs, hashes and the
exact before/after record are stored outside the repository at
`/home/user/ros2_experiments/main_crossing_yield_integration_20261029`.