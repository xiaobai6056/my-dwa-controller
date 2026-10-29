# Dynamic crossing yield integration

Source: `v2-crossing-yield-reserve020`, exact candidate commit
`818150910ac83a7b0dfda91cf80c95e56fce23ed`.
The candidate and old static `main` have unrelated roots. This integration uses
one source snapshot commit on the old `main`, rather than an unrelated-history
merge. Candidate source/configuration, plugin registration, build files and all
maintained tests are copied byte-for-byte. Historical experiment reports,
compressed logs, generated build/install directories and simulation data are
excluded. Four small surface-motion fixtures are required by the unit suite.
The optional `test/migration` replay sources remain; their historical evidence
is available in the original experiment directories, outside this main tree.

## Defaults and enabling

The checked-in YAML files are unchanged from the pre-integration formal checkout
`c667a957eb15389ff0d0cf2bd91a518337af528c` and the candidate. `scan_tracker.enabled`
is false, and `crossing_yield.enabled` defaults to false in the Controller.
No launch scripts, scoring weights, motion parameters or BT files are changed.
The earlier static main lacks the optional scan/motion YAML entries; they are
added here exactly as already present in the formal checkout, with scanning off.

To enable crossing yield explicitly in a separate user-selected ROS parameter
file/overlay under the existing controller plugin namespace:

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

This is an enabling overlay, not a replacement for the complete Nav2 config.
Use the existing `/scan` topic and TF setup, or the deployment's explicit topic.
Turning the switches on does not reproduce the full benchmark configuration;
its fixed motion settings and other inputs remain in the experiment snapshots.
No switches were enabled automatically during integration.

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
proof. No new Gazebo trials or parameter tuning are part of this integration.
Full raw evidence remains at:
`/home/user/ros2_experiments/crossing_yield_reserve020_20261029`.

## Recovery

Tags retained in the formal repository:

- `pre-crossing-yield-main`: exact old main `1bad3b66aa64384d5955f1933dcfb5c1017dcd62`.
- `pre-crossing-yield-formal-checkout-20261029`: previously checked-out formal
  version `c667a957eb15389ff0d0cf2bd91a518337af528c`.
- `v2-crossing-yield-reserve020`: exact source candidate, including provenance history.
- `main-crossing-yield-integration-20261029`: the new integration commit.

With a clean worktree, inspect either previous source without moving main:

```bash
git switch -c rollback/pre-crossing-yield-main pre-crossing-yield-main
# Alternatively, recover the previously active formal source:
git switch -c rollback/formal-c667a95 pre-crossing-yield-formal-checkout-20261029
```

To undo this integration on clean main while preserving history:

```bash
git switch main
git revert main-crossing-yield-integration-20261029
```

Resolve any conflicts with subsequent work before continuing a revert. Do not
force-reset or discard local changes. The formal installation is unchanged by
this source integration. Retained Git bundles, build/test logs, hashes and the
exact before/after record are stored outside the repository at
`/home/user/ros2_experiments/main_crossing_yield_integration_20261029`.