# Dynamic crossing yield

This branch includes optional Scan tracking and crossing-yield control. Both
`scan_tracker.enabled` in the checked-in YAML and `crossing_yield.enabled` in the
Controller default to **false**. Enable them explicitly in a complete Nav2
configuration using the existing controller plugin namespace:

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

Use the deployment's Scan topic and TF setup. This overlay does not replace the
complete configuration or reproduce all benchmark settings. Leave both switches
false for static operation. Optional tracking depends on valid source timestamps,
Scan/TF alignment and the existing motion predictor and trajectory safety checks.

The strategy maintains crossing risk and allows approach only when a complete
candidate-after-one-cycle brake meets the 0.20 m longitudinal reserve plus the
sampling allowance. The first rejection latches a zero target. Normal risk
release or the existing four-second observed-empty fallback unlocks the risk stop.
The production strategy does not implement the offline dynamic detours.

Maintained unit tests cover default-disabled and explicit dynamic operation,
including the registered plugin. Four small surface-motion fixtures are required.
The optional [migration probe](../test/migration/README.md) requires external
frozen inputs and original bags. Offline bypass experiments have their own
[index](../experiment/INDEX.md); their idealized geometry is not real perception.

Historical single D01 and turnback runs completed without contact,
post-latch acceleration or negative pending-risk prediction. S05 navigation and
contact checks also completed, but two isolated Scan returns in one frame gave
a negative raw observed-radius clearance. Their origin remains unresolved and
they did not match saved wall geometry. These results are not an all-observation
clearance claim or a general safety proof. Raw recordings and deployment records
are external archives; no Gazebo rerun or parameter tuning is included here.
