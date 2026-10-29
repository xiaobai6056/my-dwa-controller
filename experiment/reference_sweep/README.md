# Reproduce the strong Heading / reference-margin sweep

WSL Ubuntu-22.04 root:
`/home/user/ros2_experiments/reference_sweep_20261029`.
Parent locked-reference commit b43493e11e977e0705c03b909a52dbcdd38e3a70;
formal main f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446.
Independent no-hardlink clone, branch `codex/reference-sweep-20261029`.

The authoritative plan is this experiment's PLAN.md and params/matrix.json.
Inherited rolling/locked_reference docs describe previous experiments only.

## Build and verify (Bash)

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_experiments/simplified_bypass_20261029/baseline_install/setup.bash
root=/home/user/ros2_experiments/reference_sweep_20261029
bash "$root/build.sh"
```

This links the same immutable shared/static baseline libraries; no production
install is built. Final tests: model 9, weights 3, reference 14, tuning 5.
The four initially failing tuning tests are preserved in logs/red_tuning.*.

## Reproduce one archived variant into a new directory

```bash
case_name=H10_M10_t4
mkdir "$root/results/reproduction_$case_name"
"$root/logs/rolling_executed" \
  "$root/results/phase1/$case_name/input.json" \
  "$root/results/reproduction_$case_name" B 4
```

Use A instead of B for A_t4. `build/rolling` has the same SHA as the archived
actual executable. These commands were not run again after the fixed sweep.
All exact original invocations appear in commands.jsonl; every original output
directory was created exclusively and each of the 13 trials ran once.
The input JSON's `reference_parameters` contains only the three tunable fields:
`heading_multiplier`, `extra_clearance`, `forward_offset`. Other input fields
are identical to the original input.json. Ordinary scoring uses original H;
the multiplier applies only when the existing reference direction is in use.
`effective_heading_weight` and `guided_scoring` make that distinction explicit.

When command_held is true, score/weighted_score describe proposed_candidate,
not the actual zero command; score_is_held_proposal records this explicitly.
Only selected/next_pose count as executed movement. Source stamps retain the
inherited +1 second offset; all reported times are navigation-relative.

## Derived analysis / read-only scoring replay

```bash
python3 "$root/scripts/analyze.py"
python3 "$root/scripts/diagnose.py"
python3 "$root/scripts/summarize_audit.py"
python3 "$root/scripts/make_report.py"
```

These do not launch rolling trials. The scorer replay reconstructs the exact
archived action queue and current-pose map, reapplies candidate gates, and checks
selected proposal score parity within 1e-12. It records fastest admitted and
ordinary-score alternatives at the same state. Replay uses perfect scenario
geometry/association, like the original D01 model; it is not a sensor replay.

The local Windows mirror is used only for convenient review. Authoritative raw
evidence stays under the WSL root: results/phase1, params, logs, commands.jsonl,
baseline_reproduction.json, analysis.json, reference_events.json, key_scores.json,
diagnosis.json, score_evidence.json and per-variant score_audit.json.

Code and experiment scripts/docs are committed in the isolated clone; source.bundle
contains its recoverable history. version_manifest.json records commit, actual
ELF, dynamic plugin and all linked baseline static-library SHA256 values.
verification.json records tests, all 3120 cycles and unchanged protected inputs;
evidence_manifest.json hashes the archived evidence (excluding itself/build cache).

No eligible candidate, so no 6/8 second confirmation, further search, Controller
logic changes, formal repo/install edits, merge, push, deployment or Gazebo.
0.2/0.3 metre values change reference construction, not the hard exclusion zone
or a universal executed-clearance guarantee. Real perception/closed loop remains
unverified. Report.md findings must not be presented as actual robot safety.
