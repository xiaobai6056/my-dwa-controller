# Reproduce the locked-reference offline experiment

Authoritative location: `/home/user/ros2_experiments/locked_reference_20261029`.
Formal baseline: `f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446`.
Independent clone parent: `a072e771125a6482fc8801cc75b460fa4ceffbe5`.
Branch: `codex/locked-reference-20261029`. No formal install/build/deploy.

The root PLAN.md/design.json/REPORT.md govern this experiment. The inherited
`experiment/rolling/PLAN.md` is historical documentation of the original runner,
not instructions for the reference-point experiment.

## Commands (WSL Ubuntu-22.04, Bash)

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_experiments/simplified_bypass_20261029/baseline_install/setup.bash
root=/home/user/ros2_experiments/locked_reference_20261029
bash "$root/build.sh"
```

Each archived phase1 A/B invocation is recorded verbatim in commands.jsonl.
For an exact rerun of the archived producer, choose **new, empty** directories:

```bash
mkdir "$root/results/reproduction_A" "$root/results/reproduction_B"
"$root/logs/phase1_executable" "$root/input.json" "$root/results/reproduction_A" A 4
"$root/logs/phase1_executable" "$root/input.json" "$root/results/reproduction_B" B 4
```

These reproduction commands are documentation only; they were not run again.
`build/rolling` is the final version including the post-review selected-zero
fail-safe. It can be substituted above to reproduce behavior on D01 under that
version. `zero_guard_trace_proof.json` proves the added predicate was true for
all 240 B proposals; no new rolling run was used to validate this boundary fix.

Read-only scoring replay and analysis (overwrite only their derived outputs):

```bash
"$root/build/audit_reference" > "$root/logs/score_audit_reproduction.log" 2>&1
python3 "$root/analyze.py"
```

All source, scripts and dependencies' SHA256 values are archived. The original
phase1 runner patch is `runner_phase1.patch`; `runner.patch` contains final
changes. The producer-source snapshot reconstructs the phase1 selected-rank
predicate and helper prior to the zero-command guard; its other source inputs
are unchanged. The original immutable shared libraries remain the same for
both versions. There was no algorithm weight/geometry tuning after phase1.

## Evidence

- `results/phase1/{A_t4,B_t4}`: actual commands, proposed candidates, score terms,
  reference and risk states, rejection counts, nominal/backup checks, executed
  next poses, raw stdout, parameters and summaries; 240 cycles each, t=4..16 s.
- `params/design.json`: geometry/admission/screen assumptions; per-trial files
  preserve the exact pre-run parameters. The final design adds an explicit
  statement of the conservative selected-zero fail-safe only.
- `score_audit.json`: detailed original sampler/scorer replay of key cycles;
  selected total matches raw telemetry within 1e-12, all candidate hard gates
  retained. It does not generate another rolling trial.
- `analysis.json`, `comparison.png`, `REPORT.md`: derived metrics and findings.
- `logs/red_*.{log,xml}`: initial failing tests; `logs/final_*.xml` and
  `logs/final_ctest.log`: passing verification. Tooling build/audit logs retained.
- `protected_before.json`, `verification.json`, `version_manifest.json`,
  `evidence_manifest.json`, `source.bundle`: isolation, immutable-input checks,
  source/binary identities and recoverable independent commit.

Fresh observation time is navigation-relative; inherited source stamps are
offset by +1 second. A locked target is not an obstacle prediction or frozen
exclusion zone. The connectivity certificate is not supplied as a DWA path.
Selected-zero permission loss deliberately re-arms even if another nonzero
candidate exists; it is a conservative boundary, unexercised in this D01 run.

No phase2 (6/8 s), runtime Controller edits or Gazebo run were performed because
the declared pre-8.8 active-displacement screen failed. The conditions are ideal
scene observations and future truth safety gates, not real perception.
