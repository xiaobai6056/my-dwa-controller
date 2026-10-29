# Reproduce the polyline experiment (offline only)

Root: `/home/user/ros2_experiments/polyline_bypass_20261029`.
Independent no-hardlink branch `codex/polyline-bypass-20261029`; parent 5857df3.
Authoritative PLAN.md records geometry, fixed matrix and stopping rules.
Inherited experiment plans are historical, not the current experiment's plan.

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_experiments/simplified_bypass_20261029/baseline_install/setup.bash
root=/home/user/ros2_experiments/polyline_bypass_20261029
bash "$root/build.sh"
```

commands.jsonl records each actual rolling invocation, all once. Results exist
for A0/B0/A/B/C/D through16s, E through the first failed fallback at7.15s.
F/G/H, upper multipliers and 6/8s were NOT run. Do not resume or retry the
stopped matrix without new human authorization. No runtime/Gazebo was run.

For independent reproduction of a safe archived case into a new directory:

```bash
mkdir "$root/results/reproduction_A"
"$root/logs/rolling_executed" "$root/results/phase1/A/input.json" \
  "$root/results/reproduction_A" B 4
```

Use mode A for A0 and mode B for the other cases. These example reproduction
commands were not executed again. Per-case input.json preserves exact parameters;
all other scene/input fields equal the original. End time, dynamics and safety
gates are identical. No weight normalization; guided H only. Logs score fields
describe proposed_candidate when command_held=true; selected is the actual output.
The abort row contains a zero output but is not plant-advanced after failure.
Source stamps have the inherited +1s clock offset; results use navigation time.

Read-only analysis:

```bash
python3 "$root/scripts/analyze.py"
"$root/build/failure_check" "$root/input.json" \
  "$root/results/phase1/E/start_4.00.jsonl" "$root/failure_check.json"
python3 "$root/scripts/diagnose.py"
python3 "$root/scripts/report.py"
```

failure_check replays archived commands to reconstruct the actual queue, current
map and fallback at E's failure; pose parity is checked. It does not choose new
actions or simulate beyond the abort. It directly establishes the failed
exclusion subcondition; it does not claim a complete independent dynamic-filter
audit of all fallback subconditions.

Raw telemetry includes all scoring terms, rejection counts, scan source times,
fixed polylines, changing zones, risk/reference states, actual actions/next poses.
analysis.json adds crawl, translation stagnation (<0.5mm/cycle including initial
waiting), reversals, total yaw and reference recovery. Minimum clearance is a
scene-geometry circular-footprint approximation, not a real sensor safety claim.
Ideal geometry/association/rays and future truth checks remain the inherited
conditional model. No history Scan is reused after changing robot pose.

41 tests (9 model +3 weights +14 original reference +5 tuning +10 polyline),
red logs and final XML are retained. version_manifest.json contains actual ELF,
linked immutable .so/.a and source identities; source.bundle recovers committed
code/scripts. verification.json and evidence_manifest.json preserve original
input protection and artifact hashes. Existing formal source/install/scenes and
all prior experiments are unchanged. No merge, push or deployment.
