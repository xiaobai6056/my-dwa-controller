# MyDWA baseline diagnosis (2026-10-21)

Read [REPORT.md](REPORT.md) first. All additions live in this directory. No existing controller, score, policy, parameter, scenario, install library or historical record was changed. No Git commit/reset or navigation-plugin rebuild was performed.

## Inputs and outputs

- `evidence/before.json`: SHA256 of 5252 pre-existing files, original Git HEAD/diff/status. `evidence/final_audit.json`: preservation, six acquisition checks and sqlite integrity.
- `outputs/inventory.json`: historical generations, deterministic representative selection, all repeats/failures and capability gaps. The inventory reflects the pre-acquisition historical collection; new runs are recorded separately.
- `runs/first_S04`, `first_S05`, `first_D01_t3`, `first_D01_t5`, `first_D01_t6`, `first_D02`: fresh acquisitions. Each has exact launch/runner/bag commands in `manifest.json`, resolved launch script, effective scene, runtime configs in `trial/configs`, process mapping, logs, probe JSONL and bag. Collision outcomes are valid data, not infrastructure failures.
- `outputs/all_candidates.csv.gz`: one row per reconstructable evaluated candidate, including rejected candidates. `original_*` and raw/weighted O/S/G/P/H come from the probe; mean/max/Top-K/cell counts are independently reconstructed. All raw probe records remain present, including cycles that cannot be reconstructed.
- `outputs/key_candidates.csv`: selected, fastest-best, slowest-best, least-obstacle and original-policy runner-up. One candidate can have multiple roles. Terminal slowing ordering is v-first; yaw-only terminal cost is not represented as ordinary five-term scoring.
- `outputs/score_cycles.jsonl.gz`, `score_summary.json`, `score_verification.json`: cycles, contributions, attribution, timing and validation. Old no-best cycles lack an exact initial pose: no trajectory values are invented, and timing denominators retain them.
- `outputs/critical_decisions.json`: two representative linked D01 cycles, original contribution differences and fixed-ROI lethal coordinates.
- `outputs/maps_details.json`, `maps_pairs.csv`, `maps_key_availability.csv`: actual stamped-map differences and evidence windows. `maps_findings.md` retains false positives and all repeats.
- `runs/*/bag_*.jsonl.gz`, `bag_analysis.json`, `outputs/bag_summary.json`: read-only extraction, map-stamp TF interpolation, approximate unstamped-GT alignment and publication-content matches.
- `outputs/figures/`: per-condition scoring plots, two-cycle contribution comparison and temporal plots.

## Statistical definitions

K is fixed to 5. `topk_sample_mean` uses the largest min(K,N) pose-sampled costs, including repeated cells. `topk_unique_mean` uses the largest min(K,U) costs of different visited cells. `different_cells` counts U. `threshold_cells` counts distinct cells at raw cost>=253; `threshold_samples` counts pose samples at that threshold. Rejected candidates never enter selection; their original returned mean/max default to zero and must not be read as free-space statistics. Out-of-bounds trajectories have no complete offline mean.

Top-K is descriptive: no Top-K total score or replacement ranking is implemented. The obstacle-term attribution deletes only the already recorded O contribution to identify strict preference changes; this does not establish that the alternate candidate is safer and is never executed online. The original DWA is unchanged.

Temporal extraction uses OccupancyGrid header stamps in odom, registered grid centers, known shared coverage and lethal==100. Inflation/inscribed/unknown/coverage transitions are separate. The exploratory scenario-axis criterion is not a generic motion detector or a preregistered success test. Ground truth enters only post-extraction evaluation. ModelStates is unstamped; receipt-clock quantization remains. An earlier publication with identical raw map content does not reveal the actual age of the controller's map or per-cell observations.

## Offline reproduction (WSL Bash)

```bash
source /opt/ros/humble/setup.bash
source ${MYDWA_WS}/install/setup.bash
cd ${REPO_ROOT}/benchmark/baseline_diagnosis_20261021
export PYTHONDONTWRITEBYTECODE=1

python3 -B -m unittest test_scoring test_bag_export test_temporal -v
python3 -B analyze_scores.py
python3 -B bag_export.py
python3 -B analyze_maps.py \
  --trial "$PWD/runs/first_D01_t3/trial" --label new_D01_t3 \
  --trial "$PWD/runs/first_D01_t5/trial" --label new_D01_t5 \
  --trial "$PWD/runs/first_D01_t6/trial" --label new_D01_t6 \
  --trial "$PWD/runs/first_D02/trial" --label new_D02 \
  --trial "$PWD/runs/first_S04/trial" --label new_S04 \
    --sensor-stream "$PWD/runs/first_S04/bag_maps.jsonl.gz" \
  --trial "$PWD/runs/first_S05/trial" --label new_S05 \
    --sensor-stream "$PWD/runs/first_S05/bag_maps.jsonl.gz"
python3 -B critical_decisions.py
python3 -B audit.py
python3 -B make_report.py
```

These commands regenerate only derived artifacts in this directory. Original inputs remain in place and are hashed in the preservation snapshot. The full temporal command also appears in `evidence/temporal_tests_analysis_run.txt`. To update saved unit-test evidence before regenerating the report, redirect a successful unittest run to `evidence/all_tests.log`; the report checks for `OK`.

`inventory.py` can re-enumerate external inputs. The original one-off probe authoring script was removed from this public archive; `decision_probe.cpp` and the C++ fixture sources remain. The fixture checks returned command/header, exception forwarding and original-call count, not full runtime perturbation.

## Acquisition reproduction (starts Gazebo/Nav2)

On a free ROS domain, with no other test sharing the same domain/Gazebo port:

```bash
source /opt/ros/humble/setup.bash
source ${MYDWA_WS}/install/setup.bash
cd ${REPO_ROOT}/benchmark/baseline_diagnosis_20261021
python3 -B run_missing.py \
  --cases S04 S05 D01_t3 D01_t5 D01_t6 D02 \
  --tag replay_001 --domain 93
```

Choose a fresh tag; existing run directories are never overwritten. The original timing-sweep adapter supplies D01 2/3/4/5/6s keyframes; the launchers/collectors/readiness/cancellation/cleanup are reused. Only the Release overlay line is added to the static launch copy. The original dynamic subscriber-isolation preflight stays intact; recording begins after its dispatch, so initial bag samples are missing. Actual library identity is the process mapping plus manifest hash, not the old runner's hard-coded default-install hash.

For a new set of recordings, `bag_export.py` and `analyze_scores.py` discover completed valid run directories; pass the desired additional sensor streams to `analyze_maps.py`. All attempts are retained; acquisition-invalid runs are excluded from score aggregation with reasons in `outputs/score_excluded_runs.json`. The only legacy admission exception is the two explicitly named old linked runs, which predate bags/parameter-match fields but have verified plugin mappings. No success-only filtering is used.

## Limits

Fresh runs each occurred once; the old repeats are descriptive context, not independent repetitions of the new acquisition. Static S04/S05 have no GT/contact publishers in the original scenes. TF interpolation does not extrapolate; four final dynamic map samples lack a bracket. Raw map metadata.update_time is zero. Published maps (~0.6s) sample more coarsely than configured internal costmap updates. Probe computation and serialization-construction durations omit final file-write/flush time. Unchanged outcomes and absence of loop-overrun warnings do not prove an observer has zero effect.
