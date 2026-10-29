# Scan Emergency V2 archive

This archive preserves the Scan Emergency V2 proof of concept from 2026-10-23,
based on static V1.1 commit `1bad3b66aa64384d5955f1933dcfb5c1017dcd62`
(`v1.1-static`). Controller source, configuration, scenario definitions, assets,
C++ tests/fixtures, numerical results, plots and failure records are preserved.
Historical reported runs and test counts are not new validation runs.

## Start here

| Purpose | Canonical entry |
| --- | --- |
| Static scenarios and assets | [scenarios.yaml](../benchmark/scenarios.yaml), [scenes.py](../benchmark/scenes.py), [assets](../benchmark/assets/) |
| Static runners | [run_trial.py](../benchmark/run_trial.py), [run_suite.py](../benchmark/run_suite.py), [V1 usage](../benchmark/V1_README.md) |
| Dynamic scenarios and assets | [scenarios.json](../benchmark/dynamic/scenarios.json), [scenario.py](../benchmark/dynamic/scenario.py), [assets](../benchmark/dynamic/assets/) |
| Dynamic runners | [run_trial.py](../benchmark/dynamic/run_trial.py), [run_suite.py](../benchmark/dynamic/run_suite.py), [usage](../benchmark/dynamic/README.md) |
| Scan Emergency V2 behavior | [design and limitations](scan_emergency_v02.md), [initial report](../benchmark/scan_emergency_20261022/REPORT.md) |
| Final diagnostic matrix | [report](../benchmark/scan_v2_matrix_20261022/REPORT.md), [summary.csv](../benchmark/scan_v2_matrix_20261022/summary.csv), [timeline.csv](../benchmark/scan_v2_matrix_20261022/timeline.csv), [figure](../benchmark/scan_v2_matrix_20261022/matrix_interaction.png) |
| Earlier static results | [V1](../benchmark/V1_RESULTS.md), [V1.1](../benchmark/v1_1/RESULTS.md) |
| Earlier dynamic investigations | [dynamic results](../benchmark/dynamic/RESULTS.md), [timing sweep](../benchmark/dynamic/timing_sweep/REPORT_TIMING_SWEEP.md), [avoidability](../benchmark/dynamic/avoidability/REPORT_AVOIDABILITY.md) |

The final matrix contains four historical valid Gazebo trials:

| Condition | Recorded outcome | Navigation time (s) | Approximate minimum gap (m) |
| --- | --- | ---: | ---: |
| D01 t2 A | Succeeded, no recorded contact | 53.7 | 0.094 |
| D01 t2 B | Succeeded, no recorded contact | 26.0 | 0.302 |
| D01 t4 | Canceled after right-wheel/pillar contact | 7.0 | 0.000 |
| D01 t6 | Succeeded, no dynamic risk trigger | 15.1 | 0.166 |

This is not a general dynamic obstacle avoidance release. In t4 the first
Fallback state did not impose effective deceleration before contact. Stopping
certification and angular motion were inconsistent across t2 trials; a recorded
Scan visibility gap produced an isolated clear-area pulse. Terminal rotation
also inflated t2 A's completion time. Approximate geometry and receive-clock
alignment limit clearance/timing interpretation. The report preserves the
observations and unresolved causes; these four trials do not establish a
statistical success rate or a safety guarantee.

## Reproduction boundary

The public tree includes compact JSON/CSV results, figures, source and tests.
Raw bags/JSONL, full trial directories, build/runtime logs, installed plugins,
source ZIP/TAR snapshots and complete external evidence are excluded. Historical
reports describe the original collection; references to `runs/`, `outputs/`,
`evidence/` or frozen artifacts may require that separately retained data. Do
not infer that a referenced external file is present in this checkout.

For commands in historical documents, set paths for your own environment:

```bash
export REPO_ROOT="$(git rev-parse --show-toplevel)"  # run inside this clone
export MYDWA_WS="/path/to/external/ros2_ws"
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
# Source the appropriate external install only after checking its version.
```

Executable helpers derive repository paths from their location and read
`MYDWA_WS` for external install/build prefixes and baseline scripts (default:
`$HOME/ros2_ws`). `${REPO_ROOT}` and `${MYDWA_WS}` strings in archived JSON and
historical prose are redacted provenance placeholders, not new measurements.
The baseline workspace's `scripts/_baseline_common.sh`,
`scripts/_baseline_processes.py`, and baseline launch scripts are external
prerequisites, not supplied by this archive. Offline regeneration also needs
its original recordings at the documented relative locations. Frozen historical
runner hash checks still require their original external manifests; a new
experiment needs a new provenance snapshot after relocating scripts. Performance
probes accept `NAVPERF_MYDWA_LIB`; otherwise the dynamic loader resolves
`libmy_dwa_controller.so` from the configured library search path.

One-off process plans, script-authoring mutations, source/result packagers and
initial workspace snapshot utilities were removed after checking that retained
executables do not reference them. Existing launchers, analysis/report tools,
probe/fixture sources and benchmark evaluation thresholds remain. No Gazebo
trial was run for this public presentation cleanup.
