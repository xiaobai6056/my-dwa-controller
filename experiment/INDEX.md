# Offline experiment index

The self-contained entrypoint for this branch is
[polyline_bypass/reproduce.sh](polyline_bypass/reproduce.sh). It builds the current
[rolling sources and tests](rolling/CMakeLists.txt), runs the tests, then replays
the three bundled [inputs](polyline_bypass/inputs) as A0/A1/A2 into a new external
output directory. See the [reproduction instructions](polyline_bypass/README.md).

Prerequisites: Linux/Bash, CMake, C++17, ROS2 Humble/ament, a compatible installed
`my_dwa_controller` exporting its libraries, Nav2 costmap, nlohmann_json and GTest.
Source ROS2 and the controller installation before building. Python analysis also
requires numpy and matplotlib. No build/install products are bundled.

| Stage | Retained record | Scope |
|---|---|---|
| Frozen input probe | [report](../docs/experiments/simplified_bypass_20261029.md), [CMake](CMakeLists.txt) | Candidate feasibility; original bags required by export.py |
| Rolling model | [instructions](rolling/README.md), [report](../docs/experiments/rolling_bypass_20261029.md) | Conditional geometry and persistent delayed execution |
| Scoring ablation | [instructions](score_ablation/README.md), [report](score_ablation/REPORT.md) | No qualifying scheme |
| Locked point | [instructions](locked_reference/README.md), [report](locked_reference/REPORT.md), [design](locked_reference/design.json) | No confirmed early active bypass |
| Heading/margin sweep | [instructions](reference_sweep/README.md), [report](reference_sweep/REPORT.md), [matrix](reference_sweep/matrix.json) | No qualifying improvement |
| Polyline | [instructions](polyline_bypass/README.md), [report](polyline_bypass/REPORT.md) | Rearward motion without earlier completion; E stopped on braking fallback failure |

Historical reports describe the original experiments and evidence inventories.
Their raw recordings, full matrices, binaries, hashes and logs live in external
evidence archives; they are not files bundled here. Historical runners require
that evidence plus the matching source/binary version; current rolling sources
implement the final polyline experiment and cannot stand in for every predecessor.
Do not resume stopped matrices or infer successful unrun variants.

`build.sh EXTERNAL_BUILD_DIRECTORY [rolling|probe]` builds without installing.
`build_probe.sh EXTERNAL_BUILD_DIRECTORY` selects the frozen-input probe.
`export.py ORIGINAL_RECORDING_DIRECTORY NEW_OUTPUT_JSON` needs the original
`bag/` and `paired_decisions.json`; it never substitutes ideal scene data.

Historical Python tools use `EXPERIMENT_DATA_ROOT`, an explicit external directory
containing original `input.json`, results and required metadata. Trial runners
that compare a predecessor also require `EXPERIMENT_PREVIOUS_ROOT`. Analysis writes
derived JSON/plots in that selected evidence directory; use a copied archive for
review. Scripts fail if required evidence is absent. Their numerical definitions
and safety/stopping gates are retained.

All stages use ideal scene geometry/association, current-pose synthetic rays and
future-truth checks, rather than a real ROS/Gazebo perception/execution loop.
Circular-footprint clearances and single runs do not establish real-system safety.
