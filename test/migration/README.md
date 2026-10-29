# Frozen-input parity probe

This optional standalone CMake tool is not a runtime dependency or a CTest unit
test. Install ROS2 Humble rosbag2_cpp, tf2_msgs and nlohmann_json; source an isolated
my_dwa_controller installation, then run:

```bash
cmake -S test/migration -B /tmp/mydwa-migration-replay -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/mydwa-migration-replay -j3
/tmp/mydwa-migration-replay/replay_history FROZEN_FRAMES_JSON RUN_DIRECTORY OUTPUT_JSON
```

The run directory must contain a ROS2 bag/ with source Scans and /tf,/tf_static.
The frozen input schema is the archived conflict_region_feasibility_20261027
inputs: id and frames containing stamp_ns, path, robot, velocity, observations,
and optional decision (now_ns, initial, measured, accepted). Use archived files,
not reconstructed ground truth. The source timestamp and exact TF are retained.

Replay all bag Scans through the real Tracker, then query CrossingYield with each
frozen consumed-frame path and robot state, actual MotionPredictor braking, and
two duplicate reads. Compare every observation and YieldDecision value; normalize
only the old previous_index field spelling to previous_cluster_index. Output has
no numerical tolerance. A null JSON margin can represent the original infinite
empty-brake diagnostic; these histories use nonempty brake trajectories.

This proves structural parity for the supplied sequence, not the original ROS
executor ordering, all control cycles, live Costmap admission, whole-controller
scoring, or general safety. Original migration recordings and comparison reports
are maintained separately; they are not included in this repository. Supply
your own frozen inputs and ROS bag using the schema above.
