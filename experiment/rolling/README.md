# Offline rolling bypass reproduction

All commands run in WSL Ubuntu-22.04. No ROS node or Gazebo is launched.
The prior experiment's immutable standalone installation supplies the original
exported libraries; current Controller sources are unchanged.

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_experiments/simplified_bypass_20261029/baseline_install/setup.bash
cd /home/user/ros2_experiments/rolling_bypass_20261029
cmake -S source/experiment/rolling -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j3
ctest --test-dir build --output-on-failure
# Use a new output directory to preserve the archived results.
mkdir -p reproduction_results
build/rolling input.json reproduction_results
build/audit_selected .
```

`audit_selected` reads archived `results/` and replays only the selected nominal
predictions; it does not change the simulation or invent actions. `analyze.py`
generates connectivity certificates and plots. Those certificates never enter
the original reference path, velocity sampler or scorer.

The fixed model validates that input size/timing matches D01 t2. See REPORT.md
for all idealizations, particularly complete geometry, future-truth collision
admission, map/world alignment, and missing real sensor/executor validation.
