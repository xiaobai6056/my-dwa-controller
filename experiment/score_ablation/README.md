# Reproduce scoring ablations without ROS nodes or Gazebo

Run in WSL Ubuntu-22.04. The preserved prior installation provides the exact
original exported libraries; the experiment builds only the copied offline tool.

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_experiments/simplified_bypass_20261029/baseline_install/setup.bash
cd /home/user/ros2_experiments/score_ablation_20261029
cmake -S source/experiment/rolling -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j3
ctest --test-dir build --output-on-failure
# Create a new directory so archived raw outputs are never overwritten.
mkdir reproduction_B_t4
build/rolling input.json reproduction_B_t4 B 4
```

Use A through G as the fourth process argument, followed by start time. The
runner always observes until navigation time 14 s; it records first passage and
continues the original safety checks. No original gate, initial state, reference
path or scene changes between schemes. `params/*.json` records actual absolute
weights, with no weight-sum normalization.

`commands.jsonl` is the exact seven-command phase-one record. `run_trials.py`
refuses to overwrite existing result directories and checks A's complete
original prefix before executing B through G. No second-stage trials were
selected. `analyze.py` contains output-only classification definitions; they do
not enter the controller or safety logic.
