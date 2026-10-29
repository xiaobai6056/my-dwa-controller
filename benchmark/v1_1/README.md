# MyDWA V1.1：新路径与终点锁存同步

本轮只修复 Controller 终点生命周期，不调参。结果及冻结结论见 [RESULTS.md](RESULTS.md)，机器可读证据在 `evidence/analysis.json`、`evidence/comparison.csv` 和 `freeze_manifest.json`。

## 修改边界

`src/controller.cpp::setPlan()` 在原 `state_mutex_` 内执行：

1. 拒绝空路径、非法全局 frame、冲突的逐点 frame 和非法位姿（沿用 `toPose2D()` 的有限值及四元数校验）。
2. 将路径复制到局部变量；校验或分配抛异常时保留旧路径、旧终点状态。
3. 通过编译期确认不抛异常的 move assignment 提交路径，再将 `goal_phase_` 设为 `Normal`。

同终点、路径时间戳变化、四元数 q/-q 均按新路径处理。下一控制周期根据当前位置和 GoalChecker 提供的容差重新判断；没有 `setPlan()` 时仍保留原锁存。不改 `prepareGoalControl()`、评分、采样、运动约束、GoalChecker、BT、地图和仿真启动入口。无 TF 的 syntactic 校验不等于提前验证可变的 TF 可用性；TF 仍由原控制流程处理。

这与 Humble 1.1.20 的生命周期一致：[DWBLocalPlanner::setPlan](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)、[RotateToGoalCritic::reset](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp)。Nav2 的 `setPlannerPath()` 在调用插件 `setPlan()` 后重置 GoalChecker。

## 确定性测试

`test/test_controller.cpp` 新增 6 个测试，并修正 3 个历史预期。覆盖：同终点且 XY 漂出后恢复平移；XY 内继续旋转；新目标不继承状态；不重规划保持锁存；无效路径保持旧路径和状态；初始化异常不损坏状态；与真实 `SimpleGoalChecker::reset()` 配合；每控制周期和每秒重规划仍能在闭环中完成旋转。保留原正常控制指令特征签名测试。

旧实现红灯结果在 `evidence/red_tests.xml`：28 项相关测试中 5 项失败。修复后的完整输出、JUnit 和构建输出保存在 `evidence/`。没有注入内存分配失败或进行并发压力测试；分配异常安全由先复制和 noexcept 提交保证，状态一致性由统一互斥锁保证。

在 WSL Ubuntu 22.04 中运行：

```bash
source /opt/ros/humble/setup.bash
source ${MYDWA_WS}/install/setup.bash
cd ${MYDWA_WS}
colcon build --packages-select my_dwa_controller --symlink-install
ROS_DOMAIN_ID=97 colcon test --packages-select my_dwa_controller --event-handlers console_direct+
colcon test-result --test-result-base build/my_dwa_controller/test_results --verbose
```

已知 `src/obstacle_evaluator.cpp` 的历史 uncrustify 问题仍存在，不能声称全部 lint 通过。本轮修改文件的格式检查通过。

## 真实回归与复现

完全复用 `benchmark/run_suite.py`、`run_trial.py`、`start_scene.sh`。每次重启 Gazebo + Nav2，检查模型物理起点、AMCL/TF 起点、实际 Controller 插件和参数；通过 NavigateToPose 获取真实结果。原始数据位于 `benchmark/results/v1/v1_1_*`，未覆盖 V1 或 R1。

本轮实际 S04 命令保存在 `evidence/run_s04.sh`，以 `v1_1_S04_001` 至 `005` 各运行一次；每次执行后使用 `evidence/check_s04.py` 检查 valid、成功、零恢复和无进度失败，失败即停。其余命令保存在 `evidence/run_remaining.sh`。

重跑时必须换一个不存在的 run-id（原 runner 会拒绝覆盖），例如：

```bash
cd ${REPO_ROOT}/benchmark
python3 run_suite.py --controllers mydwa --scenes S04 --repeats 1 --run-id v1_1_replay_S04_001
python3 v1_1/evidence/check_s04.py results/v1/v1_1_replay_S04_001
# 逐次重复 S04，检查 5 次均没有停滞/恢复，再继续：
python3 run_suite.py --controllers mydwa --scenes S06 --repeats 3 --run-id v1_1_replay_S06
python3 run_suite.py --controllers mydwa --scenes S01 S03 S05 --repeats 2 --run-id v1_1_replay_tracking
```

离线核验本轮数据与配置快照、生成对照 CSV（不启动仿真、不改原始试验数据）：

```bash
python3 v1_1/evidence/analyze.py > v1_1/evidence/analysis.log
less v1_1/RESULTS.md
python3 -m json.tool v1_1/freeze_manifest.json
python3 v1_1/evidence/verify.py
head v1_1/evidence/comparison.csv
```

`evidence/analyze.py` 按固定的 `v1_1_*` 命名前缀发现结果；若另外添加了同前缀重跑结果，离线表会相应增加试验，原冻结清单保持首次验收范围。它直接复用 R1 的只读 debug 解析函数，原 R1 文件及历史探针不变。R1 探针断言的是旧缺陷，不应用于验证新插件。

## 指标与限制

- 完成时间统一采用 `source_window_v2` 的首个 ComputePathToPose 原生时间戳至观察到终止的仿真时间跨度，约 0.1 s 接收量化，不是精确的 NavigateToPose Action 执行时长。失败时间独立保留，不混入成功均值。
- 地图坐标轨迹、里程计实际速度、`/cmd_vel_nav` 控制指令、`/cmd_vel` 机器人指令分开记录；缺采与异常保留在 CSV flags/summary。长度及相对首条参考路径折线段的时间加权 RMSE 沿用 V1，速度统计使用原生采样间隔。
- 恢复次数以 NavigateToPose feedback 为主，结合 BT、行为 Action、Controller 进度失败日志。正常 FollowPath 因重规划被替换的状态变化不能直接当作恢复次数。
- 控制阶段日志约 1 Hz 节流，只能给出观察到的状态顺序，不能当作完整控制周期跟踪。瞬间 `Rotating` 超出 XY 并不单独证明死锁，需结合重规划后是否恢复平移、零输出持续性及进度检查。
- 仍无可靠碰撞/contact 与 footprint 障碍净间距证据，不声称更安全。Gazebo/AMCL 有随机性，少量重复只能支持本轮回归结论。S02 沿用 V1 历史结果，本轮未补跑。
