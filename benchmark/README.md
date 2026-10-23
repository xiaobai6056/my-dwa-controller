# 单次导航 Benchmark（ROS 2 Humble / Gazebo Classic）

静态多场景双 Controller V1 的场景、批量命令和指标定义见 [V1_README.md](V1_README.md)。
下文保留原单次测试说明与第一阶段验收记录。

本目录只执行一次 `NavigateToPose` 并采集数据。默认场景是已安装的
`nav2_bringup/worlds/world_only.model`，机器人为同包的 `waffle.model`。
不修改控制算法，不自动重置，不提供批量执行或性能评分。

## 启动、重置与执行

所有命令在 **WSL Ubuntu-22.04 的 Bash** 中执行。下面的启动和执行命令
均已实际运行。脚本直接从源码运行，无需安装新的 ROS 包或 Python 框架。
首次准备/修改依赖后可运行：

```bash
source /opt/ros/humble/setup.bash
cd /home/user/ros2_ws
colcon build --packages-select my_dwa_controller
source /home/user/ros2_ws/install/setup.bash
```

终端 A：

```bash
bash /home/user/ros2_ws/scripts/start_headless_baseline.sh mydwa
```

这会保留 gzserver、机器人、传感器、TF、AMCL、costmap、Nav2 和 `/clock`，
不运行 gzclient 和 RViz。原有公共启动保护会停止同一用户、同一 ROS domain
内旧的 baseline 仿真；不要在该 domain 同时运行另一项导航实验。

**每次需要相同起点时，先在终端 A 按 Ctrl+C，等清理完成，再执行同一启动命令。**
重新创建 Gazebo 会把真实机器人放回 `(-2.0, -0.5, 0)`，启动脚本等待 AMCL
激活后通过 `/set_initial_pose` 设置相同定位初值。不要仅重设 AMCL 来假装重置
机器人的物理位置，也不要在导航中调用 `/reset_simulation` 使时钟倒退。

终端 B：

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws/src/my_dwa_controller
python3 benchmark/run_trial.py \
  --controller mydwa --start -2 -0.5 0 --goal -1 -0.5 0 --timeout 90
```

默认 trial_id 是 UTC 时间加微秒；也可以传 `--trial-id your_unique_name`。
已有结果目录绝不覆盖。起点/终点均在 `map`，yaw 单位为弧度。
默认起点容差：平移 0.3 m、朝向 0.3 rad。起点不符返回 `invalid_start`，
不发送 Goal。需要其他起点时，先人工完成物理和定位重置，再显式传 `--start`。

使用官方 DWB 时先重启终端 A：

```bash
bash /home/user/ros2_ws/scripts/start_headless_baseline.sh dwb
```

然后终端 B：

```bash
python3 benchmark/run_trial.py --controller dwb --goal -1 -0.5 0 --timeout 90
```

启动脚本选择现有 `config/nav2_params.yaml` 或 `config/nav2_params_dwb.yaml`。
runner 的 `--controller` 仅声明预期插件并核实运行时状态，**不会切换已启动的
Nav2**。必须先使用正确参数启动，再运行测试。两个终端要使用相同的
`ROS_DOMAIN_ID`、RMW 配置和工作区 overlay。

如需保留 UI 流程，仍使用原命令：

```bash
bash /home/user/ros2_ws/scripts/start_ui_baseline.sh
```

UI 脚本未修改，仍使用 MyDWA 参数。交付时本次启动的仿真已停止。

## 预检和 Action 结果

runner 等待 11 个 lifecycle 节点（含两个 costmap）active，检查必要节点
`use_sim_time=true`、持续前进的 `/clock`、新鲜的 `map -> base_link` TF、
`/odom`、传感器话题发布者和 `NavigateToPose` 服务端。只进行读取服务调用；
不会重新发布初始位姿。Humble 的 `waitUntilNav2Active()` 可能等待或发布初始
位姿且无总超时，所以预检使用有总墙钟期限的 lifecycle 查询。

Controller 必须只有 `FollowPath` 一个插件；运行时 `FollowPath.plugin` 必须
匹配预期，并检查实际配置的 BT 文件里 `FollowPath controller_id="FollowPath"`
的绑定。当前只支持这一简单 BT 约定，其他绑定或多 Controller 会明确拒绝。
所有 Controller 参数和 velocity_smoother 参数均从运行中的节点读取并保存。

Goal、完成判断、取消、最终结果使用 Humble 的 `BasicNavigator.goToPose()`、
`isTaskComplete()`、`cancelTask()`、`getResult()`，没有新增 Action Client。
参见 [Humble BasicNavigator API](https://api.nav2.org/nav2-humble/html/classnav2__simple__commander_1_1robot__navigator_1_1BasicNavigator.html)。
成功仅来自 Action `SUCCEEDED`，不从终点距离推断。

Action 调用位于单独线程，采集节点使用独立 executor。主线程按
`time.monotonic()` 监控，`--timeout`、`--ready-timeout`（默认 60 s）、
`--cancel-timeout`（默认 5 s）、`--clock-stall-timeout`（默认 3 s）均为墙钟秒。
即使仿真暂停、Goal 确认或取消调用不返回，采集和有界结果保存仍能进行。
只读预检服务每 2 秒可重试一次，总体仍受 ready-timeout 限制；重试记录到异常。
按 Ctrl+C/SIGTERM 请求取消，等待最终结果或取消宽限期；不要用 SIGKILL 结束测试。

`status` 包括 `succeeded/failed/rejected/canceled/timeout/invalid/invalid_start/
preflight_failed/error`。`action_result` 和原始状态码独立保存：例如超时取消时，
status 仍为 `timeout`，action_result 为 `CANCELED`。
`valid` 表示起点有效、基本数据存在、有最终位姿、时钟正常、Action worker
结束，不表示导航成功，也不表示每行数据齐全。存在时钟异常或无法确认取消
时为 false；取消未确认会记录 `cancellation_unconfirmed`，下一次测试前需要
停止并重启 baseline。程序仅在 `succeeded` 且 `valid=true` 时返回 0，其余
已保存的试验返回 2；非法参数或文件系统错误可能直接报错。

## 数据含义

每次生成 `benchmark/results/<trial_id>/trajectory.csv`、`summary.json` 和 `configs/`。
JSON 保存目标、预设/实测起点、容差、Controller 运行参数、BT 绑定、话题端点
及 QoS、文件路径与 SHA256、Action 结果、仿真/墙钟耗时、终点误差、异常计数。
configs 包含所声明 YAML、实际 BT、实际 map YAML、安装的 launch 文件、启动脚本
及 runner 源码快照。运行参数是生效配置的证据，传入 `--params-file` 的文件是
配置来源记录；runner 不会把该参数文件重新加载到 Nav2。

轨迹默认按墙钟约 10 Hz 采样最新数据，所有原始来源时间戳单独保留：

| CSV 字段 | 来源与语义 |
| --- | --- |
| `sim_time_s` | 最近收到的 `/clock` 仿真时间；暂停时允许重复，倒退使试验无效 |
| `wall_elapsed_s` | 发送阶段起始以来的 monotonic 墙钟时间 |
| `tf_stamp_s`, `x,y,yaw` | TF `map -> base_link`，用于和 map 目标/未来全局路径统一比较 |
| `odom_stamp_s`, `odom_frame`, `odom_child_frame` | `/odom` 消息头和 child frame；不使用其 odom-frame pose 评估终点 |
| `actual_vx,actual_vy,actual_wz` | `/odom.twist.twist` 的实测运动速度，当前 child=`base_footprint`，单位 m/s、rad/s；不是 map 分量 |
| `controller_vx,controller_vy,controller_wz` | `/cmd_vel_nav`，Controller 输出、速度平滑之前 |
| `robot_cmd_vx,robot_cmd_vy,robot_cmd_wz` | `/cmd_vel`，Gazebo 差速插件实际订阅的指令通道；包含平滑输出，也可能包含 recovery 指令 |
| `*_cmd_receipt_sim_s`, `*_age_s` | Twist 没有 header：只记录接收时仿真时间和墙钟接收年龄，不伪造源时间戳 |
| `flags` | 缺失、过期、TF 获取失败标记；相应数值留空，JSON 对应值为 null |

实时端点已确认 `/cmd_vel_nav` 发布者为 controller_server，订阅者为
velocity_smoother；`/cmd_vel` 发布者含 velocity_smoother 和 behavior_server，
订阅者为 turtlebot3_diff_drive。实测速度来自该 Gazebo 插件发布的 `/odom`。
TF 链为 `map -> odom -> base_footprint -> base_link`。

订阅使用 BEST_EFFORT / VOLATILE 的 sensor-data QoS，兼容此环境中
BEST_EFFORT 的 `/clock` 以及 RELIABLE 的 odom/指令发布者。
默认新鲜度上限 0.5 秒；TF 同时检查时间戳是否继续在墙钟时间内更新，
odom/指令检查接收年龄及仿真时间年龄。因 `/clock` 约 10 Hz、odom/TF 约 30 Hz，
允许来源时间戳领先最近收到的 clock 最多 0.15 秒。CSV 是异步最新值采样，
不是硬件同步或插值重采样；不得把所有列当成严格同时的测量。

终点位置误差为 map 平面欧氏距离，朝向误差为 wrap 到 [-pi,pi] 后的绝对值。
终点取 Action 结束/取消宽限期结束时的有效 TF；不是机器人完全停稳后的误差。
Nav2 当前 XY 成功容差为 0.25 m，因此约 0.24 m 的误差仍可得到真实 Action 成功。
导航耗时包含等待 Goal 确认和取消收尾，预检时间不计入；仿真时长用两端 clock
读数相减。时钟停止/倒退时必须结合 status、valid 和墙钟耗时判断。

## 查看结果

```bash
cd /home/user/ros2_ws/src/my_dwa_controller
python3 -m json.tool benchmark/results/mydwa_006/summary.json
head -n 6 benchmark/results/mydwa_006/trajectory.csv
tail -n 3 benchmark/results/mydwa_006/trajectory.csv
python3 benchmark/evidence/inspect_results.py \
  benchmark/results/mydwa_006 benchmark/results/dwb_001
```

最后一条验证 CSV 时间顺序、实际非零速度、有限数值、真实成功状态和配置文件
哈希，仅用于验收，不计算评分。CSV 可直接由 pandas.read_csv 读取，空值成为 NaN。

## 2026-10-19 实测验收

| 试验 | 结果 | 墙钟/仿真秒 | 行数 | 终点位置/朝向误差 |
| --- | --- | --- | --- | --- |
| `mydwa_006` | SUCCEEDED，MyDWA | 3.299 / 3.300 | 35 | 0.2419 m / 0.0205 rad |
| `dwb_001` | SUCCEEDED，DWB | 3.490 / 3.500 | 36 | 0.2331 m / 0.0521 rad |
| `timeout_002` | wall_timeout，Action CANCELED | 见 JSON | 非空 | 不用距离改写结果 |
| `clock_stall_002` | clock_stalled，CANCELED，valid=false | 见 JSON | 25 | 过期 TF 留空，终点误差 null |
| `reject_wrong_start` | invalid_start，无 Goal | 不适用 | 0 | 起点偏差约 0.77 m |
| `reject_wrong_controller` | 预期 DWB、实际 MyDWA，拒绝执行 | 不适用 | 0 | 不适用 |

MyDWA 验收中 35 行位姿/实测速度全部有效，33 行 Controller 指令、32 行机器人
指令有效。导航最初几帧指令尚未到达，明确留空并记录异常；一次 AMCL 只读服务
响应超时后重试成功。DWB 只是兼容性验证，不构成正式性能对照。
`mydwa_001..005`、`clock_stall_001` 等是保留的调试/旧版本记录，不用于最终验收；
尤其 `mydwa_003` 的旧版本新鲜度判断使终点数据缺失。

本轮实际执行的验证命令：

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws
colcon build --packages-select my_dwa_controller
# 避免测试用 controller_server 名称与正在运行的仿真冲突：
ROS_DOMAIN_ID=97 colcon test --packages-select my_dwa_controller
colcon test-result --test-result-base build/my_dwa_controller --verbose
python3 -m unittest discover -s scripts/tests -v
cd /home/user/ros2_ws/src/my_dwa_controller/benchmark
python3 -m unittest -v test_run_trial
# 在全新 MyDWA baseline 上运行；002 已存在，再执行应换唯一后缀：
bash evidence/fault_checks.sh 002
sha256sum -c evidence/unchanged.sha256
bash -n /home/user/ros2_ws/scripts/start_headless_baseline.sh
bash -n /home/user/ros2_ws/scripts/start_ui_baseline.sh
```

构建通过；8 组 C++ 功能测试、12 项 benchmark Python 测试、5 项原有启动清理
测试通过，Python flake8/pep257 通过。全包测试仍有一项已存在的 uncrustify
格式问题：`src/obstacle_evaluator.cpp` 第 43 行；聚合报告会同时计入 CTest
容器和其 xunit 失败，不能把聚合计数当作两个独立故障。该 C++ 文件、全部核心
源码、两份 Nav2 YAML 和 UI 脚本校验和均未变，未为本任务改动格式。
本轮未重新手动操作 RViz/UI 导航；其脚本语法和原内容校验通过。

原始输出位于 evidence/，包括 headless 日志、Action 输出、构建/测试日志、
端点列表、独立结果检查和启动前哈希。生成的上游源码快照不参加 Python lint：
flake8 使用 AMENT_IGNORE，pep257 在 CMake 中显式排除 results/evidence。

## 最小后续扩展

保持 runner 一次只执行一个 trial。以后在它之前加外部 `reset -> 等待 TF/clock
恢复 -> 起点验证` 步骤，再用简单场景表循环调用 CLI，每次使用唯一 trial_id。
指标计算独立读取已有 CSV/JSON，先增加路径长度和成功率；若需要偏离全局路径，
再额外保存 map-frame `/plan`。自动重置、动态障碍、评分和统计比较均留待后续。
