# Nav2 动态障碍 Benchmark V0.1

目的：在现有冻结 MyDWA / 官方 DWB 上测量端到端 NavigateToPose 原始性能，为后续研究提供可复现基线。仅新增 benchmark 工具，没有修改 Controller、Nav2 YAML、机器人模型、雷达或 Costmap 配置，没有提交或推送 Git。

## 从干净 WSL 终端复现

```bash
cd ${MYDWA_WS}
source /opt/ros/humble/setup.bash
colcon build --packages-select my_dwa_controller --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
cd src/my_dwa_controller
# 完整 2 场景 × 2 Controller × 3 次，必须使用新的 run-id
bash benchmark/dynamic/run.sh --repeats 3 --run-id dynamic_reproduce_001
# 单独选择：这些是独立的测试批次，不替代完整正式批次
bash benchmark/dynamic/run.sh --scenes D01 --controllers mydwa --repeats 1 --run-id smoke_new_d01
bash benchmark/dynamic/run.sh --scenes D02 --controllers dwb --repeats 1 --run-id smoke_new_d02
# 无仿真重算已保存数据
python3 benchmark/dynamic/run_suite.py --aggregate benchmark/results/dynamic/formal_v0_1_20261020
# Python/启动器回归、语法检查
bash benchmark/dynamic/evidence/checks.sh
# 完整包检查（现存 C++ uncrustify 失败单独见 RESULTS.md）
cd ${MYDWA_WS}
colcon test --packages-select my_dwa_controller --event-handlers console_direct+
colcon test-result --test-result-base build/my_dwa_controller --verbose
```

无 UI：launch 参数固定 `headless:=True use_rviz:=False use_sim_time:=True`。依赖现有 ROS2 Humble / Gazebo Classic 11、gazebo_ros 插件、Nav2 Simple Commander、Python 标准库与已有 PyYAML；未引入 RL 或大型测试框架。`run.sh` 自动 source overlay、生成 world/map、设置 TurtleBot3 model。现有路径按本机工作区固定；迁移到另一工作区时需统一替换路径。

默认 ROS_DOMAIN_ID=86，Gazebo master 端口 11431；可用 `--domain` 选择独立域。不要在同一域启动另一个 baseline，现有 baseline_preflight 会清理同域旧 baseline。批次串行执行，重复 2 交换 Controller 顺序；每次均创建新的 Gazebo / Nav2 / AMCL / Controller 进程树，以避免上轮 Costmap、TF、插件状态和运动脚本污染。wall watchdog、取消 grace、SIGINT/TERM/KILL 和现有 domain-aware cleanup 负责退出；清理失败时拒绝启动后续污染运行。结果目录禁止覆盖，失败继续保留。计划表、实际命令、返回码、清理结果写入 manifest.json。

## 场景

|项目|D01 横向穿越|D02 临时封堵后释放|
|---|---|---|
|起点 → 终点|[0,0,0] → [4,0,0]|相同|
|主通道宽度|3.0 m|0.9 m，x=1.7..2.3 有封闭侧袋|
|实体盒尺寸 x/y/z|0.5/0.5/0.7 m|0.4/0.8/0.7 m|
|路径|x=1.6，y=-1.1 → +1.1|x=2.0，y=1.1 → 0 → 1.1|
|相对任务时间|t=4 开始，t=12.8 结束|t=2 进入，t=6.4 完全封堵，停至 t=20，t=24.4 返回侧袋|
|移动速度|0.25 m/s|0.25 m/s|
|仿真导航超时|90 s|120 s|

以 `scenarios.json` 为唯一运动/尺寸/起终点定义，`scenario.py` 复用静态 `scenes.generate()`，生成一致的 Gazebo 物理墙和 occupancy map，增加少量非对称凹槽供现有 AMCL 定位。与静态测试保持相同机器人、Nav2 参数和生成方法；本次为独立直通道地图，不声称和 S01–S06 完全同图。

D02 封堵时两侧各约 0.05 m，不能容纳机器人，侧袋为死路，不能提供绕过 blocker 的完整路径。退出后盒底边 y>=0.45 才标记通道恢复；理论 t=23.4，实际时刻从真值观测记录。实体退出至 y=1.1 后，主通道完全恢复原始 0.9 m 宽度。D01 没有要求绕行或等待必须采取哪种策略。

障碍是带 collision、visual、contact sensor 的 Gazebo static model，由 `motion.py` 调用 `/benchmark/set_entity_state` 按仿真时间关键帧移位。它是规定运动的不可推动物体，物理接触与激光射线都有效；不是 RViz marker，也不是有动力学响应的人。运动请求受 /clock 的约 10 Hz 更新限制（0.25 m/s 对应约 0.025 m 台阶），不是连续轨迹或可被机器人推开的刚体。每次服务响应被核对，超时归测试系统异常。

运动 epoch = 导航 dispatch 前采集到的 `/clock`，不依赖机器人位置、Controller 类型或真值反馈。两种控制器使用相同关键帧；不同前进速度导致到达相交位置的时间不同，故遭遇强度不完全相同，这是固定时间方案的公平性限制。运动本身确定性；Gazebo 传感器噪声、AMCL 粒子和进程调度保留原设置，未强行固定系统所有随机源，不能保证位级相同。

## 控制与评价数据流

Controller 仍通过现有 `/scan` → Nav2 obstacle/voxel layer → Costmap 以及里程计、TF、规划路径获取输入。没有真值速度/位置信息注入 Controller 或成本评分。

仅新世界加载 `libgazebo_ros_state.so`，在 `/benchmark/model_states` 输出机器人/障碍位置；仅读取评价节点 `/benchmark_collector` 订阅。真值订阅者列表会保存并检查。同节点中的独立 Motion 辅助类只允许写 `dynamic_obstacle`，不写机器人位姿、速度指令或 Costmap。Controller 源码、配置、插件二进制哈希保留。全套原有 Git 管理文件在批次前后比较 SHA256。

`Observer` 保存原始真值、实际 `/cmd_vel`、碰撞 pairs/depths、scan 和 Local Costmap。scan 在 TF + Gazebo 机器人世界位姿下投影，端点落在动态盒边界 0.08 m 容差区域计为观测证据；这是关联证据，不是逐射线物体 ID。Costmap 使用同一时刻世界真值相对机器人坐标，再通过 TF 转到 grid frame，统计盒投影内 >=99 的 occupancy cells。原始雷达 ranges、相关坐标和完整 Costmap grid 压缩保留，便于离线抽查。可能有同步误差和边界静态墙误关联；命中计数不能被当作感知精度评测。

## 指标定义

- **成功率**：真实 `NavigateToPose` 返回 `SUCCEEDED`，数据有效且无机器人物理 contact。`action_result` 与 benchmark `termination` 分列；取消碰撞任务不能当作 Nav2 自主导航失败。预检失败属于系统异常，不算一次已接收导航。
- **时间**：从任务 dispatch 观测 `/clock` 到终端 action 结果观测 `/clock`；包括取消确认等待，与静态 V1.1 的首 planner 时间窗定义不同，不直接横向混比。约 0.1 s 量化；不是精确 action server 内部时间。成功均值仅包含成功项。失败/碰撞/超时运行时间逐项保留；wall 时间只作为 watchdog 和诊断。
- **真实碰撞**：动态盒和全部生成墙体安装 100 Hz contact sensor，使用 Gazebo `ContactsState` 中包含 `turtlebot3_waffle::` 的碰撞对；地面轮胎接触不计。首次机器人接触触发取消；保存后续取消期间的所有接触消息。消息数不是独立碰撞次数。没有用 `/scan` 阈值判碰撞。contact 实际采样/通信有限，不能证明绝无漏检；新世界障碍运动为离散台阶，不能外推到连续碰撞检测。
- **最小净间距（近似）**：障碍的精确二维盒与机器人保守外包矩形之间的欧氏最短距离；重叠记 0。机器人矩形相对 base_footprint 为 x=[-0.1965,0.077]、y=[-0.153,0.153] m，来自原 Waffle 底盘盒、轮胎、雷达、脚轮、相机碰撞投影的包络；含空隙，可能低估真实间距，不能当作精确碰撞几何并集或穿透深度。Gazebo model_states 名义 50 Hz，但消息无源时间戳，记录接收时最新 `/clock`，同步精度约一个 clock tick。只统计动态盒净间距，不是对全部墙面的全局最小间距。
- **终止原因**：success / timeout（sim）/ collision / navigation_failed / system_error；原始 action status、取消、clock/运动/真值异常和错误文本另存。仿真时间回退/停滞、真值停滞、运动服务异常和 wall watchdog 不冒充控制器失败。
- **角速度变化**：实际底盘输入 `/cmd_vel`（经 velocity_smoother），原生消息序列 `sum(abs(w[i]-w[i-1]))`，单位 rad/s；总变差与任务时长、消息频率有关，不是角加速度。方向切换要求超过 ±0.05 rad/s 死区并保持至少 0.2 s，零值中断候选方向；稳定正负状态切换计数。
- **低速/停止**：Gazebo 机器人实际前向速度 |v|<0.02 m/s 且 |w|<0.05 rad/s 的累计仿真时间；采样间隔 >0.3 s 不积分。此值包含启动、等待及终点停止，不全是避障等待。
- **D02 恢复运动**：真值确认释放后，实际 v>0.03 m/s 连续至少 0.5 s，取该连续段起点减释放时刻；不存在或运行提前结束为 NA，不填 0。此值包括感知清除、规划、恢复行为与 Controller 反应的综合延迟。
- **Recovery**：NavigateToPose feedback 的 `number_of_recoveries` 最大值；可能包含上下文恢复/清 Costmap，不等价于 Spin/Backup 次数。`nav_events.jsonl` 按 action goal UUID 记录 spin/backup/wait 等状态，`bt_events.jsonl` 和完整 launch log 提供原因线索。未能可靠获得的内部原因不编造。

## 文件与结果

- `scenario.py`、`scenarios.json`、`assets/D01|D02`：场景生成器、配置和物理世界/地图/几何。
- `motion.py`：独立障碍运动辅助类；`observer.py`：只读物理/传感器记录。
- `run_trial.py`：复用静态预检、采集器、BasicNavigator/ActionWorker，增加动态隔离和超时监控。
- `evaluation.py`：独立离线评价；`run_suite.py`、`run.sh`、`start_scene.sh`：无 UI 批量运行与 Controller YAML 切换。
- `../test_dynamic.py`：几何、运动、插件节点唯一性、终止分类、死区回归；由现有 CMake unittest discover 自动纳入。
- `evidence/`：构建/测试/Smoke/正式批次日志、审计脚本及验证结果；`RESULTS.md`：本轮报告。
- `../results/dynamic/<run-id>/`：每次运行的 summary、metrics、trajectory、GT、commands、motion、contacts、perception、sensor gzip、反馈/BT/action、配置快照；批次 manifest、CSV、JSON、aggregate 和 Markdown。

根 `.gitignore` 原本整体忽略 benchmark；本次仅增加动态源码、文档和 evidence 日志的例外，静态目录仍保持原忽略规则。原有 `benchmark/.gitignore` 忽略 `results/*`，因此原始结果虽然保存在仓库目录内，但不会自动被 Git 跟踪；本次未擅自 git add/commit。正式批次的精简 JSON/CSV/审计副本另存于 dynamic/first_run。交接时需同时保留 `benchmark/results/dynamic`，不能只复制 tracked 文件。最初故障 Smoke 的结果从初始 dynamic/results 路径搬入统一 results/dynamic，原命令文本保持原貌。

## 已知限制与研究边界

本次只测原始性能，未修改任何算法或冻结 YAML。每组 n=3 仅作描述；不推断显著性、不宣称全面优越。固定时间启动、AMCL/激光随机性、kinematic 台阶、近似轮廓、有限接触/真值采样和运行调度都限制外推。恢复原因来自公开 feedback/BT/action/log，未解析所有 Nav2 内部状态。D01 首次接触后主动取消，运行未覆盖完整穿越周期是预先规定的碰撞终止规则。保留失败，不重试以挑选更好结果。

方法参考：[DynNav-Bench](https://github.com/AnasAlqadhi/DynNav-Bench) 的确定性动态实体与场景组织思想（MIT）；没有复制其源码或引入强化学习环境。插件接口依据已安装 gazebo_ros 及其[官方 state 实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/src/gazebo_ros_state.cpp)、[contact 实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_bumper.cpp)。