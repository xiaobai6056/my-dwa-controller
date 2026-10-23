# MyDWA / DWB 静态导航 Benchmark V1

目标是 Nav2 端到端对照（NavigateToPose），不是单独控制器的因果性能证明。
Controller C++、两份核心参数 YAML、全局规划器、GoalChecker、costmap 和速度约束
没有改动。旧单次 CLI 仍可原样使用；新增采集仅在 `--record-navigation` 启用。

## 场景、资源和坐标

用户确认的混合方案见 `scenarios.yaml`，所有起终点使用 map 坐标、yaw 使用 rad。

|ID|目的|起点 x,y,yaw|目标 x,y,yaw|超时/墙钟 s|几何依据|
|---|---|---|---|---:|---|
|S01|长直线路径跟踪|−2,−0.55,0|2,−0.55,0|90|stock 两排圆柱间 4 m 直线|
|S02|90° 转弯|0.5,0,0|5,4.5,π/2|150|1.6 m 宽 L 形走廊，中心线名义长 9 m|
|S03|连续双向转向|0.5,0,0|8.5,0,0|240|强制依次经 (3,0)→(3,3)→(6,3)→(6,0)，中心线名义长 14 m|
|S04|静态绕障|−2,0,0|2,0,0|120|直连目标的线段穿过 y=0 的三根 stock 圆柱|
|S05|窄通道进入/通过/退出|−1,0,0|7,0,0|180|6 m 长、0.8 m 宽通道，两端有入口房间，移动约 8 m|
|S06|终点减速和旋转|−2,0.55,0|1.9,0.55,π|120|约 3.9 m 平移后要求相反终点朝向，审计恢复 Spin|

stock 使用已安装的 `nav2_bringup/worlds/world_only.model` 和
`maps/turtlebot3_world.yaml`。场地约 6 m 见方，圆柱中心 x/y∈{-1.1,0,1.1}，
半径 0.15 m。检查图为 `evidence/stock_map_geometry.png`。
S02/S03/S05 的 SDF、PGM、map YAML、geometry.json 在 `assets/<ID>/`。
S03 是多次连续转弯路线，不是数学正弦曲线；走廊墙体阻断直连捷径，Navfn
仍可在走廊内平滑切弯，所以实际路径会短于名义中心线（pilot 约 11 m）。

`scenes.py` 从同一组自由矩形生成相反区域的静态 Gazebo 盒体和 0.05 m 栅格。
没有任何自由栅格位于障碍物内部。地图只把可观测墙面标为占据，实体墙内为
未知，不是自由空间；封闭占据轮廓隔离内部区域。S02/S03 有不缩窄通道的
不对称小凹槽，提供定位特征。ODE physics 整段复制自 stock World，机器人
仍是同一 `waffle.model`，没有替换引擎或模型。生成资源可重复执行，正式结果
中的资源快照不可被重新生成步骤覆盖。

早期把厚实体墙的所有内部格子也标为占据，AMCL 的 likelihood field 会在墙内
同样产生零距离，pilot 中两种 Controller 均发生严重定位漂移。已检查当时实际
发布的地图与几何 26244 格完全对应，激光与物理墙面的平均残差约 8 mm。
最终改成表面占据、内部未知后，MyDWA S02 pilot 成功且无恢复。旧失败 trial
保留在 `results/v1_pilots/`，带原配置快照，不能混入正式版本统计。

## 运行

在 WSL Ubuntu-22.04 Bash 中执行，先保证没有其他使用同一 Gazebo master 的
手动实验。批量默认 ROS domain=77，并使用专属 Gazebo master 端口 11422。
仅需要 ROS 已有的 PyYAML；指标计算模块本身只用标准库。

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws/src/my_dwa_controller/benchmark
python3 scenes.py generate
python3 -m unittest discover -p 'test_*.py' -v

# 全部六场景、双 Controller、每组 3 次，实际执行过：
python3 run_suite.py --controllers mydwa dwb \
  --scenes S01 S02 S03 S04 S05 S06 --repeats 3 --run-id static_v1_20261019

# 自定义选择；省略 run-id 会自动产生不重复时间 ID：
python3 run_suite.py --controllers mydwa --scenes S01 S06 --repeats 1
```

已有 run-id 绝不覆盖。正式结果位于
`results/v1/static_v1_20261019/<controller>/<scene>/trial_00N/`。
批量 manifest 包含全部已调度任务，即使启动失败或没有有效数据也保留。
`logs/` 包含每次 launch 和单次 runner 的完整标准输出。
每次结束关闭 runner，再让 launcher 执行已有的有界进程清理；异常也执行清理，
继续下一项，不调用未经验证的 Gazebo reset 服务。每个重复中交替先后 Controller。
启动、预检、导航和取消均有墙钟总期限；脚本崩溃/启动失败至少产生带原因的
summary 和只有表头的 CSV。批量任务中不要另启同一 domain 的 baseline。

若只调试一个场景，两个终端使用同一 ROS domain：

```bash
# 终端 A：每次重测 Ctrl+C 停止并等清理完成，再启动，恢复物理/定位初值。
bash /home/user/ros2_ws/src/my_dwa_controller/benchmark/start_scene.sh mydwa S02

# 终端 B（同样 source ROS 和 overlay）：
cd /home/user/ros2_ws/src/my_dwa_controller/benchmark
python3 run_trial.py --controller mydwa --start 0.5 0 0 \
  --goal 5 4.5 1.5707963267948966 --timeout 150 --record-navigation \
  --world-file /home/user/ros2_ws/src/my_dwa_controller/benchmark/assets/S02/world.sdf \
  --startup-script /home/user/ros2_ws/src/my_dwa_controller/benchmark/start_scene.sh
```

初始化设置实际 spawn 的 x/y/yaw，并等待 AMCL active 后以同一位置设置初值。
runner 不替换导航执行逻辑，仍使用 BasicNavigator 发送 Goal、检查完成、取消和
读取真实结果。预检核验 lifecycle、Controller 插件、BT FollowPath 绑定、TF、
clock、起点及必要话题；V1 还读取起点 Gazebo 物理位置，偏离预设超过 0.15 m
拒绝执行。map 起点容差仍为旧 runner 的 0.3 m / 0.3 rad。

AMCL 初始位姿服务若已执行但 CLI 回复丢失，launcher 记录警告并保留仿真，不因
回复超时关闭正在执行的任务，也不重发导致导航中途重置。警告不代表初始化成功；
runner 的 lifecycle、TF 和起点预检仍必须通过，否则拒绝发送 Goal。
正式批量后额外 smoke 发现并修正此启动路径问题；原失败记录保留，与正式 36 次分列。
旧清理器另修复一次 `/proc` 非原子快照可能遗漏新子进程的竞态，专属组暂空时
重新扫描确认，6 项回归覆盖该边界及真实进程树；不改变 UI 入口或信号/域筛选规则。

## 原始数据与导航系统归因

每个 trial 保存：

- `trajectory.csv`、`summary.json`：兼容旧版本，map-frame TF 位姿、odom child-frame
  实测 twist、Controller `/cmd_vel_nav` 和机器人 `/cmd_vel` 指令分列。
- `plans.jsonl`：每次 `/plan` 消息、全部折线点、源帧/时间戳、接收时仿真时间和几何 hash。
- `nav_events.jsonl`：FollowPath、ComputePathToPose、Spin、BackUp、DriveOnHeading、
  Wait 的 Action UUID 和状态变化；可靠、transient-local QoS，发 Goal 前确认服务端发布者存在。
- `bt_events.jsonl`：BehaviorTreeLog 的节点状态；BT 内部时间戳可能为墙钟，不能当仿真时间。
- `feedback.jsonl`：NavigateToPose 导航时长、剩余距离、恢复计数的原始反馈。
- `localization.jsonl`：AMCL 位姿/协方差，以及明确标记 odom 帧的原始里程计位姿。
- `configs/`：World、地图图像/YAML、机器人 SDF、BT、参数文件、执行脚本和计算源码快照。
- `metrics.json`：指标定义参数、输入文件 SHA256、计算源码标识和可用数据覆盖时间。

summary 同时保存全部关键节点的实际运行参数，以及起终点 `gz model -p` 的 Gazebo
world 位置和查询耗时。终点 world/map 差异超过 0.5 m 会产生定位警示，不能把此时
map 轨迹误差简单解释为 Controller 跟踪误差。该异步诊断不是碰撞或净间距测量。
World/map 原点对应关系由场景定义建立；stock 的栅格地图本身仍有建图/定位误差。

Humble 在重新规划时可能把被新路径抢占的旧 FollowPath Goal 标为 ABORTED。
因此 raw Action ABORTED 不自动等同于控制器故障；指标明确提示这一语义，应结合
BT 节点 FAILURE、恢复反馈和 launch 日志。`replanning_publications` 是首次计划
之后的发布次数，不等价于路径拓扑改变次数。几何 hash 因路径起点前移也会变化。

S06 的归因要求存在近终点（0.30 m 内）低线速（|vx|≤0.03）、非零旋转
（|wz|≥0.05）的采样，且恢复动作审计已就绪、没有任何恢复 Goal 的证据。
即使错过 EXECUTING，只收到 Spin 的终态也会标为 `unconfirmed`。
MyDWA pilot 额外有日志 `goal_phase=rotating`；pilot 无 Spin、无恢复反馈，观察到
49 个近终点旋转样本。任何使用恢复 Spin 的 trial 都不能据此证明终点 Controller 逻辑。

## 指标的精确定义

主参考路径为该次导航首次有效的 map-frame `/plan`，之后固定，所有重规划保留
供分析。两种 Controller 的端到端初始计划受定位误差影响可能不同，本轮没有
把它伪装成完全相同参考的纯跟踪实验。固定参考 FollowPath（概念 B）暂未执行。

|指标|定义|
|---|---|
|成功/失败|真实 NavigateToPose Action 结果；另列数据 valid、预检/超时/异常原因，绝不从距离推断成功|
|主表仿真耗时|首次 ComputePathToPose Goal 的原生接收时间戳，到检测结果/停止时最近一次收到的 `/clock`；不是精确的 NavigateToPose Action 时长，详见下文|
|轨迹长度|相邻有效 map TF 位姿的平面距离之和；缺失段不连线、不补点，定位跳变可能影响长度|
|横向 RMSE|点到参考折线每个**线段投影**的最小距离 e；sqrt(∫e²dt/∫dt)，对实际 TF 源时间做梯形积分|
|最大横向误差|有效采样点的最大折线段距离；不是连续时间最大值的严格上界|
|速度均值/标准差|实测及两路指令分别统计，∫vdt/T 与 sqrt(∫v²dt/T−mean²)|
|速度变化|总变差 Σ|Δv|、Σ|Δv|/T、RMS 加速度 sqrt(Σ(Δv/Δt)²Δt/T)、最大 |Δv/Δt|；角速度同理|
|角速度方向切换|默认排除 |wz|<0.05 rad/s；新符号连续保持至少 0.15 s 才确认，统计确认后的正负切换|
|终点误差|Action 结束时有效 map TF 的平面欧氏距离、wrap 到 [-π,π] 后的绝对朝向差；不是完全停稳后的误差|

速度为差速机器人的有符号车体 vx 和 wz，不把 odom twist 当成 map 速度。
实测速度用 odom 源时间，指令 Twist 没有源 header，使用已保存的接收仿真时间。
采样约 10 Hz，统计不能反映采样频率以上的快速抖动。TF/odom 时间戳可略领先
10 Hz 的 clock 接收值，保留旧 runner 的 0.15 s 允许范围。

正式运行后发现，起点 Gazebo 诊断查询期间没有执行 Collector 回调，旧版
`summary.navigation_sim_s` 会包含查询前缓存的 clock 起点（本次多出 0.4–0.7 s）。
原始 summary/CSV 不改写；离线 `source_window_v2` 保存三种时间：
`timing.first_planner_to_observed_stop_sim_s` 用于主表，
`legacy_phase_clock_span_s` 保留原跨度，`nav2_last_feedback_duration_s` 保留最后
反馈（反馈可能早于结果）。首次规划时间来自 Action GoalInfo 原生 stamp，排除
trial 时间外或未接收的 Goal，并按 UUID 去重。缺少依据时记 null 和原因，不推造时间。
这一定义不含导航请求分发至规划的延迟，末端受约 0.1 s clock 接收量化影响；
超时项包含取消至观测结束阶段，不应解释为完成任务的速度。

各流按自己的 TF/odom/指令接收时间排除首次规划前缓存的采样，置为缺失断点，
不能按 CSV 行时钟统一筛选。保留直到结果检测期间实际采集的尾部；源时间可能
略领先最后 clock，不强行截掉。`integrated_time_s` 分别报告位姿及各速度流的
有效覆盖时长，不能把它们假设为完全相同的积分区间。旧数据没有规划原生时间
时保留旧采集窗口并标明缺失依据。最终计算源码另保存在 suite 的
`analysis_sources/`，指标含源码 SHA256，试验时的 `configs/` 原快照保持不变。

所有积分使用真实 Δt，不假定等间隔。时间重复/倒退、缺失数据和超过默认 0.5 s
的间隔不用于积分/差分；时间间隔断开时不跨段计算方向切换。没有可用区间的
长度、变差、切换次数为 null，不混作 0；真实观察到的静止运动可以为 0。
每项提供覆盖时间/n，失败 trial 的局部轨迹也保留，不能把短失败解释为导航更快。

`safety.minimum_clearance_m` 和 `collision_count` 为 null，含明确缺失原因。
尚无经过验证的完整机器人碰撞 footprint 和接触检测；激光最小量程、costmap
半径、膨胀距离或机器人中心到墙距离均不冒充真实安全净间距。

## 公平性、重算和统计

启动前解析两份 YAML，去掉算法自身 FollowPath 配置后比较全部共用设置，并
单独核对最大线/角速度 0.26 m/s、1.0 rad/s，以及加速度 2.5 m/s²、3.2 rad/s²。
运行后按场景审计实际参数（排除算法各自评分/采样项）、相同四个约束、起终点、
机器人、World、地图图像、地图 YAML、BT 的 hash。只有两个 Controller 都有
实际有效预检记录且签名一致，`common_runtime_matches` 才为 true；证据不足不会
被单一 Controller 的签名误判为公平通过。

```bash
cd /home/user/ros2_ws/src/my_dwa_controller/benchmark
# 不启动仿真，重新计算单次指标：
python3 metrics.py results/v1/static_v1_20261019/mydwa/S01/trial_001
# 不启动仿真，重算所有已有 trial 指标并聚合：
python3 run_suite.py --aggregate results/v1/static_v1_20261019
cat results/v1/static_v1_20261019/report.md
python3 -m json.tool results/v1/static_v1_20261019/aggregate.json
```

聚合保留全部 scheduled trial、Action 成功次数、无效数据数、失败原因。
`all_observed` 包含成功与失败的已观测数值，缺失项不凑数，每项都有 n、均值、
样本标准差（n−1）、最小/最大值。另给出 `successful_valid`，不可隐去失败计数
只展示成功者。仅三次重复的描述性结果不支持统计显著性或普遍优越性结论。

34 项 Python 测试包括不等间隔解析计算、线段投影、wrap、缺失/静止区别、
去噪切换、map/world 几何一致、失败聚合、公平性资源标识、状态遗漏时的保守归因，
以及旧 runner 的全部测试。C++ 功能测试沿用；历史 `obstacle_evaluator.cpp`
uncrustify 问题单独列出，不修算法文件来隐藏已有格式故障。

```bash
# 构建以及 C++ / Python / lint（已实际执行；历史 uncrustify 项仍失败）：
cd /home/user/ros2_ws
colcon build --packages-select my_dwa_controller --symlink-install
ROS_DOMAIN_ID=97 colcon test --packages-select my_dwa_controller --event-handlers console_direct+
colcon test-result --test-result-base build/my_dwa_controller/test_results --verbose

# 单独验证数据、核心文件未变及自动化测试：
bash /home/user/ros2_ws/src/my_dwa_controller/benchmark/evidence/check_v1.sh
```

最终正式数值与每次原始配置以 `results/v1/static_v1_20261019/report.md`、
`aggregate.json`、`manifest.json` 和各 trial 文件为准。场景 pilot 记录用于验收
环境，不作为正式三次重复统计的额外样本。
本次解读与完整指标表另见 `V1_RESULTS.md`。
