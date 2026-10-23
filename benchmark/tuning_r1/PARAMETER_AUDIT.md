# 第一轮参数审计

审计对象：`config/nav2_params.yaml`、`src/controller.cpp`、采样/预测/评分/障碍模块、
`include/my_dwa_controller/velocity_sampler.hpp` 与 `trajectory_scorer.hpp`。
“默认”指 MyDWA 代码默认值，不是 Nav2 或 DWB 默认。所有数值必须有限。

## 全部 MyDWA 插件参数

参数名均位于 `controller_server.ros__parameters.FollowPath`。
`plugin=my_dwa_controller::MyDWAController` 由 Nav2 pluginlib 选择，无 MyDWA 内部默认。

|参数|代码默认|当前 YAML|实际限制与含义|
|---|---:|---:|---|
|max_vel_x|0.22|0.26|有效配置必须 >0，m/s；仅采样前进 [0,max]；采样器自身允许 0，但 scorer 构造拒绝 0|
|max_vel_theta|1.0|1.0|>0，rad/s；正负角速度幅值上限|
|acc_lim_x|0.5|2.5|>0，m/s²；动态窗的加/减速共用幅值，终点制动也使用|
|acc_lim_theta|2.0|3.2|>0，rad/s²；角速度动态窗及角制动使用|
|vx_samples|5|20|整数 ≥2，无显式上限；含区间两端，可达零值补入后去重|
|vtheta_samples|11|20|整数 ≥2，无显式上限；补零可变成 21 个角样本|
|sim_time|1.7|1.7|>0，s；恒速度预测时域，同时影响终点角速度上限 abs(yaw_error)/sim_time|
|simulation_dt|0.05|0.05|>0，s；Euler 预测步长，和控制周期不同；代码不强制 ≤sim_time|
|forward_length|2.0|2.0|≥max_vel_x×sim_time（当前 ≥0.442 m）；截取参考路径弧长，允许参考超出局部地图|
|follow_lookahead|0.6|0.6|>0，m；从局部路径起点沿弧长前看的跟随目标|
|heading_lookahead|0.6|0.6|>0，m；从预测末端在路径上的投影再沿弧长前看，用于过程朝向|
|obstacle_weight|0.10|0.10|≥0；平均栅格代价/252 的权重；不改变硬碰撞拒绝|
|speed_weight|0.10|0.10|≥0；1−v/max_v 的权重|
|goal_weight|0.60|0.60|≥0；实际是预测末端到 follow_lookahead 跟随点的距离/goal_ref_distance，不是终点距离|
|path_weight|0.20|0.20|≥0；全部预测点到参考折线的平均距离/path_ref_distance|
|heading_weight|0.05|0.05|≥0；过程朝向误差/π，单项位于 [0,1]；终点阶段关闭|
|goal_ref_distance|1.0|1.0|>0，m；跟随目标距离项的归一化尺度，不是到达容差|
|path_ref_distance|0.5|0.5|>0，m；路径距离项归一化尺度|
|trans_stopped_velocity|0.02|0.02|>0，m/s；Slowing→Rotating 还要求精确零平移在动态窗内可达|
|terminal_xy_tolerance|0.25|0.25|>0，m；仅 GoalChecker 无法提供合法 XY 容差时的 fallback|
|terminal_yaw_tolerance|0.05|0.05|0<值≤π，rad；仅 GoalChecker 无合法 yaw 容差时的 fallback|

五个权重至少一个 >0，直接相加，不自动归一化（当前和为 1.05）。距离项不截断到 1。
`controller_frequency` 是 ControllerServer 级参数，MyDWA 默认/当前均为 20 Hz，必须 >0；
configure 时一次性换算 `control_dt=0.05 s`。插件无动态参数回调，在线 `ros2 param set`
不能作为算法已使用新值的证据；任何未来候选都应重新启动/配置插件并核验实际运行参数。

## 普通评分、可行性与计算负载

动态窗为 v∈[max(0,v_current−acc_x×0.05),min(max_v,v_current+acc_x×0.05)]，
w 同理限制于 ±max_w。当前 Δv=0.125 m/s、Δw=0.16 rad/s。
20×20 网格常因补零成为 420 条轨迹；1.7/0.05 为 34 个预测间隔、35 个位姿。
最大预测前移 0.442 m，.6 m 跟随点通常在其前方；没有“时域一定过长”的依据。

预测点落在栅格 cost≥253（inscribed/lethal/unknown）、地图外或非有限时硬拒绝。
该实现读取预测中心点的膨胀栅格，没有完整多边形 swept-footprint 安全证明。
权重不能让被硬拒绝的轨迹重新有效。不能以提高 obstacle_weight 宣称更安全。

S04 失败 trial_001 在已记录的普通阶段没有零可行候选周期，停滞时仍有有效零速度轨迹。
成功 trial_003 则曾短暂出现 400 条全被 inscribed 拒绝，警告跨度约 0.16 s，后续继续，
未成为持续故障；限频日志不能给出完整不可行区间。
诊断 INFO/WARN 各约 1 Hz 节流，只能统计已记录周期，不能宣称完整逐周期候选分布。
普通阶段 S04 的已记录计算耗时常约 60–80 ms，超过 20 Hz 的 50 ms 预算；这是独立的
计算负载问题。终点停滞周期约 0.1 ms，因此它不能解释旋转完成后的长时间零输出。
减少采样可能减少负载，但不是本次状态冲突的修复，而且会降低候选分辨率，故未搜索。

## 终点与 Nav2 共用配置

- 正常阶段距离≤实际 GoalChecker XY 容差后锁存 Slowing；可停止后锁存 Rotating。
  Slowing 按最小可行 v 优先，Rotating 只允许 v=0；两者评分仅比较终点 yaw，五权重不参与。
- 同 frame、同终点 XY/yaw（1e-6 内）的 setPlan 保留锁存；没有“距离重新超限后恢复平移”分支。
- 实际 SimpleGoalChecker：XY=0.25 m、yaw=0.25 rad、stateful=true。
  S04 日志 tolerance_fallback=[0,0]；修改 MyDWA fallback 不会改变本次行为。
- 实际 SimpleProgressChecker：0.5 m / 10 s，failure_tolerance=0.3 s。
  旋转不产生 XY 进展；延长时间只能推迟错误，不能解除 v=0。保留这些验收规则。
- local/global robot_radius=0.22 m、inflation_radius=0.55 m、cost_scaling_factor=3.0，
  与 DWB 共用；本轮全部未变，也没有缩小 footprint 或放松碰撞检查。

## 与 DWB 的运动学可比性

两者 max_vel_x=0.26、max_vel_theta=1.0、acc_lim_x=2.5、acc_lim_theta=3.2，
共用 20 Hz ControllerServer 与同一 velocity_smoother。
DWB decel_lim_x=-2.5、decel_lim_theta=-3.2 与 MyDWA 对称动态窗幅值一致；DWB vy=0。
MyDWA 不采样倒车；smoother 允许负 vx 是为了 Nav2 恢复 BackUp，不代表 Controller 会倒车。
DWB 预测生成器/critics 与 MyDWA 并不相同，采样数相等也不意味着等价算法。
DWB 的 trans_stopped_velocity=0.25 是 RotateToGoal 的配置，不能当成 MyDWA 0.02 的同义阈值。

速度限制是指令/生成器约束，不是 Gazebo 实测速度的绝对上界。V1 S04 的实际 vx 峰值：
MyDWA 0.31070、DWB 0.28345 m/s；本轮原参数复跑峰值 0.31189 m/s。
两路指令均≤0.26 m/s、|w|≤1.0 rad/s（浮点舍入容差内）。仿真动力学、采样和反馈造成
实测与指令差异，当前没有新增参数，不能宣称已消除运动学超调或获得真实机器人安全认证。

## 候选门槛与结论

|参数类别|原始值|本轮候选值|为何不进入候选实验|潜在副作用|
|---|---|---|---|---|
|普通五权重|.10/.10/.60/.20/.05|未设定，保留原值|异常发生在不使用五权重的 rotating；改权重最多改变进入该状态的路线|改变原本正常的 S03/S05 跟踪，掩盖潜在状态问题|
|采样密度|20/20|未设定，保留原值|停滞不是障碍导致无解，而是非零 v 被状态规则拒绝|降密度影响转弯/窄通道；加密度加重超周期|
|预测时域/步长|1.7/.05 s|未设定，保留原值|缩短时域虽会加快终点转向，但不消除锁存与重置不一致|改变普通预测且可能仅让本次导航侥幸早于定位更新结束|
|终点 fallback|.25 m/.05 rad|未设定，保留原值|实际 GoalChecker 已提供 .25/.25，fallback 不生效|误以为已改善实际判定|
|Goal/ProgressChecker|.25/.25、.5 m/10 s|不调整|放宽验收或延长停滞不是真正修复|掩盖不可达的最终状态|

本轮候选数为 **0**。用户允许诊断发现算法/坐标/定位交互后停止无依据调参。
下一步应在单独的算法修复任务中定义终点状态与 Nav2 重规划/定位修正的同步规则，
并覆盖 S06 与恢复后重新进入平移的组合回归，本轮不直接改实现或原有期望。
