# Controller 第一阶段整合设计

## 范围与结构

MyDWAController 直接调用现有模块，不增加总调度类，不修改已测试的算法模块。
持有 SamplingConfig、Predictor、Scorer、PathHandler、已有 TF/Costmap 依赖；setPlan 唯一保存 global_plan_。
状态互斥量串行保护配置、生命周期、路径、限速和计算，地图使用其已有 mutex 单次加锁，不在每条轨迹中重复加锁。

## 参数和生命周期

configure 读取 FollowPath 命名空间参数；max_vel_x/max_vel_theta、acc_lim_x/acc_lim_theta、vx_samples/vtheta_samples 沿用 YAML 名称，未提供时使用 SamplingConfig 默认值。
control_dt = 1 / controller_frequency（现有服务器参数，默认 20 Hz），与新增 simulation_dt（默认 0.05 s）分开。
sim_time 沿用 1.7 s；新增 forward_length 默认 2 m，必须不小于 max_vel_x * sim_time。
四个权重、两个参考距离沿用 Scorer 默认值和既定命名。配置在 configure 时读取，不添加动态参数回调。
Sampler 验证配置、Predictor/Scorer 构造验证参数，配置失败不留下可用的半初始化状态。
activate 后允许计算，deactivate 停止计算但保留配置；cleanup 释放模块、TF、地图和路径。
setSpeedLimit 遵守 Nav2 NO_SPEED_LIMIT=0 的取消限速约定；非零绝对值或百分比限制前向上限，同步更新 Sampler 和 Scorer，Predictor 只使用候选速度。超出限速的当前速度不截断，空动态窗口明确失败。

## 计算和失败

检查生命周期、机器人帧等于 Costmap global_frame；先由 PathHandler 校验机器人及路径消息，在当前机器人时间将全局路径投影、截取并输出局部参考路径；不对每条候选做 TF。
消息校验通过后在此局部坐标系中转换为 Pose2D，避免再次复制整套位姿校验；线速度取 speed.linear.x，角速度取 speed.angular.z。
调用 sampleVelocities，再遍历 predict -> evaluateTrajectory -> score；只有碰撞/越界等正常不可行结果被 continue，模块异常不当作过滤。
在同一次地图锁内评估所有候选，total_cost 严格更小时更新最优值，相等时保留最早候选。
返回 TwistStamped：本体坐标系来自 getBaseFrameID()，时间为节点当前时间，写入 linear.x/angular.z，其他分量为零。
Humble 已安装 nav2_core/exceptions.hpp 仅提供 PlannerException：无候选/无可行轨迹使用它；输入/配置 invalid_argument 和 TF 异常明确传播，服务器可分别处理运行失败。
GoalChecker 的目标完成判断仍由 Nav2 服务器执行，本阶段不新增终点朝向控制或恢复状态机。

## 配置与验证

YAML 的 FollowPath 块仅列出实际使用的参数，移除 DWB critics 等不生效项，保留原有限速、加速度、采样数和时域数值。
新增 Controller GTest：真实调用公开接口，人工构造 LayeredCostmap，注入 TF；无需地图更新线程、传感器或 Gazebo。
验证手算的直线最优速度、与所有可行候选评分比较、碰撞过滤、障碍代价影响、旋转位姿/跨帧路径、无候选/全不可行、非法输入、限速及生命周期。
验证 pluginlib 仍可加载 MyDWAController，原有算法全部回归，检查导出和 CMake。
未验证实际导航、时序和实时性能；仍保留现有离散点碰撞检查及缺少终点朝向控制的适用边界。

## 实际参数

下列 FollowPath 参数位于插件命名空间，controller_frequency 位于 Controller Server 命名空间。采样器采用对称加速度约束，不独立实现非对称减速度。

| 参数 | 未提供 YAML 时的默认值 | 当前 YAML |
| --- | --- | --- |
| controller_frequency | 20.0 Hz | 20.0 Hz |
| max_vel_x | 0.22 m/s | 0.26 m/s |
| max_vel_theta | 1.0 rad/s | 1.0 rad/s |
| acc_lim_x | 0.5 m/s² | 2.5 m/s² |
| acc_lim_theta | 2.0 rad/s² | 3.2 rad/s² |
| vx_samples | 5 | 20 |
| vtheta_samples | 11 | 20 |
| sim_time | 1.7 s | 1.7 s |
| simulation_dt | 0.05 s | 0.05 s |
| forward_length | 2.0 m | 2.0 m |
| obstacle_weight | 0.50 | 0.50 |
| speed_weight | 0.10 | 0.10 |
| goal_weight | 0.20 | 0.20 |
| path_weight | 0.20 | 0.20 |
| goal_ref_distance | 1.0 m | 1.0 m |
| path_ref_distance | 0.5 m | 0.5 m |

配置时检查 forward_length >= max_vel_x * sim_time，当前为 2.0 >= 0.442 m。
该关系针对预测弧长上界，不保证参考路径所有点都位于 3 m 局部地图中；候选轨迹的地图越界仍由现有 ObstacleEvaluator 判断。
所有配置读取一次，运行中只有 Nav2 setSpeedLimit 接口调整有效线速度上限；不注册动态参数回调。
NO_SPEED_LIMIT=0.0 表示取消限速，恢复配置上限；百分比大于 100、负数和非有限值明确拒绝。

## 构建与运行

在 WSL Bash 中执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select my_dwa_controller
ctest --test-dir build/my_dwa_controller -R '^test_controller$' --output-on-failure
ctest --test-dir build/my_dwa_controller -L gtest --output-on-failure
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 colcon test --packages-select my_dwa_controller
colcon test-result --test-result-base build/my_dwa_controller/test_results --verbose
```

测试通过 pluginlib 加载本次安装的 Controller 插件；因此先 colcon build 完成安装，再执行 ctest。
CMake 测试环境自动追加当前安装前缀；不直接链接含插件导出宏的共享库，避免重复注册。
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS 只在命令中临时设置，让系统 cppcheck 2.7 实际运行而非被 ament 跳过。

## 2026-10-17 实际验证

- colcon build 成功，无编译警告。
- Controller 19/19 测试通过，包含实际 YAML 加载、手算最低总分、所有可行候选比较、碰撞过滤、软障碍代价、人工旋转平移 TF、当前速度、不同控制周期与预测步长、限速、生命周期、空路径/无候选/全不可行和非法输入。
- 原有独立算法 127/127 用例通过，全部 GTest 共 146 个用例通过。
- 全包 test-result：193 tests, 0 errors, 0 failures, 0 skipped；包括实际执行的 cppcheck、uncrustify、lint_cmake、xmllint。本次 Controller 改写同时消除了之前两个 Controller 文件的格式问题。
- 插件导出类名、Nav2 公共接口保持，动态加载测试通过。YAML 安装到 share/my_dwa_controller/config。
- 只读代码复核未发现 P1/P2 问题；未修改 VelocitySampler、Predictor、ObstacleEvaluator、Scorer、PathHandler 或 path_geometry 的算法实现。
- 未启动 Gazebo；未测真实传感器、动态地图并发更新、控制周期耗时及实际导航成功率。

## 首次 Gazebo 闭环前

确认实际 launch 加载更新后的参数 YAML 和 MyDWAController 插件，确认仿真时钟、TF、里程计、本体坐标系、局部地图及速度指令话题配置匹配。
首次从低速、短直线、目标朝向与路径一致的场景检查闭环；再验证障碍过滤和失败报告，不把本阶段基础测试视为导航成功证明。
现有障碍评估仍是离散点检查，不包含 footprint 或连续轨迹段碰撞检测；本阶段没有专门的终点朝向控制。任意目标朝向可靠到达及更完整碰撞模型属于后续工作。
