# 第一版 TrajectoryScorer 设计

## 范围与接口

复用 types.hpp 的 Trajectory、Pose2D 和 path_geometry.hpp 的 distanceToPath。
新增 ScoreWeights、ScoreResult、TrajectoryScorer；保持 C++17 算法库，不增加 ROS 依赖。
已有 Pose2D 本身来自 geometry_msgs，沿用其头文件依赖。

构造签名：`TrajectoryScorer(double max_forward_speed, const ScoreWeights & weights = ScoreWeights{}, double goal_ref_distance = 1.0, double path_ref_distance = 0.5)`。
评分签名：`ScoreResult score(const Trajectory & trajectory, const std::vector<Pose2D> & local_path, double mean_obstacle_cost) const`。

与直接传入 ObstacleEvaluation 相比，只传均值可避免评分器依赖 Costmap；与重新实现线段投影相比，复用现有路径几何工具可统一退化情况及错误处理。
Controller 必须先确认 evaluateTrajectory 返回 collision_free，再把对应 mean_cost 传入 score；评分器不能验证统计值与轨迹的对应关系。
所有输入均位于同一局部地图坐标系（当前 odom）。不负责碰撞检查、TF、消息转换、截取路径或选择轨迹。

## 计算

- 障碍代价为 mean_obstacle_cost / 252.0。
- 第一版只允许前进速度；command.v 必须位于 [0, max_forward_speed]，越界抛出 std::invalid_argument("candidate speed out of range")。合法速度的代价为 1 - command.v / max_forward_speed，不再截断速度。
- 终点代价为轨迹末点到 local_path.back() 的欧氏距离 / goal_ref_distance。
- 路径代价为每个轨迹点到整条折线最近线段的欧氏距离的算术平均值 / path_ref_distance；含初始点、重复点。
- 四项按配置权重直接加权。距离代价不截断到 1，权重不自动归一化。

默认权重依次为 0.50、0.10、0.20、0.20；参考距离为 1.0 m、0.5 m；速度上限由调用者显式指定，与采样器保持一致。

## 边界与异常

速度上限和参考距离必须有限且大于零。权重必须有限、非负，且至少一项大于零；允许关闭单项，允许权重和不等于 1。
空轨迹、空路径、非有限位姿或速度分量、负向或超过上限的候选线速度、非有限或不在 [0,252] 的障碍均值均抛出 std::invalid_argument。
单点路径与零长度线段沿用 distanceToPath 的点距离语义。所有返回项必须有限；溢出抛出 std::overflow_error，沿用路径几何工具的距离平方范围限制。
即使某项权重为零，也验证并计算该项，以保持 ScoreResult 的完整性。

## 测试与工程

新增 PIC 静态库 trajectory_scorer，公开链接 path_geometry，安装并导出目标；新增 ament GTest。
覆盖手算四项及总分、默认和自定义配置、线段内外投影、折线最近段、单点和重复点、速度范围边界与越界异常、代价单调性、所有非法参数和输入、数值溢出。
先运行缺失评分器时失败的测试，再实现并运行所有算法测试及包内静态检查。保留现有模块算法和 Controller 行为。

## 构建与运行

在 WSL Ubuntu-22.04 的 Bash 中执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select my_dwa_controller --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/my_dwa_controller -R '^test_trajectory_scorer$' --output-on-failure
ctest --test-dir build/my_dwa_controller -L gtest --output-on-failure
colcon test --packages-select my_dwa_controller
colcon test-result --test-result-base build/my_dwa_controller/test_results --verbose
```

2026-10-17 验证：构建成功，评分器 19/19 用例通过，全部 5 个算法测试目标通过；cppcheck、lint_cmake、xmllint 通过，新增三份 C++ 文件的 uncrustify 通过。全包 uncrustify 仍报告未修改的 include/my_dwa_controller/controller.hpp 和 src/controller.cpp 两处文件格式问题。

同日按第一版仅前进约定改为速度越界直接抛异常，保留非有限分量检查。先确认越界测试在旧实现下失败，再验证新实现：构建成功，评分器 20/20 用例通过，全部 5 个算法测试目标及本次修改的三份 C++ 文件格式检查通过。速度测试包含 0、上限、中间值，以及刚刚越过两端边界的浮点值。

## 后续 Controller 使用

在 Controller 的配置阶段读取参数并构造评分器；使用与速度采样器一致的最大前向速度。四个权重分别对应 obstacle_weight、speed_weight、goal_weight、path_weight，两个距离对应 goal_ref_distance 和 path_ref_distance。本次仅提供 C++ 构造参数，不注册 ROS 参数。

```cpp
my_dwa_controller::ScoreWeights weights;
my_dwa_controller::TrajectoryScorer scorer(max_forward_speed, weights, 1.0, 0.5);

// 评估地图期间按 ObstacleEvaluator 现有约定加锁。
const auto evaluation = my_dwa_controller::evaluateTrajectory(trajectory, costmap);
if (evaluation.collision_free) {
  const auto result = scorer.score(trajectory, local_path, evaluation.mean_cost);
  // 调用方保存分项结果，并在所有可行候选中比较 result.total_cost。
}
```

同包内目标可链接 trajectory_scorer；其他包通过 find_package(my_dwa_controller REQUIRED)，链接 my_dwa_controller::trajectory_scorer。该目标会传递 path_geometry 和既有 geometry_msgs 类型头文件依赖。调用方按已有异常处理策略处理无效配置、无效参考路径或数值溢出。
