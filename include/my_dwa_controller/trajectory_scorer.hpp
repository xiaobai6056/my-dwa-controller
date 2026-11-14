#ifndef MY_DWA_CONTROLLER__TRAJECTORY_SCORER_HPP_
#define MY_DWA_CONTROLLER__TRAJECTORY_SCORER_HPP_

#include <vector>

#include "my_dwa_controller/types.hpp"

namespace my_dwa_controller
{

struct ScoreWeights
{
  double obstacle = 0.10;
  double speed = 0.10;
  double goal = 0.60;
  double path = 0.20;
  double heading = 0.05;
};

struct ScoreResult
{
  double obstacle_cost;
  double speed_cost;
  double goal_cost;
  double path_cost;
  double total_cost;
  double heading_cost;
};

/// 对已通过障碍物评估的轨迹计算五项代价，所有代价和总分均越小越好。
/// 不访问地图、不检查碰撞、不执行 TF 或路径截取，也不选择最优轨迹。
class TrajectoryScorer
{
public:
  /// 最大前向速度单位为 m/s，参考距离及两项前视距离单位为 m，均必须为有限正数。
  /// 权重必须有限、非负且至少一项为正；直接加权，不自动归一化。
  /// 非法配置抛出 std::invalid_argument；构造后保存配置副本。
  explicit TrajectoryScorer(
    double max_forward_speed, const ScoreWeights & weights = ScoreWeights{},
    double goal_ref_distance = 1.0, double path_ref_distance = 0.5,
    double follow_lookahead = 0.6, double heading_lookahead = 0.6);

  /// trajectory 和 local_path 必须同处局部 Costmap 坐标系（当前为 odom）。
  /// 调用方先检查 collision_free，再传入对应 ObstacleEvaluation::mean_cost。
  /// 障碍代价为均值 / 252；速度代价为 1-command.v / 最大前向速度。
  /// 第一版仅允许前进速度，command.v 必须位于 [0, 最大前向速度]，越界抛出异常。
  /// Goal slot is PathFollow: endpoint distance to the lookahead from local_path.front().
  /// Process Heading uses endpoint projection plus its own arc lookahead; [0, 1].
  /// Exhausted/coincident references use forward segment geometry, not stored yaw.
  /// Directionless paths contribute zero Heading. Controller supplies the terminal gate.
  /// 路径代价为所有轨迹点到路径最近线段的平均距离 / path_ref_distance。
  /// 单点路径退化为点距离；重复轨迹点重复计入；两项距离代价不截断到 1。
  /// 空轨迹、空路径、非有限位姿或速度分量、均值非有限或不在 [0,252]
  /// 均抛出 std::invalid_argument；计算溢出抛出 std::overflow_error。
  /// 路径距离沿用 distanceToPath 对距离平方的 double 范围限制。
  ScoreResult score(
    const Trajectory & trajectory, const std::vector<Pose2D> & local_path,
    double mean_obstacle_cost, bool heading_enabled = true) const;

private:
  double max_forward_speed_;
  ScoreWeights weights_;
  double goal_ref_distance_;
  double path_ref_distance_;
  double follow_lookahead_;
  double heading_lookahead_;
};

}  // 命名空间 my_dwa_controller

#endif  // 头文件保护 MY_DWA_CONTROLLER__TRAJECTORY_SCORER_HPP_
