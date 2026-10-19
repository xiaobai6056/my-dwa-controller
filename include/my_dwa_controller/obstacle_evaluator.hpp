#ifndef MY_DWA_CONTROLLER__OBSTACLE_EVALUATOR_HPP_
#define MY_DWA_CONTROLLER__OBSTACLE_EVALUATOR_HPP_

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "my_dwa_controller/types.hpp"

namespace my_dwa_controller
{

// Diagnostic output only: records the existing early-return branch.
enum class ObstacleRejection {None, Empty, NonFinite, OutOfBounds, Inscribed, Lethal, Unknown};

struct ObstacleEvaluation
{
  bool collision_free = false;
  unsigned char max_cost = 0;
  double mean_cost = 0.0;
};

/// 前置条件：所有 poses 已位于 Costmap 的坐标系中（当前局部地图为 odom）。
/// 前置条件：调用方负责地图访问加锁，保证整个评估期间可安全读取地图。
/// 只检测离散位姿点，不执行 TF、轨迹段插值或机器人 footprint 碰撞检测。
/// 空轨迹、非有限位姿、地图越界或内切膨胀阈值及以上的代价（含未知区域）均不可行。
/// 可行时返回最大代价和按轨迹点计算的平均代价，重复经过同一栅格也重复计入。
/// 不可行时返回默认结果；本函数不修改轨迹或地图，也不缓存评估结果。
ObstacleEvaluation evaluateTrajectory(
  const Trajectory & trajectory, const nav2_costmap_2d::Costmap2D & costmap,
  ObstacleRejection * rejection = nullptr);

}  // 命名空间 my_dwa_controller

#endif  // 头文件保护 MY_DWA_CONTROLLER__OBSTACLE_EVALUATOR_HPP_
