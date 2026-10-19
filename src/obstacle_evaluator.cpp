#include "my_dwa_controller/obstacle_evaluator.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"

namespace my_dwa_controller
{

ObstacleEvaluation evaluateTrajectory(
  const Trajectory & trajectory, const nav2_costmap_2d::Costmap2D & costmap,
  ObstacleRejection * rejection)
{
  // Observe the existing decision path; no extra costmap traversal.
  const auto reject = [rejection](ObstacleRejection reason) {
      if (rejection) {
        *rejection = reason;
      }
    };
  reject(ObstacleRejection::None);
  if (trajectory.poses.empty()) {
    reject(ObstacleRejection::Empty);
    return {};
  }

  unsigned char max_cost = 0;
  double total_cost = 0.0;
  for (const auto & pose : trajectory.poses) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
      reject(ObstacleRejection::NonFinite);
      return {};
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap.worldToMap(pose.x, pose.y, mx, my)) {
      reject(ObstacleRejection::OutOfBounds);
      return {};
    }
    const unsigned char cost = costmap.getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      reject(cost == nav2_costmap_2d::NO_INFORMATION ? ObstacleRejection::Unknown :
        cost == nav2_costmap_2d::LETHAL_OBSTACLE ? ObstacleRejection::Lethal :
        ObstacleRejection::Inscribed);
      return {};
    }
    max_cost = std::max(max_cost, cost);
    total_cost += cost;
  }

  // 全部点可行后才填写统计值，提前返回时保持默认不可行结果。
  return {true, max_cost, total_cost / static_cast<double>(trajectory.poses.size())};//未来考虑行驶距离或时间加权
}

}  // 命名空间 my_dwa_controller
