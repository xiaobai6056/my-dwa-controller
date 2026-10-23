#include "my_dwa_controller/obstacle_evaluator.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"

namespace my_dwa_controller
{

ObstacleEvaluation evaluateTrajectory(
  const Trajectory & trajectory, const nav2_costmap_2d::Costmap2D & costmap)
{
  if (trajectory.poses.empty()) {
    return {};
  }

  unsigned char max_cost = 0;
  double total_cost = 0.0;
  for (const auto & pose : trajectory.poses) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
      return {};
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap.worldToMap(pose.x, pose.y, mx, my)) {
      return {};
    }
    const unsigned char cost = costmap.getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return {};
    }
    max_cost = std::max(max_cost, cost);
    total_cost += cost;
  }

  // 全部点可行后才填写统计值，提前返回时保持默认不可行结果。
  return {true, max_cost, total_cost / static_cast<double>(trajectory.poses.size())};//未来考虑行驶距离或时间加权
}

}  // 命名空间 my_dwa_controller
