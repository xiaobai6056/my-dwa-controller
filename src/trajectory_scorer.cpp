#include "my_dwa_controller/trajectory_scorer.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "my_dwa_controller/path_geometry.hpp"

namespace my_dwa_controller
{
namespace
{

double checkedCost(long double value)
{
  if (!std::isfinite(value) || value > std::numeric_limits<double>::max()) {
    throw std::overflow_error("trajectory score exceeds double range");
  }
  return static_cast<double>(value);
}

long double meanPathDistance(
  const Trajectory & trajectory, const std::vector<Pose2D> & local_path)
{
  long double mean = 0.0L;
  const long double count = trajectory.poses.size();
  for (const auto & pose : trajectory.poses) {
    // 复用整条折线的最近线段投影，包含位姿校验、单点路径和零长度线段处理。
    // 调用前已拒绝空路径，因此 distanceToPath 必定返回距离或抛出异常。
    mean += distanceToPath(pose, local_path).value() / count;
  }
  return mean;
}

long double endpointDistance(
  const Trajectory & trajectory, const std::vector<Pose2D> & local_path, double lookahead)
{
  const PathProjection start{local_path.front(), 0, 0.0, 0.0};
  const auto reference = lookAheadOnPath(local_path, start, lookahead).value().point;
  const auto & endpoint = trajectory.poses.back();
  return std::hypot(
    static_cast<long double>(endpoint.x) - reference.x,
    static_cast<long double>(endpoint.y) - reference.y);
}

double processHeading(
  const Pose2D & endpoint, const std::vector<Pose2D> & local_path, double lookahead)
{
  const auto projection = projectOntoPath(endpoint, local_path).value();
  const auto reference = lookAheadOnPath(local_path, projection, lookahead).value();
  const long double dx = static_cast<long double>(reference.point.x) - endpoint.x;
  const long double dy = static_cast<long double>(reference.point.y) - endpoint.y;
  constexpr double direction_epsilon = 1e-9;
  double desired;
  if (reference.distance > direction_epsilon && std::hypot(dx, dy) > direction_epsilon) {
    desired = static_cast<double>(std::atan2(dy, dx));
  } else if (reference.forward_heading) {
    desired = *reference.forward_heading;
  } else {
    return 0.0;
  }
  const double pi = std::acos(-1.0);
  return std::abs(std::remainder(desired - endpoint.theta, 2.0 * pi)) / pi;
}

}  // 匿名命名空间

TrajectoryScorer::TrajectoryScorer(
  double max_forward_speed, const ScoreWeights & weights,
  double goal_ref_distance, double path_ref_distance,
  double follow_lookahead, double heading_lookahead)
: max_forward_speed_(max_forward_speed), weights_(weights),
  goal_ref_distance_(goal_ref_distance), path_ref_distance_(path_ref_distance),
  follow_lookahead_(follow_lookahead), heading_lookahead_(heading_lookahead)
{
  for (double parameter : {
      max_forward_speed_, goal_ref_distance_, path_ref_distance_,
      follow_lookahead_, heading_lookahead_})
  {
    if (!std::isfinite(parameter) || parameter <= 0.0) {
      throw std::invalid_argument(
              "speed limit, reference distances and lookaheads must be finite and positive");
    }
  }
  bool has_positive_weight = false;
  for (double weight :
    {weights_.obstacle, weights_.speed, weights_.goal, weights_.path, weights_.heading})
  {
    if (!std::isfinite(weight) || weight < 0.0) {
      throw std::invalid_argument("score weights must be finite and nonnegative");
    }
    has_positive_weight = has_positive_weight || weight > 0.0;
  }
  if (!has_positive_weight) {
    throw std::invalid_argument("at least one score weight must be positive");
  }
}

ScoreResult TrajectoryScorer::score(
  const Trajectory & trajectory, const std::vector<Pose2D> & local_path,
  double mean_obstacle_cost, bool heading_enabled) const
{
  if (trajectory.poses.empty() || local_path.empty()) {
    throw std::invalid_argument("trajectory and local reference path must not be empty");
  }
  if (!std::isfinite(trajectory.command.v) || !std::isfinite(trajectory.command.omega)) {
    throw std::invalid_argument("trajectory command components must be finite");
  }
  if (!std::isfinite(mean_obstacle_cost) || mean_obstacle_cost < 0.0 ||
    mean_obstacle_cost > 252.0)
  {
    throw std::invalid_argument("mean obstacle cost must be finite and within [0, 252]");
  }

  const double v = trajectory.command.v;
  if (v < 0.0 || v > max_forward_speed_) {
    throw std::invalid_argument("candidate speed out of range");
  }

  ScoreResult result{};
  result.obstacle_cost = mean_obstacle_cost / 252.0;
  result.speed_cost = 1.0 - v / max_forward_speed_;
  // 路径计算同时验证全部输入位姿，即使对应权重为零也不跳过。
  result.path_cost = checkedCost(meanPathDistance(trajectory, local_path) / path_ref_distance_);
  result.goal_cost = checkedCost(
    endpointDistance(
      trajectory, local_path,
      follow_lookahead_) / goal_ref_distance_);
  result.heading_cost = heading_enabled ?
    processHeading(trajectory.poses.back(), local_path, heading_lookahead_) : 0.0;
  const long double total =
    static_cast<long double>(weights_.obstacle) * result.obstacle_cost +
    static_cast<long double>(weights_.speed) * result.speed_cost +
    static_cast<long double>(weights_.goal) * result.goal_cost +
    static_cast<long double>(weights_.path) * result.path_cost +
    static_cast<long double>(weights_.heading) * result.heading_cost;
  result.total_cost = checkedCost(total);
  return result;
}

}  // 命名空间 my_dwa_controller
