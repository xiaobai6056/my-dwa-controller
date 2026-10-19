#include "my_dwa_controller/trajectory_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace my_dwa_controller
{

TrajectoryPredictor::TrajectoryPredictor(double sim_time, double dt)
: sim_time_(sim_time), dt_(dt)
{
  if (!std::isfinite(sim_time_) || sim_time_ <= 0.0) {
    throw std::invalid_argument("sim_time must be finite and positive (seconds)");
  }
  if (!std::isfinite(dt_) || dt_ <= 0.0) {
    throw std::invalid_argument("dt must be finite and positive (seconds)");
  }
}

Trajectory TrajectoryPredictor::predict(const Pose2D & initial_pose, const Velocity2D & cmd) const
{
  if (!std::isfinite(initial_pose.x) || !std::isfinite(initial_pose.y) ||
    !std::isfinite(initial_pose.theta))
  {
    throw std::invalid_argument("initial_pose components must be finite");
  }
  if (!std::isfinite(cmd.v) || !std::isfinite(cmd.omega)) {
    throw std::invalid_argument("command components must be finite");
  }

  Trajectory trajectory{cmd, {initial_pose}};
  Pose2D pose = initial_pose;
  // 按步数计算时间网格，避免反复累加 dt 引起舍入误差。
  const long double horizon = sim_time_;
  const long double interval = dt_;
  const long double time_tolerance = 8.0L * std::numeric_limits<double>::epsilon() * horizon;
  long double elapsed = 0.0L;
  std::size_t step_index = 1;

  while (elapsed < horizon) {
    long double next_time = std::min(step_index * interval, horizon);
    // 对于 0.9 / 0.3 等十进制输入，避免产生额外的极小尾部时间间隔。
    // 容差随预测总时长缩放，使极小的正数预测时长仍能正常计算。
    if (horizon - next_time <= time_tolerance) {
      next_time = horizon;
    }
    const double step = static_cast<double>(next_time - elapsed);

    // 前向 Euler 积分：两个位置分量均使用本步开始时的朝向更新。
    pose.x += cmd.v * std::cos(pose.theta) * step;
    pose.y += cmd.v * std::sin(pose.theta) * step;
    pose.theta += cmd.omega * step;
    trajectory.poses.push_back(pose);

    elapsed = next_time;
    ++step_index;
  }

  return trajectory;
}

}  // 命名空间 my_dwa_controller
