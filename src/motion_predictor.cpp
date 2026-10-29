#include "my_dwa_controller/motion_predictor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace my_dwa_controller
{
MotionPredictor::MotionPredictor(const MotionConfig & config)
: config_(config)
{
  for (double value : {config.linear_acceleration, config.angular_acceleration,
      config.horizon, config.step})
  {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument("positive motion limits required");
    }
  }
  if (!std::isfinite(config.response_delay) || config.response_delay < 0) {
    throw std::invalid_argument("nonnegative response delay required");
  }
}

MotionRollout MotionPredictor::predict(
  const Pose2D & initial, const Velocity2D & measured,
  const Velocity2D & command, double execute_for) const
{
  for (double value : {initial.x, initial.y, initial.theta, measured.v, measured.omega,
      command.v, command.omega, execute_for})
  {
    if (!std::isfinite(value)) {throw std::invalid_argument("finite motion input required");}
  }
  if (execute_for < 0) {throw std::invalid_argument("negative command duration");}
  const auto approach = [](double from, double to, double change) {
      return from + std::clamp(to - from, -change, change);
    };
  const double delay = config_.response_delay, switch_time = delay + execute_for;
  const double a = config_.linear_acceleration, alpha = config_.angular_acceleration;
  const Velocity2D at_switch{
    approach(measured.v, command.v, a * execute_for),
    approach(measured.omega, command.omega, alpha * execute_for)};
  const auto stop_time =
    [delay, switch_time](double start, double target, double end, double rate) {
      if (end != 0) {return switch_time + std::abs(end) / rate;}
      if (start == 0) {return 0.0;}
      return target == 0 ? delay + std::abs(start) / rate : switch_time;
    };
  const double linear_stop = stop_time(measured.v, command.v, at_switch.v, a);
  const double angular_stop = stop_time(measured.omega, command.omega, at_switch.omega, alpha);
  const auto velocity_at = [&](double time) -> Velocity2D {
      if (time < delay) {return measured;}
      if (time <= switch_time) {
        return {approach(measured.v, command.v, a * (time - delay)),
        approach(measured.omega, command.omega, alpha * (time - delay))};
      }
      return {time >= linear_stop ? 0.0 : approach(at_switch.v, 0, a * (time - switch_time)),
      time >= angular_stop ? 0.0 : approach(at_switch.omega, 0, alpha * (time - switch_time))};
    };
  const std::array<double, 6> boundaries{delay, switch_time,
    delay + std::abs(command.v - measured.v) / a,
    delay + std::abs(command.omega - measured.omega) / alpha, linear_stop, angular_stop};
  MotionRollout result{{command, {initial}}, std::max(linear_stop, angular_stop)};
  Pose2D pose = initial;
  const long double horizon = config_.horizon, step = config_.step;
  const long double tolerance = 8.0L * std::numeric_limits<double>::epsilon() * horizon;
  long double elapsed = 0;
  for (std::size_t index = 1; elapsed < horizon; ++index) {
    long double next = std::min(index * step, horizon);
    if (horizon - next <= tolerance) {next = horizon;}
    while (elapsed < next) {
      long double end_time = next;
      for (double boundary : boundaries) {
        if (elapsed < boundary && boundary < end_time) {end_time = boundary;}
      }
      const double dt = static_cast<double>(end_time - elapsed);
      const auto begin = velocity_at(static_cast<double>(elapsed));
      const auto end = velocity_at(static_cast<double>(end_time));
      const double v = (begin.v + end.v) / 2.0, w = (begin.omega + end.omega) / 2.0;
      pose.x += v * std::cos(pose.theta + w * dt / 2.0) * dt;
      pose.y += v * std::sin(pose.theta + w * dt / 2.0) * dt;
      pose.theta += w * dt;
      elapsed = end_time;
    }
    result.trajectory.poses.push_back(pose);
  }
  return result;
}
}  // namespace my_dwa_controller
