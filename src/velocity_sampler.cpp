#include "my_dwa_controller/velocity_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace my_dwa_controller
{
namespace
{

void validateLimit(double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
  }
}

std::vector<double> sampleInterval(double lower, double upper, std::size_t count)
{
  if (lower == upper) {
    return {lower};
  }

  std::vector<double> values;
  values.reserve(count);
  values.push_back(lower);
  for (std::size_t i = 1; i < count - 1; ++i) {
    const double fraction = static_cast<double>(i) / static_cast<double>(count - 1);
    // 使用凸组合插值，避免较大正负边界相减时溢出。
    const double value = (1.0 - fraction) * lower + fraction * upper;
    values.push_back(std::max(lower, std::min(upper, value)));
  }
  values.push_back(upper);

  if (lower <= 0.0 && upper >= 0.0) {
    values.push_back(0.0);
  }
  std::sort(values.begin(), values.end());
  // 按精确相等去重，保留极窄有效窗口内仍可区分的采样值。
  values.erase(std::unique(values.begin(), values.end()), values.end());
  return values;
}

}  // 匿名命名空间

std::vector<Velocity2D> sampleVelocities(
  const Velocity2D & current, const SamplingConfig & config)
{
  if (!std::isfinite(current.v) || !std::isfinite(current.omega)) {
    throw std::invalid_argument("current velocity components must be finite");
  }
  validateLimit(config.max_v, "max_v");
  validateLimit(config.max_w, "max_w");
  validateLimit(config.acc_v, "acc_v");
  validateLimit(config.acc_w, "acc_w");
  if (!std::isfinite(config.control_dt) || config.control_dt <= 0.0) {
    throw std::invalid_argument("control_dt must be finite and positive (seconds)");
  }
  if (config.v_samples < 2 || config.w_samples < 2) {
    throw std::invalid_argument("v_samples and w_samples must both be at least 2");
  }

  const double delta_v = config.acc_v * config.control_dt;
  const double delta_w = config.acc_w * config.control_dt;
  const double v_min = std::max(0.0, current.v - delta_v);
  const double v_max = std::min(config.max_v, current.v + delta_v);
  const double w_min = std::max(-config.max_w, current.omega - delta_w);
  const double w_max = std::min(config.max_w, current.omega + delta_w);
  if (v_min > v_max || w_min > w_max) {
    return {};
  }

  const auto linear_samples = sampleInterval(v_min, v_max, config.v_samples);
  const auto angular_samples = sampleInterval(w_min, w_max, config.w_samples);
  std::vector<Velocity2D> velocities;
  for (const double v : linear_samples) {
    for (const double w : angular_samples) {
      velocities.push_back({v, w});
    }
  }
  return velocities;
}

}  // 命名空间 my_dwa_controller
