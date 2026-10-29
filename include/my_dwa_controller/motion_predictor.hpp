#pragma once

#include "my_dwa_controller/types.hpp"

namespace my_dwa_controller
{
struct MotionConfig
{
  double linear_acceleration = 1.0;
  double angular_acceleration = 1.0;
  double response_delay = 0.12;
  double horizon = 1.7;
  double step = 0.05;
};

struct MotionRollout
{
  Trajectory trajectory;
  double stop_time = 0.0;
};

class MotionPredictor
{
public:
  explicit MotionPredictor(const MotionConfig & config);
  double step() const {return config_.step;}
  MotionRollout predict(
    const Pose2D & pose, const Velocity2D & measured,
    const Velocity2D & command, double execute_for) const;

private:
  MotionConfig config_;
};
}  // namespace my_dwa_controller
