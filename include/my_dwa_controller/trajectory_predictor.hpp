#ifndef MY_DWA_CONTROLLER__TRAJECTORY_PREDICTOR_HPP_
#define MY_DWA_CONTROLLER__TRAJECTORY_PREDICTOR_HPP_

#include "my_dwa_controller/types.hpp"

namespace my_dwa_controller
{

/// 在初始位姿所属坐标系中，以恒定速度执行前向 Euler 预测，不进行 TF 变换。
class TrajectoryPredictor
{
public:
  /// 时间参数单位为秒，必须为有限正数，否则抛出 std::invalid_argument。
  TrajectoryPredictor(double sim_time, double dt);

  /// 包含 t=0 和 t=sim_time 的位姿，最后一个时间间隔可能短于 dt。
  /// 角度单位为弧度，保留累计值；不存储坐标系或时间戳。
  /// 位姿或速度包含非有限分量时，抛出 std::invalid_argument。
  Trajectory predict(const Pose2D & initial_pose, const Velocity2D & cmd) const;

private:
  double sim_time_;
  double dt_;
};

}  // 命名空间 my_dwa_controller

#endif  // 头文件保护 MY_DWA_CONTROLLER__TRAJECTORY_PREDICTOR_HPP_
