#ifndef MY_DWA_CONTROLLER__TYPES_HPP_
#define MY_DWA_CONTROLLER__TYPES_HPP_

#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"

namespace my_dwa_controller
{
using Pose2D = geometry_msgs::msg::Pose2D;

struct Velocity2D
{
  double v;  // 线速度，单位 m/s；负值表示倒车。
  double omega;  // 角速度，单位 rad/s；正值表示左转。
};

struct Trajectory
{
  Velocity2D command;
  std::vector<Pose2D> poses;
};

}  // 命名空间 my_dwa_controller

#endif  // 头文件保护 MY_DWA_CONTROLLER__TYPES_HPP_
