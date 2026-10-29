#pragma once

#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/time.hpp"

// Two intersecting faces sampled by a real polar beam grid. Unlike three nearby
// returns, this fixture constrains both components of surface translation.
inline sensor_msgs::msg::LaserScan cornerScan(
  int64_t stamp = 1000000000LL, const std::string & frame = "scan_world",
  double x = 1.0, double y = 1.0)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = rclcpp::Time(stamp);
  scan.header.frame_id = frame;
  scan.angle_min = std::atan2(y, x) - 0.7;
  scan.angle_increment = 0.01;
  scan.range_min = 0.12;
  scan.range_max = 3.5;
  for (int i = 0; i <= 140; ++i) {
    const double angle = scan.angle_min + i * scan.angle_increment;
    const double a = x / std::cos(angle), b = y / std::sin(angle);
    scan.ranges.push_back(a > 0.0 && b > 0.0 ? std::min(a, b) : std::max(a, b));
  }
  return scan;
}

inline sensor_msgs::msg::LaserScan uScan(int64_t stamp, double half_height)
{
  auto scan = cornerScan(stamp);
  scan.angle_min = -1.1;
  scan.ranges.clear();
  for (int i = 0; i <= 220; ++i) {
    const double angle = scan.angle_min + i * scan.angle_increment;
    scan.ranges.push_back(
      std::min(
        1.0 / std::cos(angle),
        half_height / std::abs(std::sin(angle))));
  }
  return scan;
}
