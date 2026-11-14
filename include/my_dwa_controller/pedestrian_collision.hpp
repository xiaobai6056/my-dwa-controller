#pragma once

#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "my_dwa_controller/msg/pedestrian_state_array.hpp"
#include "my_dwa_controller/types.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/buffer.h"

namespace my_dwa_controller
{
enum class VelocitySource {Unknown, Ols, Difference, Held, Kalman};

const char * velocitySourceName(VelocitySource source);

struct PedestrianConfig
{
  double max_position_age = .5;
  double max_velocity_age = .5;
  double retention = 5.;
  double kf_max_cache_age = .85;
  double horizon = 1.7;
  double max_speed = 2.;
  double min_difference_dt = .05;
  double max_difference_dt = .30;
  double position_buffer = .06;
  double ols_growth = .08;
  double difference_growth = .20;
  double held_growth = .24;
  double unknown_growth = .35;
  double max_growth = .25;
  bool soft_growth_enabled = false;
  double soft_growth_weight = .30;
};

struct PedestrianPrediction
{
  std::string id;
  uint32_t epoch = 0;
  // Center at the cycle time. Future offsets are relative to this same time.
  double x = 0., y = 0., vx = 0., vy = 0.;
  double radius = 0., age = 0., growth = 0., max_growth = 0., horizon = 0.;
  double physical_radius = 0.;
  double velocity_age = -1.;
  VelocitySource source = VelocitySource::Unknown;
  double radiusAt(double future) const;
};

struct PedestrianSnapshot
{
  std::vector<PedestrianPrediction> people;
  std::size_t received = 0, valid_position = 0, valid_velocity = 0, rejected = 0;
};

class PedestrianInput
{
public:
  PedestrianInput(
    PedestrianConfig config, std::string frame, std::shared_ptr<tf2_ros::Buffer> tf);
  void receive(const msg::PedestrianStateArray & batch, int64_t now);
  PedestrianSnapshot snapshot(int64_t now);

private:
  struct History
  {
    double x = 0., y = 0., radius = 0., vx = 0., vy = 0.;
    int64_t position = 0, velocity = 0;
    bool observed_on_previous_scan = false, currently_measured = false;
    VelocitySource source = VelocitySource::Unknown;
  };
  void observeClock(int64_t now);
  PedestrianConfig config_;
  std::string frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::mutex mutex_;
  std::map<std::string, History> history_;
  PedestrianSnapshot counts_;
  int64_t last_now_ = 0, publication_ = 0, scan_ = 0;
  uint32_t epoch_ = 0;
};

struct PedestrianCollisionResult
{
  bool collision = false;
  double first_time = std::numeric_limits<double>::infinity();
  double min_clearance = std::numeric_limits<double>::infinity();
  std::string id;
  double soft_cost = 0.;
};

PedestrianCollisionResult pedestrianCollision(
  const Trajectory & trajectory, double step, double horizon,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const std::vector<PedestrianPrediction> & people, bool soften_growth = false);
}  // namespace my_dwa_controller
