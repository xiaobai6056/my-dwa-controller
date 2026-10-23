#ifndef MY_DWA_CONTROLLER__RISK_GOVERNOR_HPP_
#define MY_DWA_CONTROLLER__RISK_GOVERNOR_HPP_

#include <cstddef>
#include <optional>
#include "my_dwa_controller/path_conflict_monitor.hpp"

namespace my_dwa_controller
{

enum class RiskState {Normal, Limited, Braking, Wait, Recovering, DwaFallback};
const char * riskStateName(RiskState state);

struct RiskGovernorConfig
{
  double warning_speed = 0.12;
  double brake_acceleration = 0.3;
  double reaction_time = 0.3;
  double stop_margin = 0.1;
  std::size_t clear_states = 3;
  double robot_radius = 0.22;
  double control_dt = 0.05;
  double angular_acceleration = 3.2;  // Existing sampler limit; no new ROS parameter.
  double stopped_velocity = 0.02;
};

struct RiskDecision
{
  RiskState state = RiskState::Normal;
  double max_linear_speed = 0.0;
  // Soft preference only; original faster DWA candidates remain escape options.
  std::optional<double> preferred_max_linear_speed;
  double stopping_distance = 0.0;
  double available_distance = 0.0;
  bool allow_wait = false;
  bool cannot_stop = false;
  bool release_scan_risk = false;
};

/// A transient policy only: no command publication, scoring or parameter writes.
class RiskGovernor
{
public:
  explicit RiskGovernor(RiskGovernorConfig config = {});
  RiskDecision decide(
    const ConflictEvidence & evidence, const Velocity2D & velocity, double maximum_speed);
  void reset();

private:
  RiskGovernorConfig config_;
  RiskDecision previous_;
  std::size_t clear_count_ = 0;
};

}  // namespace my_dwa_controller
#endif  // MY_DWA_CONTROLLER__RISK_GOVERNOR_HPP_
