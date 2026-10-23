#include "my_dwa_controller/risk_governor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace my_dwa_controller
{
RiskGovernor::RiskGovernor(RiskGovernorConfig config)
: config_(config)
{
  for (double positive : {config.brake_acceleration, config.robot_radius,
      config.control_dt, config.stopped_velocity, config.angular_acceleration})
  {
    if (!std::isfinite(positive) || positive <= 0.0) {
      throw std::invalid_argument("risk braking/radius/period/stopped threshold must be positive");
    }
  }
  for (double nonnegative : {config.warning_speed, config.reaction_time, config.stop_margin}) {
    if (!std::isfinite(nonnegative) || nonnegative < 0.0) {
      throw std::invalid_argument("risk speed/delay/margin must be finite and nonnegative");
    }
  }
  if (config.clear_states == 0) {
    throw std::invalid_argument("risk clear_states must be positive");
  }
}

RiskDecision RiskGovernor::decide(
  const ConflictEvidence & e, const Velocity2D & velocity, double maximum)
{
  RiskDecision d;
  d.max_linear_speed = maximum;
  const bool local_area = e.scan_enabled && e.scan_area_active;
  const bool observed_clear = local_area ? e.scan_area_clear : e.corridor_known;
  const bool sensor_clear = !e.scan_enabled || (e.scan_fresh &&
    (local_area ? e.scan_area_clear : !e.scan_unresolved));
  const bool recovery_aligned = local_area || e.path_aligned;
  const bool new_clear_state = e.scan_enabled ? e.scan_new_state : e.new_map_state;
  const double v = std::max(0.0, velocity.v);
  const double decrement = config_.brake_acceleration * config_.control_dt;
  const bool stopped = v <= std::min(config_.stopped_velocity, decrement) &&
    std::abs(velocity.omega) <= std::min(0.05, config_.angular_acceleration * config_.control_dt);
  d.stopping_distance = v * (config_.reaction_time + config_.control_dt) +
    v * v / (2.0 * config_.brake_acceleration);
  if (e.inner_conflict) {
    clear_count_ = 0;
    d.available_distance = e.inner.start - config_.robot_radius - config_.stop_margin;
    // Geometric clearance is meaningful only while on the observed forward path.
    // Unknown Inner cells at/before the conflict forbid parking. Unknown far-end
    // or Outer cells do not invalidate this observed prefix; recovery is stricter.
    const double reverse_tolerance = e.scan_enabled ?
      std::min(config_.stopped_velocity, decrement) : 0.0;
    if (e.known_inner_distance > e.inner.start && e.path_aligned &&
      velocity.v >= -reverse_tolerance &&
      d.available_distance >= d.stopping_distance)
    {
      d.state = stopped ? RiskState::Wait : RiskState::Braking;
      d.allow_wait = stopped;
      d.max_linear_speed = stopped ? 0.0 : std::min(maximum, std::max(0.0, v - decrement));
    } else {
      // Cannot certify a stop before the region; let collision-checked DWA evade.
      d.state = RiskState::DwaFallback;
      d.cannot_stop = true;
    }
  } else {
    const bool latched = previous_.state == RiskState::Braking ||
      previous_.state == RiskState::Wait || previous_.state == RiskState::Limited ||
      previous_.state == RiskState::DwaFallback;
    if (e.outer_risk || !observed_clear || !recovery_aligned ||
      e.map_state == ConflictMapState::First || !sensor_clear)
    {
      clear_count_ = 0;
    } else if (new_clear_state) {
      clear_count_ = std::min(config_.clear_states, clear_count_ + 1);
    }
    if (e.outer_risk && previous_.state == RiskState::DwaFallback) {
      d.state = RiskState::Limited;
      d.max_linear_speed = std::min(maximum, config_.warning_speed);
    } else if (latched && clear_count_ < config_.clear_states) {
      d.state = previous_.state;
      d.allow_wait = previous_.allow_wait && stopped;
      if (d.state == RiskState::Wait && !d.allow_wait) {d.state = RiskState::Braking;}
      if (d.state == RiskState::Limited) {
        d.max_linear_speed = std::min(maximum, config_.warning_speed);
      } else if (d.state != RiskState::DwaFallback) {
        d.max_linear_speed = std::min(maximum, std::max(0.0, v - decrement));
      }
      // Pending clear confirmation is not a certified stop. Keep DWA available
      // during fallback instead of treating missing/misaligned evidence as Normal.
    } else if (e.outer_risk) {
      d.state = RiskState::Limited;
      d.max_linear_speed = std::min(maximum, config_.warning_speed);
    } else if (latched || previous_.state == RiskState::Recovering) {
      const bool can_recover = observed_clear && recovery_aligned &&
        e.map_state != ConflictMapState::First && sensor_clear;
      d.max_linear_speed = std::min(
        maximum, previous_.max_linear_speed + (can_recover ? decrement : 0.0));
      d.state = d.max_linear_speed < maximum ? RiskState::Recovering : RiskState::Normal;
    }
  }
  if (e.scan_enabled && d.state == RiskState::DwaFallback &&
    previous_.state != RiskState::Normal)
  {
    // Losing stop certification is not clearance. Preserve the established cap;
    // Controller still keeps the reachable deceleration boundary and DWA scoring.
    d.max_linear_speed = std::min(d.max_linear_speed, previous_.max_linear_speed);
  }
  if (d.state == RiskState::DwaFallback) {
    d.preferred_max_linear_speed = std::min(maximum, v);
  }
  d.release_scan_risk = local_area && d.state == RiskState::Normal &&
    observed_clear && sensor_clear && clear_count_ >= config_.clear_states;
  previous_ = d;
  return d;
}

void RiskGovernor::reset()
{
  previous_ = {};
  clear_count_ = 0;
}

const char * riskStateName(RiskState state)
{
  switch (state) {
    case RiskState::Normal: return "normal";
    case RiskState::Limited: return "limited";
    case RiskState::Braking: return "braking";
    case RiskState::Wait: return "wait";
    case RiskState::Recovering: return "recovering";
    case RiskState::DwaFallback: return "dwa_fallback";
  }
  return "normal";
}
}  // namespace my_dwa_controller
