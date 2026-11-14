#include "my_dwa_controller/pedestrian_collision.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace my_dwa_controller
{
namespace
{
constexpr double seconds = 1.e-9;

int64_t stamp(const builtin_interfaces::msg::Time & t)
{
  return t.sec >= 0 && t.nanosec < 1000000000u ?
         static_cast<int64_t>(t.sec) * 1000000000 + t.nanosec : -1;
}

bool finite(const tf2::Vector3 & v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

// Signed circle-to-polygon distance. Work in the inverse-transformed robot frame
// so the Nav2 footprint stays fixed for all samples.
double clearance(
  const std::vector<geometry_msgs::msg::Point> & polygon,
  const Pose2D & pose, const PedestrianPrediction & person, double t, double radius)
{
  const double dx = person.x + person.vx * t - pose.x;
  const double dy = person.y + person.vy * t - pose.y;
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  const double x = c * dx + s * dy, y = -s * dx + c * dy;
  bool inside = false;
  double closest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & a = polygon[j];
    const auto & b = polygon[i];
    if ((a.y > y) != (b.y > y) && x < a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y)) {
      inside = !inside;
    }
    const double ex = b.x - a.x, ey = b.y - a.y;
    const double len2 = ex * ex + ey * ey;
    const double u = len2 > 0. ?
      std::clamp(((x - a.x) * ex + (y - a.y) * ey) / len2, 0., 1.) : 0.;
    closest = std::min(closest, std::hypot(x - a.x - u * ex, y - a.y - u * ey));
  }
  return (inside ? -closest : closest) - radius;
}
}  // namespace

const char * velocitySourceName(VelocitySource source)
{
  switch (source) {
    case VelocitySource::Ols: return "ols";
    case VelocitySource::Difference: return "difference";
    case VelocitySource::Held: return "held";
    case VelocitySource::Kalman: return "kf";
    default: return "unknown";
  }
}

double PedestrianPrediction::radiusAt(double future) const
{
  return radius + std::min(max_growth, growth * (age + future));
}

PedestrianInput::PedestrianInput(
  PedestrianConfig config, std::string frame, std::shared_ptr<tf2_ros::Buffer> tf)
: config_(config), frame_(std::move(frame)), tf_(std::move(tf))
{
  for (double value : {config.max_position_age, config.max_velocity_age, config.retention,
      config.kf_max_cache_age, config.horizon, config.max_speed,
      config.min_difference_dt, config.max_difference_dt})
  {
    if (!std::isfinite(value) || value <= 0.) {
      throw std::invalid_argument("pedestrian timing and speed limits must be positive");
    }
  }
  for (double value : {config.position_buffer, config.ols_growth, config.difference_growth,
      config.held_growth, config.unknown_growth, config.max_growth, config.soft_growth_weight})
  {
    if (!std::isfinite(value) || value < 0.) {
      throw std::invalid_argument("pedestrian uncertainty must be nonnegative");
    }
  }
  if (frame_.empty() || config.min_difference_dt > config.max_difference_dt ||
    config.retention <
    std::max({config.max_position_age, config.max_velocity_age, config.kf_max_cache_age}))
  {
    throw std::invalid_argument("invalid pedestrian frame or timing bounds");
  }
}

void PedestrianInput::observeClock(int64_t now)
{
  if (now < last_now_) {
    history_.clear();
    counts_ = {};
    publication_ = scan_ = 0;
    epoch_ = 0;
  }
  last_now_ = now;
}

void PedestrianInput::receive(const msg::PedestrianStateArray & batch, int64_t now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  observeClock(now);
  const auto published = stamp(batch.header.stamp), scan = stamp(batch.scan_stamp);
  if (published <= 0 || published > now || scan < 0 || scan > published ||
    batch.header.frame_id.empty() || batch.header.frame_id.front() == '/' ||
    batch.epoch < epoch_ || (batch.epoch == epoch_ && published < publication_))
  {
    return;
  }
  if (batch.epoch != epoch_ || batch.status == "clock_reset") {
    history_.clear();
    scan_ = 0;
    epoch_ = batch.epoch;
  }
  publication_ = published;
  counts_ = {};
  counts_.received = batch.states.size();
  const bool finalized = batch.status != "scan_timeout" && batch.status != "clock_reset";
  const bool new_scan = finalized && scan > scan_;
  const auto previous_scan = scan_;
  if (new_scan) {scan_ = scan;}
  for (auto & [id, h] : history_) {
    h.currently_measured = false;
  }
  std::set<std::string> ids, duplicates, accepted;
  for (const auto & row : batch.states) {
    if (!ids.insert(row.track_id).second) {duplicates.insert(row.track_id);}
  }
  for (const auto & row : batch.states) {
    const auto observation = stamp(row.last_observation_stamp);
    const bool measured = new_scan && row.position_valid && observation == scan;
    // The fusion contract allows velocity_valid on a held row only if its
    // OLS endpoint equals last_observation_stamp.
    const bool held = !batch.kalman_estimate && finalized && !row.position_valid &&
      row.velocity_valid;
    if ((!measured && !held) || row.track_id.empty() || duplicates.count(row.track_id) ||
      observation <= 0 || observation > scan ||
      (now - observation) * seconds > config_.max_position_age ||
      !std::isfinite(row.occupancy_radius) || row.occupancy_radius <= 0.)
    {
      ++counts_.rejected;
      continue;
    }
    tf2::Vector3 position(row.position.x, row.position.y, row.position.z);
    tf2::Vector3 velocity(row.velocity.x, row.velocity.y, 0.);
    if (!finite(position)) {
      ++counts_.rejected;
      continue;
    }
    if (batch.header.frame_id != frame_) {
      try {
        if (!tf_) {++counts_.rejected; continue;}
        const auto transform = static_cast<const tf2::BufferCore &>(*tf_).lookupTransform(
          frame_, batch.header.frame_id, tf2::TimePoint(std::chrono::nanoseconds(observation)));
        tf2::Transform convert;
        tf2::fromMsg(transform.transform, convert);
        position = convert * position;
        velocity = convert.getBasis() * velocity;
      } catch (const tf2::TransformException &) {
        ++counts_.rejected;
        continue;
      }
    }
    if (!finite(position)) {
      ++counts_.rejected;
      continue;
    }
    auto & h = history_[row.track_id];
    if (observation < h.position) {++counts_.rejected; continue;}
    const bool valid_velocity = row.velocity_valid && finite(velocity) &&
      std::hypot(velocity.x(), velocity.y()) <= config_.max_speed;
    if (held && !valid_velocity) {++counts_.rejected; continue;}
    counts_.valid_position += measured;
    counts_.valid_velocity += valid_velocity;
    if (valid_velocity && observation >= h.velocity) {
      h.vx = velocity.x();
      h.vy = velocity.y();
      h.velocity = observation;
      h.source = batch.kalman_estimate ? VelocitySource::Kalman :
        held ? VelocitySource::Held : VelocitySource::Ols;
    } else if (batch.kalman_estimate) {
      h.velocity = 0;
      h.vx = h.vy = 0.;
      h.source = VelocitySource::Unknown;
    } else if (measured && h.observed_on_previous_scan && h.position == previous_scan) {
      const double dt = (observation - h.position) * seconds;
      const double dx = position.x() - h.x, dy = position.y() - h.y;
      if (dt >= config_.min_difference_dt && dt <= config_.max_difference_dt &&
        std::hypot(dx, dy) <= config_.max_speed * dt)
      {
        h.vx = dx / dt;
        h.vy = dy / dt;
        h.velocity = observation;
        h.source = VelocitySource::Difference;
      }
    }
    h.x = position.x();
    h.y = position.y();
    h.radius = row.occupancy_radius;
    h.position = observation;
    if (measured) {
      accepted.insert(row.track_id);
      h.currently_measured = true;
    }
  }
  if (new_scan) {
    for (auto & [id, h] : history_) {
      h.observed_on_previous_scan = accepted.count(id) != 0;
    }
  }
}

PedestrianSnapshot PedestrianInput::snapshot(int64_t now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  observeClock(now);
  PedestrianSnapshot result = counts_;
  for (auto it = history_.begin(); it != history_.end(); ) {
    const auto & [id, h] = *it;
    // A converged KF posterior carries position and velocity at the same source stamp.
    const bool converged_kf = h.source == VelocitySource::Kalman && h.velocity == h.position;
    const double max_position_age =
      converged_kf ? config_.kf_max_cache_age : config_.max_position_age;
    const double max_velocity_age =
      converged_kf ? config_.kf_max_cache_age : config_.max_velocity_age;
    const double age = static_cast<double>(now - h.position) / 1.e9;
    if (age > config_.retention || h.position <= 0) {
      it = history_.erase(it);
      continue;
    }
    ++it;
    if (age < 0. || age > max_position_age) {continue;}
    PedestrianPrediction p;
    p.id = id;
    p.epoch = epoch_;
    p.age = age;
    p.horizon = config_.horizon;
    p.physical_radius = h.radius;
    p.radius = h.radius + config_.position_buffer;
    p.max_growth = config_.max_growth;
    p.velocity_age = h.velocity ? static_cast<double>(now - h.velocity) / 1.e9 : -1.;
    p.growth = config_.unknown_growth;
    if (p.velocity_age >= 0. && p.velocity_age <= max_velocity_age) {
      p.vx = h.vx;
      p.vy = h.vy;
      p.source = h.velocity != h.position || !h.currently_measured ?
        VelocitySource::Held : h.source;
      p.growth =
        (p.source == VelocitySource::Ols ||
        p.source == VelocitySource::Kalman) ? config_.ols_growth :
        p.source == VelocitySource::Difference ? config_.difference_growth : config_.held_growth;
    }
    if (config_.soft_growth_enabled && p.source != VelocitySource::Unknown) {
      const double speed_scale = std::clamp((std::hypot(p.vx, p.vy) - .03) / (.12 - .03), 0., 1.);
      p.growth *= std::max(speed_scale, age > .5 ? .5 : 0.);
    }
    p.x = h.x + p.vx * age;
    p.y = h.y + p.vy * age;
    result.people.push_back(std::move(p));
  }
  return result;
}

PedestrianCollisionResult pedestrianCollision(
  const Trajectory & trajectory, double step, double horizon,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const std::vector<PedestrianPrediction> & people, bool soften_growth)
{
  PedestrianCollisionResult result;
  if (people.empty()) {return result;}
  if (footprint.size() < 3 || trajectory.poses.empty() || step <= 0. || horizon < 0.) {
    result.collision = true;
    result.first_time = 0.;
    return result;
  }
  double footprint_radius = 0.;
  for (const auto & p : footprint) {
    footprint_radius = std::max(footprint_radius, std::hypot(p.x, p.y));
  }
  constexpr double tolerance = .01;
  for (const auto & person : people) {
    const auto record = [&](double distance, double time, bool hit) {
        result.min_clearance = std::min(result.min_clearance, distance);
        if (hit && time < result.first_time) {
          result.collision = true;
          result.first_time = time;
          result.id = person.id;
        }
      };
    const auto evaluate = [&](const Pose2D & pose, double time) {
        const double envelope = person.radiusAt(time);
        const double radius = soften_growth ? person.physical_radius : envelope;
        const double hard_clearance = clearance(footprint, pose, person, time, radius);
        if (soften_growth) {
          result.soft_cost = std::max(result.soft_cost, envelope - radius - hard_clearance);
        }
        return hard_clearance;
      };
    double prev_clear = evaluate(trajectory.poses.front(), 0.);
    record(prev_clear, 0., prev_clear <= 0.);
    for (std::size_t i = 1; i < trajectory.poses.size(); ++i) {
      const double ta = std::min((i - 1) * step, horizon);
      const double tb = std::min(i * step, horizon);
      if (ta >= person.horizon || tb <= ta) {break;}
      const auto & a = trajectory.poses[i - 1];
      const auto & b = trajectory.poses[i];
      const double dx = b.x - a.x, dy = b.y - a.y, turn = b.theta - a.theta;
      const double relative = std::hypot(dx - person.vx * (tb - ta), dy - person.vy * (tb - ta)) +
        footprint_radius * std::abs(turn) + (soften_growth ? 0. : person.growth * (tb - ta));
      const double fraction = (std::min(tb, person.horizon) - ta) / (tb - ta);
      const auto at = [&](double u) {
          Pose2D p;
          p.x = a.x + u * dx;
          p.y = a.y + u * dy;
          p.theta = a.theta + u * turn;
          return evaluate(p, ta + u * (tb - ta));
        };
      const double end_clear = at(fraction);
      record(end_clear, ta + fraction * (tb - ta), end_clear <= 0.);
      // Distance is Lipschitz bounded by relative translation, footprint
      // rotation and radial growth. Subdivide only potentially close intervals.
      const auto inspect =
        [&](auto && self, double l, double r, double left, double right) -> void {
          if (std::min(left, right) > .5 * relative * (r - l)) {return;}
          if (left <= 0. || relative * (r - l) <= tolerance) {
            record(left, ta + l * (tb - ta), true);
            return;
          }
          const double m = .5 * (l + r);
          const double middle = at(m);
          record(middle, ta + m * (tb - ta), middle <= 0.);
          self(self, l, m, left, middle);
          self(self, m, r, middle, right);
        };
      inspect(inspect, 0., fraction, prev_clear, end_clear);
      prev_clear = end_clear;
    }
  }
  return result;
}
}  // namespace my_dwa_controller
