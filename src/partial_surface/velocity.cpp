#include "velocity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace velocity
{
namespace
{
double dot(Point a, Point b) {return a[0] * b[0] + a[1] * b[1];}
Point subtract(Point a, Point b) {return {a[0] - b[0], a[1] - b[1]};}

struct Segment
{
  int begin = 0, end = 0;
  Point center{}, direction{}, start{};
  double length = 0.0, error = 0.0;
};

// Same ordered TLS/chord split geometry as the existing local-line experiment.
Segment fit(const Points & points, int begin, int end)
{
  Segment line;
  line.begin = begin;
  line.end = end;
  for (int i = begin; i < end; ++i) {
    line.center[0] += points[i][0];
    line.center[1] += points[i][1];
  }
  line.center[0] /= end - begin;
  line.center[1] /= end - begin;
  double xx = 0.0, xy = 0.0, yy = 0.0;
  for (int i = begin; i < end; ++i) {
    const auto p = subtract(points[i], line.center);
    xx += p[0] * p[0];
    xy += p[0] * p[1];
    yy += p[1] * p[1];
  }
  const double angle = 0.5 * std::atan2(2.0 * xy, xx - yy);
  line.direction = {std::cos(angle), std::sin(angle)};
  double low = std::numeric_limits<double>::infinity(), high = -low;
  const Point normal{-line.direction[1], line.direction[0]};
  for (int i = begin; i < end; ++i) {
    const auto p = subtract(points[i], line.center);
    low = std::min(low, dot(p, line.direction));
    high = std::max(high, dot(p, line.direction));
    line.error = std::max(line.error, std::abs(dot(p, normal)));
  }
  line.length = high - low;
  line.start = {line.center[0] + low * line.direction[0], line.center[1] + low * line.direction[1]};
  return line;
}

void split(
  const Points & points, int begin, int end, const Config & config,
  std::vector<Segment> & segments)
{
  if (end - begin < config.min_line_points) {return;}
  const auto line = fit(points, begin, end);
  if (line.error <= config.line_error) {
    if (line.length >= config.min_line_length) {segments.push_back(line);}
    return;
  }
  const auto chord = subtract(points[end - 1], points[begin]);
  const double length = std::hypot(chord[0], chord[1]);
  double largest = -1.0;
  int corner = begin + 1;
  for (int i = begin + 1; i < end - 1; ++i) {
    const auto p = subtract(points[i], points[begin]);
    const double error = length > 0.0 ? std::abs(p[0] * chord[1] - p[1] * chord[0]) / length :
      std::hypot(p[0], p[1]);
    if (error > largest) {largest = error; corner = i;}
  }
  split(points, begin, corner + 1, config, segments);
  split(points, corner, end, config, segments);
}

struct Constraint
{
  Point normal;
  double distance, weight;
  int index;
};

std::vector<Constraint> matchSurface(
  const Segment & current, const Points & points,
  const std::vector<Segment> & previous, const Config & config)
{
  std::vector<Constraint> best;
  double best_cost = std::numeric_limits<double>::infinity();
  const double cosine = std::cos(config.direction_degrees * std::acos(-1.0) / 180.0);
  for (const auto & line : previous) {
    if (std::abs(dot(current.direction, line.direction)) < cosine) {continue;}
    const Point normal{-line.direction[1], line.direction[0]};
    std::vector<Constraint> candidate;
    double squared_distance = 0.0;
    for (int i = current.begin; i < current.end; ++i) {
      const double projection = dot(subtract(points[i], line.start), line.direction);
      const double distance = dot(subtract(points[i], line.center), normal);
      // Endpoints define support, never a material-point displacement measurement.
      if (projection < 0.0 || projection > line.length ||
        std::abs(distance) > config.association_distance) {continue;}
      candidate.push_back({normal, distance, 0.0, i});
      squared_distance += distance * distance;
    }
    if (candidate.size() < static_cast<std::size_t>(config.min_line_points)) {continue;}
    const double cost = squared_distance / candidate.size();
    if (cost < best_cost) {best = std::move(candidate); best_cost = cost;}
  }
  // Each surface has total weight one, independent of how many beams hit it.
  for (auto & constraint : best) {
    constraint.weight = 1.0 / best.size();
  }
  return best;
}
}  // namespace

}  // namespace velocity

// Ordered surface geometry remains local to this measurement implementation.
// The original surface_motion estimator remains separate; do not merge the two
// computations without a dedicated numerical and runtime-cost validation.
#include "measurement.hpp"
#include <Eigen/Dense>

namespace partial_surface
{
Measurement measure(
  const velocity::Points & previous, const velocity::Points & current, double dt,
  const velocity::Config & config)
{
  Measurement result;
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.5) {
    result.reason = "invalid_interval";
    return result;
  }
  std::vector<velocity::Segment> before, after;
  velocity::split(previous, 0, static_cast<int>(previous.size()), config, before);
  velocity::split(current, 0, static_cast<int>(current.size()), config, after);
  std::vector<velocity::Constraint> constraints;
  std::vector<bool> supported(current.size(), false);
  for (const auto & surface : after) {
    auto matches = velocity::matchSurface(surface, current, before, config);
    for (const auto & match : matches) {
      supported[match.index] = true;
    }
    constraints.insert(constraints.end(), matches.begin(), matches.end());
  }
  result.support = std::count(supported.begin(), supported.end(), true);
  if (constraints.empty() || result.support < config.support_fraction * current.size()) {
    return result;
  }
  Eigen::Matrix2d h = Eigen::Matrix2d::Zero();
  Eigen::Vector2d b = Eigen::Vector2d::Zero();
  for (const auto & constraint : constraints) {
    const Eigen::Vector2d normal(constraint.normal[0], constraint.normal[1]);
    h += constraint.weight * normal * normal.transpose();
    b += constraint.weight * normal * constraint.distance;
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(h);
  const double largest = eigen.eigenvalues()[1];
  Eigen::Matrix2d observed_h = Eigen::Matrix2d::Zero();
  Eigen::Vector2d observed_b = Eigen::Vector2d::Zero();
  Eigen::Vector2d displacement = Eigen::Vector2d::Zero();
  int rank = 0;
  for (int mode = 0; mode < 2; ++mode) {
    const double information = eigen.eigenvalues()[mode];
    if (information <= 0.0 || information < largest / config.max_condition) {continue;}
    const Eigen::Vector2d direction = eigen.eigenvectors().col(mode);
    observed_h += information * direction * direction.transpose();
    observed_b += direction * direction.dot(b);
    displacement += direction * (direction.dot(b) / information);
    ++rank;
  }
  double squared_error = 0.0, total_weight = 0.0;
  for (const auto & constraint : constraints) {
    const double error = constraint.normal[0] * displacement[0] +
      constraint.normal[1] * displacement[1] - constraint.distance;
    squared_error += constraint.weight * error * error;
    total_weight += constraint.weight;
    result.max_error = std::max(result.max_error, std::abs(error));
  }
  result.rms = std::sqrt(squared_error / total_weight);
  if (result.rms > config.fit_error || result.max_error > 2.0 * config.fit_error) {
    result.reason = "inconsistent";
    return result;
  }
  result.information = {observed_h(0, 0), observed_h(0, 1), observed_h(1, 1)};
  result.right_hand_side = {observed_b[0] / dt, observed_b[1] / dt};
  result.rank = rank;
  result.reason = rank == 2 ? "full" : "normal_only";
  return result;
}
}  // namespace partial_surface
