#include "my_dwa_controller/surface_motion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace my_dwa_controller::surface_motion
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

Result estimate(const Points & previous, const Points & current, double dt, const Config & config)
{
  Result result;
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.5) {
    result.reason = "invalid_interval";
    return result;
  }
  std::vector<Segment> before, after;
  split(previous, 0, static_cast<int>(previous.size()), config, before);
  split(current, 0, static_cast<int>(current.size()), config, after);
  std::vector<Constraint> constraints;
  std::vector<bool> supported(current.size(), false);
  for (const auto & surface : after) {
    auto matches = matchSurface(surface, current, before, config);
    if (matches.empty()) {continue;}
    ++result.surfaces;
    for (const auto & match : matches) {
      supported[match.index] = true;
    }
    constraints.insert(constraints.end(), matches.begin(), matches.end());
  }
  result.support = std::count(supported.begin(), supported.end(), true);
  if (constraints.empty() || result.support < config.support_fraction * current.size()) {
    return result;
  }
  auto & h = result.normal_matrix;
  auto & b = result.right_hand_side;
  for (const auto & c : constraints) {
    h[0] += c.weight * c.normal[0] * c.normal[0];
    h[1] += c.weight * c.normal[0] * c.normal[1];
    h[2] += c.weight * c.normal[1] * c.normal[1];
    b[0] += c.weight * c.normal[0] * c.distance;
    b[1] += c.weight * c.normal[1] * c.distance;
  }
  const double trace = h[0] + h[2], gap = std::hypot(h[0] - h[2], 2.0 * h[1]);
  const double maximum = (trace + gap) / 2.0, minimum = (trace - gap) / 2.0;
  result.condition = minimum > 0.0 ? maximum / minimum : std::numeric_limits<double>::infinity();
  if (result.condition > config.max_condition) {
    result.reason = "degenerate";
    return result;
  }
  const double determinant = h[0] * h[2] - h[1] * h[1];
  const Point displacement{(h[2] * b[0] - h[1] * b[1]) / determinant,
    (h[0] * b[1] - h[1] * b[0]) / determinant};
  double sum = 0.0, weight = 0.0;
  for (const auto & c : constraints) {
    const double error = dot(c.normal, displacement) - c.distance;
    sum += c.weight * error * error;
    weight += c.weight;
    result.max_error = std::max(result.max_error, std::abs(error));
  }
  result.rms = std::sqrt(sum / weight);
  if (result.rms > config.fit_error || result.max_error > 2.0 * config.fit_error) {
    result.reason = "inconsistent";
    return result;
  }
  result.value = Point{displacement[0] / dt, displacement[1] / dt};
  result.reason = "estimated";
  return result;
}
}  // namespace my_dwa_controller::surface_motion
