#pragma once

#include <array>
#include <optional>
#include <vector>

namespace my_dwa_controller::surface_motion
{
using Point = std::array<double, 2>;
using Points = std::vector<Point>;

struct Config
{
  int min_line_points = 6;
  double min_line_length = 0.15;
  double line_error = 0.03;
  double association_distance = 0.35;
  double direction_degrees = 10.0;
  double support_fraction = 0.5;
  double fit_error = 0.03;
  double max_condition = 20.0;
};

struct Result
{
  std::optional<Point> value;
  const char * reason = "insufficient_overlap";
  int support = 0;
  int surfaces = 0;
  double rms = 0.0;
  double max_error = 0.0;
  double condition = 0.0;
  std::array<double, 3> normal_matrix{};  // xx, xy, yy; diagnostic, not matching features
  Point right_hand_side{};
};

// Inputs are original ordered, finite points from a single associated cluster
// in the same odom frame. Output describes past average translation only.
Result estimate(
  const Points & previous, const Points & current, double dt,
  const Config & config = {});
}  // namespace my_dwa_controller::surface_motion
