#pragma once

#include <array>
#include <vector>

namespace velocity
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

}  // namespace velocity
