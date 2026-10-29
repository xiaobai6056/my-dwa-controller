#pragma once

#include "velocity.hpp"


namespace partial_surface
{
using velocity::Point;
using Matrix = std::array<double, 3>;  // xx, xy, yy

// Observable surface-normal constraints, not a tracked or filtered velocity.
// rank 0: unusable, rank 1: one observable direction, rank 2: two directions.
// Unobserved components are absent constraints, never evidence of zero motion.
struct Measurement
{
  Matrix information{};
  Point right_hand_side{};  // normal speed, not displacement
  int rank = 0;
  int support = 0;
  double rms = 0.0;
  double max_error = 0.0;
  const char * reason = "insufficient_overlap";
};

Measurement measure(
  const velocity::Points & previous, const velocity::Points & current, double dt,
  const velocity::Config & config = {});
}  // namespace partial_surface
