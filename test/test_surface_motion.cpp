#include "my_dwa_controller/surface_motion.hpp"
#include <cmath>
#include <fstream>
#include <gtest/gtest.h>


using namespace my_dwa_controller::surface_motion;

Points corner(double first_x = 0.0, double last_y = 1.0)
{
  Points points;
  for (double x = first_x; x <= 1.0001; x += 0.025) {
    points.push_back({x, 0.0});
  }
  for (double y = 0.025; y <= last_y + 0.0001; y += 0.025) {
    points.push_back({1.0, y});
  }
  return points;
}

Points translated(Points points, Point displacement)
{
  for (auto & point : points) {
    point[0] += displacement[0];
    point[1] += displacement[1];
  }
  return points;
}

Points rotated(Points points, double angle)
{
  for (auto & point : points) {
    const double x = point[0], y = point[1];
    point = {std::cos(angle) * x - std::sin(angle) * y, std::sin(angle) * x + std::cos(angle) * y};
  }
  return points;
}

TEST(Translation, RecoversTwoDimensionalMotion)
{
  const auto previous = corner();
  const auto result = estimate(previous, translated(previous, {-0.03, 0.05}), 0.2);
  ASSERT_TRUE(result.value);
  EXPECT_NEAR((*result.value)[0], -0.15, 1e-9);
  EXPECT_NEAR((*result.value)[1], 0.25, 1e-9);
}

TEST(Translation, StaticCroppingDoesNotCreateMotion)
{
  const auto result = estimate(corner(), corner(0.3, 0.7), 0.2);
  ASSERT_TRUE(result.value);
  EXPECT_NEAR((*result.value)[0], 0.0, 1e-9);
  EXPECT_NEAR((*result.value)[1], 0.0, 1e-9);
}

TEST(Translation, MotionAndCroppingCanCoexist)
{
  const auto result = estimate(corner(), translated(corner(0.3, 0.7), {0.04, -0.02}), 0.2);
  ASSERT_TRUE(result.value);
  EXPECT_NEAR((*result.value)[0], 0.2, 1e-9);
  EXPECT_NEAR((*result.value)[1], -0.1, 1e-9);
}

TEST(Translation, DoesNotAssumeAxisAlignedFaces)
{
  const auto previous = rotated(corner(), 0.63);
  const auto result = estimate(previous, translated(previous, {0.03, 0.04}), 0.2);
  ASSERT_TRUE(result.value);
  EXPECT_NEAR((*result.value)[0], 0.15, 1e-9);
  EXPECT_NEAR((*result.value)[1], 0.2, 1e-9);
}

TEST(Observability, ALineCannotEstablishFullVelocity)
{
  Points line;
  for (int i = 0; i < 41; ++i) {
    line.push_back({i *0.025, 0.0});
  }
  EXPECT_FALSE(estimate(line, translated(line, {0.05, 0.02}), 0.2).value);
}

TEST(Observability, AlmostParallelFacesRemainDegenerate)
{
  Points points;
  for (int i = 0; i < 41; ++i) {
    points.push_back({i *0.025, 0.0});
  }
  for (int i = 1; i < 41; ++i) {
    points.push_back({1.0 + i * 0.025 * std::cos(0.1), i * 0.025 * std::sin(0.1)});
  }
  EXPECT_FALSE(estimate(points, translated(points, {0.03, 0.04}), 0.2).value);
}

TEST(Model, RejectsInconsistentMotionWithoutDiscardingTheChangedSurface)
{
  auto previous = corner();
  for (int i = 1; i < 41; ++i) {
    previous.push_back({1 - i * 0.025, 1.0});
  }
  auto current = translated(corner(), {0.0, 0.05});
  for (int i = 1; i < 41; ++i) {
    current.push_back({1 - i * 0.025, 1.15});
  }
  EXPECT_FALSE(estimate(previous, current, 0.2).value);
}

TEST(Model, LargeObjectRotationDoesNotBecomeTranslation)
{
  EXPECT_FALSE(estimate(corner(), rotated(corner(), 0.3), 0.2).value);
}

TEST(Input, EmptyAndUnsupportedGroupsHaveNoVelocity)
{
  EXPECT_FALSE(estimate({}, corner(), 0.2).value);
  EXPECT_FALSE(estimate(corner(), {}, 0.2).value);
  EXPECT_FALSE(estimate({{0, 0}, {1, 0}, {1, 1}}, corner(), 0.2).value);
}

TEST(Input, InvalidOrUnassociatedIntervalsHaveNoVelocity)
{
  EXPECT_FALSE(estimate(corner(), corner(), 0.0).value);
  EXPECT_FALSE(estimate(corner(), corner(), -0.1).value);
  EXPECT_FALSE(estimate(corner(), corner(), 0.51).value);
}

Result recorded(const std::string & name)
{
  std::ifstream input(std::string(SURFACE_MOTION_FIXTURE_DIR) + "/" + name + ".txt");
  double dt;
  std::size_t before, after;
  input >> dt >> before >> after;
  Points previous(before), current(after);
  for (auto & p : previous) {
    input >> p[0] >> p[1];
  }
  for (auto & p : current) {
    input >> p[0] >> p[1];
  }
  if (!input) {throw std::runtime_error("invalid recorded surface fixture");}
  return estimate(previous, current, dt);
}

TEST(Recorded, CrossingSideSamplingDoesNotCreateLargeLateralVelocity)
{
  const auto result = recorded("crossing");
  ASSERT_TRUE(result.value);
  EXPECT_NEAR((*result.value)[0], 0.0, 0.05);
  EXPECT_NEAR((*result.value)[1], 0.25, 0.05);
}

TEST(Recorded, CroppedStaticCornerDoesNotApproachAtOneMeterPerSecond)
{
  const auto result = recorded("wall_crop");
  ASSERT_TRUE(result.value);
  EXPECT_LT(std::hypot((*result.value)[0], (*result.value)[1]), 0.05);
}

TEST(Recorded, SingleWallDoesNotReceiveAFullVelocity)
{
  EXPECT_FALSE(recorded("wall_aperture").value);
}

TEST(Recorded, StoppedSideDoesNotReceiveAFakeReturningVelocity)
{
  EXPECT_FALSE(recorded("stopped_aperture").value);
}
