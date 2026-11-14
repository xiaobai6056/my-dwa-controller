#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "my_dwa_controller/velocity_sampler.hpp"

namespace my_dwa_controller
{
namespace
{
constexpr double kTolerance = 1e-12;

void expectGrid(
  const std::vector<Velocity2D> & actual,
  const std::vector<double> & expected_v, const std::vector<double> & expected_w)
{
  ASSERT_EQ(actual.size(), expected_v.size() * expected_w.size());
  std::size_t index = 0;
  for (const double v : expected_v) {
    for (const double w : expected_w) {
      SCOPED_TRACE(::testing::Message() << "candidate=" << index);
      EXPECT_NEAR(actual[index].v, v, kTolerance);
      EXPECT_NEAR(actual[index].omega, w, kTolerance);
      ++index;
    }
  }
}

TEST(VelocitySampler, DefaultConfigMatchesPublicContract)
{
  const SamplingConfig config;
  EXPECT_DOUBLE_EQ(config.max_v, 0.22);
  EXPECT_DOUBLE_EQ(config.max_w, 1.0);
  EXPECT_DOUBLE_EQ(config.acc_v, 0.5);
  EXPECT_DOUBLE_EQ(config.acc_w, 2.0);
  EXPECT_DOUBLE_EQ(config.control_dt, 0.05);
  EXPECT_EQ(config.v_samples, 5u);
  EXPECT_EQ(config.w_samples, 11u);
}

TEST(VelocitySampler, SamplesDefaultWindowFromRest)
{
  const auto velocities = sampleVelocities({0.0, 0.0}, SamplingConfig{});
  expectGrid(
    velocities, {0.0, 0.00625, 0.0125, 0.01875, 0.025},
    {-0.1, -0.08, -0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06, 0.08, 0.1});
}

TEST(VelocitySampler, NonzeroCurrentMovesBothWindows)
{
  SamplingConfig config;
  config.v_samples = 3;
  config.w_samples = 3;
  expectGrid(sampleVelocities({0.1, 0.4}, config), {0.075, 0.1, 0.125}, {0.3, 0.4, 0.5});
  expectGrid(sampleVelocities({0.1, -0.4}, config), {0.075, 0.1, 0.125}, {-0.5, -0.4, -0.3});
}

TEST(VelocitySampler, ClipsWindowsAtPositiveSpeedLimits)
{
  SamplingConfig config;
  config.v_samples = 3;
  config.w_samples = 3;
  const auto velocities = sampleVelocities({0.21, 0.95}, config);
  expectGrid(velocities, {0.185, 0.2025, 0.22}, {0.85, 0.925, 1.0});
  for (const auto & velocity : velocities) {
    EXPECT_LE(velocity.v, config.max_v);
    EXPECT_LE(velocity.omega, config.max_w);
  }
}

TEST(VelocitySampler, ClipsWindowAtNegativeAngularLimit)
{
  SamplingConfig config;
  config.v_samples = 2;
  config.w_samples = 3;
  const auto velocities = sampleVelocities({0.1, -0.95}, config);
  expectGrid(velocities, {0.075, 0.125}, {-1.0, -0.925, -0.85});
  for (const auto & velocity : velocities) {
    EXPECT_GE(velocity.omega, -config.max_w);
  }
}

TEST(VelocitySampler, EveryCandidateRespectsDynamicWindowAndAcceleration)
{
  const auto velocities = sampleVelocities({0.11, -0.35}, SamplingConfig{});
  ASSERT_FALSE(velocities.empty());
  // 独立计算得到窗口：v=[0.085,0.135]，omega=[-0.45,-0.25]。
  for (const auto & velocity : velocities) {
    EXPECT_GE(velocity.v, 0.085 - kTolerance);
    EXPECT_LE(velocity.v, 0.135 + kTolerance);
    EXPECT_GE(velocity.omega, -0.45 - kTolerance);
    EXPECT_LE(velocity.omega, -0.25 + kTolerance);
    EXPECT_LE(std::abs(velocity.v - 0.11), 0.025 + kTolerance);
    EXPECT_LE(std::abs(velocity.omega + 0.35), 0.1 + kTolerance);
  }
}

TEST(VelocitySampler, AddsReachableZeroToAsymmetricAngularGrid)
{
  SamplingConfig config;
  config.v_samples = 2;
  config.w_samples = 4;
  const auto velocities = sampleVelocities({0.01, 0.03}, config);
  // 基础网格有 2*4 个点；补零为每个线速度增加一个角速度样本。
  expectGrid(velocities, {0.0, 0.035}, {-0.07, -1.0 / 300.0, 0.0, 19.0 / 300.0, 0.13});
  bool contains_stop = false;
  for (const auto & velocity : velocities) {
    contains_stop = contains_stop || (velocity.v == 0.0 && velocity.omega == 0.0);
  }
  EXPECT_TRUE(contains_stop);
}

TEST(VelocitySampler, DoesNotForceUnreachableZero)
{
  SamplingConfig config;
  config.v_samples = 2;
  config.w_samples = 2;
  const auto velocities = sampleVelocities({0.03, 0.11}, config);
  expectGrid(velocities, {0.005, 0.055}, {0.01, 0.21});
  for (const auto & velocity : velocities) {
    EXPECT_GT(velocity.v, 0.0);
    EXPECT_GT(velocity.omega, 0.0);
  }
}

TEST(VelocitySampler, AddsAngularZeroWithoutForcingLinearZero)
{
  SamplingConfig config;
  config.v_samples = 2;
  config.w_samples = 2;
  const auto velocities = sampleVelocities({0.1, 0.0}, config);
  expectGrid(velocities, {0.075, 0.125}, {-0.1, 0.0, 0.1});
  ASSERT_EQ(velocities.size(), 6u);
  EXPECT_DOUBLE_EQ(velocities[1].omega, 0.0);
  EXPECT_DOUBLE_EQ(velocities[4].omega, 0.0);
}

TEST(VelocitySampler, IncludesLinearZeroWithoutForcingAngularZero)
{
  SamplingConfig config;
  config.v_samples = 2;
  config.w_samples = 2;
  const auto velocities = sampleVelocities({0.01, 0.4}, config);
  expectGrid(velocities, {0.0, 0.035}, {0.3, 0.5});
}

TEST(VelocitySampler, IncludesZeroExactlyOnWindowBoundary)
{
  SamplingConfig config;
  config.v_samples = 2;
  config.w_samples = 2;
  const auto velocities = sampleVelocities({0.025, 0.1}, config);
  expectGrid(velocities, {0.0, 0.05}, {0.0, 0.2});
  ASSERT_EQ(velocities.size(), 4u);
  EXPECT_DOUBLE_EQ(velocities.front().v, 0.0);
  EXPECT_DOUBLE_EQ(velocities.front().omega, 0.0);
}

TEST(VelocitySampler, EmptyIntersectionReturnsEmptyWithoutRelaxingLimits)
{
  const std::array<Velocity2D, 4> currents{{
    {0.3, 0.0}, {-0.1, 0.0}, {0.1, 1.2}, {0.1, -1.2}}};
  for (const auto & current : currents) {
    SCOPED_TRACE(::testing::Message() << "current=" << current.v << "," << current.omega);
    EXPECT_TRUE(sampleVelocities(current, SamplingConfig{}).empty());
  }
}

TEST(VelocitySampler, CurrentOutsideLimitsCanStillHaveReachableWindow)
{
  SamplingConfig config;
  config.v_samples = 2;
  config.w_samples = 2;
  expectGrid(sampleVelocities({0.23, 1.05}, config), {0.205, 0.22}, {0.95, 1.0});
  expectGrid(sampleVelocities({-0.01, 0.2}, config), {0.0, 0.015}, {0.1, 0.3});
}

TEST(VelocitySampler, ZeroAccelerationCollapsesBothAxesToOneCandidate)
{
  SamplingConfig config;
  config.acc_v = 0.0;
  config.acc_w = 0.0;
  expectGrid(sampleVelocities({0.12, -0.35}, config), {0.12}, {-0.35});
  expectGrid(sampleVelocities({0.0, 0.0}, config), {0.0}, {0.0});
  EXPECT_TRUE(sampleVelocities({0.3, 0.0}, config).empty());
}

TEST(VelocitySampler, ZeroSpeedLimitsCollapseEachAxisWithoutDuplicates)
{
  SamplingConfig config;
  config.max_v = 0.0;
  config.w_samples = 2;
  expectGrid(sampleVelocities({0.0, 0.0}, config), {0.0}, {-0.1, 0.0, 0.1});
  config.max_v = 0.22;
  config.max_w = 0.0;
  config.v_samples = 2;
  expectGrid(sampleVelocities({0.0, 0.0}, config), {0.0, 0.025}, {0.0});
  config.max_v = 0.0;
  expectGrid(sampleVelocities({0.0, 0.0}, config), {0.0}, {0.0});
}

TEST(VelocitySampler, KeepsDistinctSamplesInVeryNarrowWindow)
{
  SamplingConfig config;
  config.acc_v = 1e-14;
  config.control_dt = 1.0;
  config.max_w = 0.0;
  config.v_samples = 3;
  const auto velocities = sampleVelocities({0.1, 0.0}, config);
  ASSERT_EQ(velocities.size(), 3u);
  EXPECT_NEAR(velocities[0].v, 0.09999999999999, 1e-16);
  EXPECT_DOUBLE_EQ(velocities[1].v, 0.1);
  EXPECT_NEAR(velocities[2].v, 0.10000000000001, 1e-16);
  EXPECT_LT(velocities[0].v, velocities[1].v);
  EXPECT_LT(velocities[1].v, velocities[2].v);
}

TEST(VelocitySampler, OutputIsUniqueSortedAndStable)
{
  SamplingConfig config;
  config.v_samples = 3;
  config.w_samples = 4;
  const Velocity2D current{0.0, 0.0};
  const auto first = sampleVelocities(current, config);
  const auto second = sampleVelocities(current, config);
  ASSERT_EQ(first.size(), 15u);
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_DOUBLE_EQ(first[i].v, second[i].v);
    EXPECT_DOUBLE_EQ(first[i].omega, second[i].omega);
    if (i > 0) {
      EXPECT_TRUE(
        first[i - 1].v < first[i].v ||
        (first[i - 1].v == first[i].v && first[i - 1].omega < first[i].omega));
    }
  }
  EXPECT_DOUBLE_EQ(current.v, 0.0);
  EXPECT_DOUBLE_EQ(current.omega, 0.0);
  EXPECT_EQ(config.v_samples, 3u);
  EXPECT_EQ(config.w_samples, 4u);
}

TEST(VelocitySampler, RejectsNegativeOrNonFiniteLimits)
{
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<double, 4> invalid{{
    -0.1, infinity, -infinity, std::numeric_limits<double>::quiet_NaN()}};
  const std::array<double SamplingConfig::*, 4> fields{{
    &SamplingConfig::max_v, &SamplingConfig::max_w, &SamplingConfig::acc_v,
    &SamplingConfig::acc_w}};
  for (std::size_t i = 0; i < fields.size(); ++i) {
    for (const double value : invalid) {
      SCOPED_TRACE(::testing::Message() << "config field=" << i << ", value=" << value);
      SamplingConfig config;
      config.*fields[i] = value;
      EXPECT_THROW(sampleVelocities({0.0, 0.0}, config), std::invalid_argument);
    }
  }
}

TEST(VelocitySampler, RejectsInvalidControlPeriod)
{
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<double, 6> invalid{{
    0.0, -0.0, -0.05, infinity, -infinity, std::numeric_limits<double>::quiet_NaN()}};
  for (const double value : invalid) {
    SCOPED_TRACE(::testing::Message() << "control_dt=" << value);
    SamplingConfig config;
    config.control_dt = value;
    EXPECT_THROW(sampleVelocities({0.0, 0.0}, config), std::invalid_argument);
  }
}

TEST(VelocitySampler, RejectsSampleCountsBelowTwo)
{
  for (const std::size_t count : {0u, 1u}) {
    SCOPED_TRACE(::testing::Message() << "count=" << count);
    SamplingConfig config;
    config.v_samples = count;
    EXPECT_THROW(sampleVelocities({0.0, 0.0}, config), std::invalid_argument);
    config = SamplingConfig{};
    config.w_samples = count;
    EXPECT_THROW(sampleVelocities({0.0, 0.0}, config), std::invalid_argument);
  }
}

TEST(VelocitySampler, RejectsEveryNonFiniteCurrentComponent)
{
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<double, 3> invalid{{
    infinity, -infinity, std::numeric_limits<double>::quiet_NaN()}};
  for (const double value : invalid) {
    SCOPED_TRACE(::testing::Message() << "current component=" << value);
    EXPECT_THROW(sampleVelocities({value, 0.0}, SamplingConfig{}), std::invalid_argument);
    EXPECT_THROW(sampleVelocities({0.0, value}, SamplingConfig{}), std::invalid_argument);
  }
}

TEST(VelocitySampler, ValidatesConfigEvenWhenWindowWouldBeEmpty)
{
  SamplingConfig config;
  config.w_samples = 0;
  EXPECT_THROW(sampleVelocities({0.5, 0.0}, config), std::invalid_argument);
  config = SamplingConfig{};
  config.acc_w = -1.0;
  EXPECT_THROW(sampleVelocities({0.5, 0.0}, config), std::invalid_argument);
}

}  // 匿名命名空间
}  // 命名空间 my_dwa_controller
