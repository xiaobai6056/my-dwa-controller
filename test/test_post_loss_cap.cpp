#include "gtest/gtest.h"
#include "my_dwa_controller/post_loss_cap.hpp"

namespace my_dwa_controller
{
namespace
{
TEST(PostLossCap, RequiresAnObservedClosePredictionAndExpiresAt600Milliseconds)
{
  PostLossCap cap;
  EXPECT_FALSE(cap.update(0, false));
  EXPECT_FALSE(cap.update(100000000, true));
  EXPECT_TRUE(cap.update(100000001, false));
  EXPECT_TRUE(cap.update(699999999, false));
  EXPECT_FALSE(cap.update(700000000, false));
}

TEST(PostLossCap, ReacquisitionRefreshesTheWindowAndClockResetClearsIt)
{
  PostLossCap cap;
  EXPECT_FALSE(cap.update(1000000000, true));
  EXPECT_TRUE(cap.update(1200000000, false));
  EXPECT_FALSE(cap.update(1300000000, true));
  EXPECT_TRUE(cap.update(1800000000, false));
  EXPECT_FALSE(cap.update(1700000000, false));
}

TEST(PostLossCap, LimitsReachableSamplesWithoutRaisingAnExistingLimit)
{
  SamplingConfig sampling;
  sampling.acc_v = 2.5;
  sampling.max_v = .26;
  PostLossCap::limit(sampling, 0.);
  EXPECT_DOUBLE_EQ(sampling.max_v, .12);
  EXPECT_DOUBLE_EQ(sampleVelocities({}, sampling).back().v, .12);
  sampling.max_v = .26;
  PostLossCap::limit(sampling, .26);
  const auto braking = sampleVelocities({.26, 0.}, sampling);
  ASSERT_FALSE(braking.empty());
  EXPECT_NEAR(braking.back().v, .135, 1.e-12);
  sampling.max_v = .10;
  PostLossCap::limit(sampling, 0.);
  EXPECT_DOUBLE_EQ(sampling.max_v, .10);
}
}  // namespace
}  // namespace my_dwa_controller
