#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "my_dwa_controller/quasi_static_observer.hpp"

namespace my_dwa_controller
{
namespace
{
int64_t at(int64_t ms) {return 1000000000LL + ms * 1000000LL;}
std::vector<Point> cloud(double x = 0.0, double y = 0.0)
{
  return {{x - 0.01, y}, {x, y}, {x + 0.01, y}};
}
QuasiStaticObserver::Result sample(QuasiStaticObserver & observer, int64_t ms)
{
  return observer.update(at(ms), at(ms), cloud(), true, true, false);
}
QuasiStaticObserver::Result mature(QuasiStaticObserver & observer)
{
  for (int64_t ms = 0; ms < 2100; ms += 300) {
    sample(observer, ms);
  }
  return sample(observer, 2100);
}
void settle(RobotStopObserver & observer)
{
  observer.feed(at(0), 0, 0, 0);
  observer.feed(at(250), 0, 0, 0);
  observer.feed(at(500), 0, 0, 0);
}

TEST(QuasiStaticObserver, RequiresEightFramesAndTwoSeconds)
{
  QuasiStaticObserver observer;
  for (int64_t ms = 0; ms <= 1800; ms += 300) {
    const auto result = sample(observer, ms);
    EXPECT_FALSE(result.permission);
    EXPECT_STREQ(result.reason, "window_immature");
  }
  const auto result = sample(observer, 2000);
  EXPECT_TRUE(result.permission);
  EXPECT_EQ(result.frames, 8u);
  EXPECT_EQ(result.span_ns, 2000000000LL);
  EXPECT_DOUBLE_EQ(result.diameter, 0.0);
  EXPECT_STREQ(result.reason, "stable");
}

TEST(QuasiStaticObserver, ComputesCoordinateMedianIncludingEvenMiddleAverage)
{
  QuasiStaticObserver observer;
  const std::vector<Point> points{{1, 10}, {3, 0}, {8, 4}, {6, 8}};
  const auto result = observer.update(at(0), at(0), points, true, true, false);
  EXPECT_DOUBLE_EQ(result.median[0], 4.5);
  EXPECT_DOUBLE_EQ(result.median[1], 6.0);
  EXPECT_EQ(result.frames, 1u);
}

TEST(QuasiStaticObserver, NonfiniteDerivedPositionOrDiameterClearsEvidence)
{
  QuasiStaticObserver observer;
  const double large = std::numeric_limits<double>::max();
  const std::vector<Point> points(4, Point{large, -large});
  QuasiStaticObserver::Result result{};
  for (int64_t ms = 0; ms <= 2100; ms += 300) {
    result = observer.update(at(ms), at(ms), points, true, true, false);
  }
  EXPECT_FALSE(result.permission);
  EXPECT_EQ(result.frames, 0u);
  EXPECT_STREQ(result.reason, "position_invalid");

  observer.reset();
  observer.update(at(0), at(0), cloud(large), true, true, false);
  result = observer.update(at(300), at(300), cloud(-large), true, true, false);
  EXPECT_FALSE(result.permission);
  EXPECT_EQ(result.frames, 0u);
  EXPECT_STREQ(result.reason, "position_invalid");
}

TEST(QuasiStaticObserver, RetainsOnePredecessorAtTheWindowBoundary)
{
  QuasiStaticObserver observer;
  ASSERT_TRUE(mature(observer).permission);
  const auto result = sample(observer, 2400);
  EXPECT_TRUE(result.permission);
  EXPECT_EQ(result.frames, 8u);
  EXPECT_EQ(result.span_ns, 2100000000LL);
}

TEST(QuasiStaticObserver, UsesMaximumPairwiseEuclideanDiameterAndSlidesToRecover)
{
  QuasiStaticObserver observer;
  observer.update(at(0), at(0), cloud(0.04, 0.04), true, true, false);
  for (int64_t ms = 300; ms < 2100; ms += 300) {
    sample(observer, ms);
  }
  const auto exceeded = sample(observer, 2100);
  EXPECT_FALSE(exceeded.permission);
  EXPECT_EQ(exceeded.frames, 8u);
  EXPECT_NEAR(exceeded.diameter, std::hypot(0.04, 0.04), 1e-14);
  EXPECT_STREQ(exceeded.reason, "range_exceeded");
  const auto recovered = sample(observer, 2400);
  EXPECT_TRUE(recovered.permission);
  EXPECT_EQ(recovered.frames, 8u);
  EXPECT_DOUBLE_EQ(recovered.diameter, 0.0);
}

TEST(QuasiStaticObserver, AcceptsFiveCentimetreDiameterIncludingRoundingTolerance)
{
  QuasiStaticObserver observer;
  observer.update(at(0), at(0), cloud(0.03, 0.04), true, true, false);
  for (int64_t ms = 300; ms < 2100; ms += 300) {
    sample(observer, ms);
  }
  EXPECT_TRUE(sample(observer, 2100).permission);
}

TEST(QuasiStaticObserver, DuplicateSourceCannotGrantOrIncreaseEvidence)
{
  QuasiStaticObserver observer;
  ASSERT_TRUE(mature(observer).permission);
  const auto duplicate = observer.update(at(2100), at(2200), cloud(), true, true, false);
  EXPECT_FALSE(duplicate.permission);
  EXPECT_EQ(duplicate.frames, 8u);
  EXPECT_EQ(duplicate.span_ns, 2100000000LL);
  EXPECT_STREQ(duplicate.reason, "source_not_new");
  EXPECT_TRUE(sample(observer, 2400).permission);
}

TEST(QuasiStaticObserver, SourceFreshnessAndGapUseInclusive350Milliseconds)
{
  QuasiStaticObserver observer;
  sample(observer, 0);
  const auto edge = observer.update(at(350), at(700), cloud(), true, true, false);
  EXPECT_EQ(edge.frames, 2u);
  EXPECT_STREQ(edge.reason, "window_immature");
  const auto gap = observer.update(at(700) + 1, at(700) + 1, cloud(), true, true, false);
  EXPECT_EQ(gap.frames, 1u);
  EXPECT_STREQ(gap.reason, "source_gap");
  const auto stale = observer.update(at(800), at(1150) + 1, cloud(), true, true, false);
  EXPECT_EQ(stale.frames, 0u);
  EXPECT_STREQ(stale.reason, "stale_source");
  const auto future = observer.update(at(900), at(900) - 1, cloud(), true, true, false);
  EXPECT_EQ(future.frames, 0u);
  EXPECT_STREQ(future.reason, "stale_source");
}

TEST(QuasiStaticObserver, ClockReversalAndExplicitResetStartNewWindows)
{
  QuasiStaticObserver observer;
  ASSERT_TRUE(mature(observer).permission);
  const auto reversed = sample(observer, 2000);
  EXPECT_FALSE(reversed.permission);
  EXPECT_EQ(reversed.frames, 1u);
  EXPECT_STREQ(reversed.reason, "clock_reversed");
  observer.reset();
  const auto restarted = sample(observer, 2000);
  EXPECT_EQ(restarted.frames, 1u);
  EXPECT_STREQ(restarted.reason, "window_immature");
}

TEST(QuasiStaticObserver, InvalidObservationRobotMotionAndTrackerMotionClearTheWindow)
{
  for (int cause = 0; cause < 5; ++cause) {
    SCOPED_TRACE(cause);
    QuasiStaticObserver observer;
    ASSERT_TRUE(mature(observer).permission);
    auto points = cloud();
    if (cause == 3) {points.resize(2);}
    if (cause == 4) {points[1][0] = std::numeric_limits<double>::quiet_NaN();}
    const auto invalid = observer.update(
      at(2400), at(2400), points, cause != 0, cause != 1, cause == 2);
    EXPECT_FALSE(invalid.permission);
    EXPECT_EQ(invalid.frames, 0u);
    const auto resumed = sample(observer, 2700);
    EXPECT_EQ(resumed.frames, 1u);
    EXPECT_FALSE(resumed.permission);
  }
}

TEST(RobotStopObserver, RequiresHalfSecondOfRawLinearAndAngularQuiet)
{
  RobotStopObserver observer;
  EXPECT_FALSE(observer.stoppedAt(at(0), at(0)));
  observer.feed(at(0), 0, 0, 0);
  observer.feed(at(250), 0, 0, 0);
  observer.feed(at(499), 0, 0, 0);
  EXPECT_FALSE(observer.stoppedAt(at(499), at(499)));
  observer.feed(at(500), 0, 0, 0);
  EXPECT_TRUE(observer.stoppedAt(at(500), at(500)));
  observer.reset();
  EXPECT_FALSE(observer.stoppedAt(at(500), at(500)));
}

TEST(RobotStopObserver, EnforcesStrictLinearNormAndRotationLimits)
{
  const std::vector<Point> velocities{{0.01, 0}, {0, 0.01}, {0.008, 0.008}};
  for (const auto & velocity : velocities) {
    RobotStopObserver observer;
    settle(observer);
    ASSERT_TRUE(observer.stoppedAt(at(500), at(500)));
    observer.feed(at(600), velocity[0], velocity[1], 0);
    EXPECT_FALSE(observer.stoppedAt(at(600), at(600)));
  }
  for (double w : {-0.05, 0.05}) {
    RobotStopObserver observer;
    settle(observer);
    observer.feed(at(600), 0, 0, w);
    EXPECT_FALSE(observer.stoppedAt(at(600), at(600)));
    observer.feed(at(850), 0, 0, 0.049);
    observer.feed(at(1100), 0, 0, -0.049);
    observer.feed(at(1350), 0, 0, 0);
    EXPECT_TRUE(observer.stoppedAt(at(1350), at(1350)));
  }
}

TEST(RobotStopObserver, GapsDuplicatesReversalAndNonfiniteTwistBreakContinuity)
{
  for (int cause = 0; cause < 6; ++cause) {
    SCOPED_TRACE(cause);
    RobotStopObserver observer;
    settle(observer);
    ASSERT_TRUE(observer.stoppedAt(at(500), at(500)));
    const int64_t stamp = cause == 0 ? at(850) + 1 : cause == 1 ? at(500) :
      cause == 2 ? at(400) : at(600);
    const double bad = std::numeric_limits<double>::infinity();
    observer.feed(stamp, cause == 3 ? bad : 0, cause == 4 ? bad : 0, cause == 5 ? bad : 0);
    EXPECT_FALSE(observer.stoppedAt(stamp, stamp));
    observer.feed(stamp + 250000000LL, 0, 0, 0);
    EXPECT_FALSE(observer.stoppedAt(stamp + 250000000LL, stamp + 250000000LL));
  }
}

TEST(RobotStopObserver, ChoosesOnlyCausalOdomSnapshotsAndChecksBothAges)
{
  RobotStopObserver observer;
  settle(observer);
  observer.feed(at(700), 0.1, 0, 0);
  EXPECT_TRUE(observer.stoppedAt(at(500), at(700)));
  EXPECT_TRUE(observer.stoppedAt(at(600), at(850)));
  EXPECT_FALSE(observer.stoppedAt(at(600), at(850) + 1));
  EXPECT_FALSE(observer.stoppedAt(at(700), at(700)));
  EXPECT_FALSE(observer.stoppedAt(at(-1), at(0)));
  EXPECT_FALSE(observer.stoppedAt(at(500), at(500) - 1));
}

TEST(RobotStopObserver, CoverageWaitsForScanWhileStopUsesCausalTwist)
{
  RobotStopObserver observer;
  EXPECT_FALSE(observer.covers(at(0)));
  settle(observer);
  EXPECT_TRUE(observer.covers(at(500)));
  EXPECT_FALSE(observer.covers(at(600)));
  observer.feed(at(700), 0.1, 0, 0);
  EXPECT_TRUE(observer.covers(at(600)));
  EXPECT_TRUE(observer.stoppedAt(at(600), at(700)));
  EXPECT_FALSE(observer.stoppedAt(at(700), at(700)));
}

TEST(RobotStopObserver, FreshnessAndFeedGapIncludeExactly350Milliseconds)
{
  RobotStopObserver observer;
  observer.feed(at(0), 0, 0, 0);
  observer.feed(at(350), 0, 0, 0);
  EXPECT_TRUE(observer.stoppedAt(at(500), at(700)));
  EXPECT_FALSE(observer.stoppedAt(at(500), at(700) + 1));
  EXPECT_TRUE(observer.stoppedAt(at(700), at(700)));
  EXPECT_FALSE(observer.stoppedAt(at(700) + 1, at(700) + 1));
  observer.feed(at(700), 0, 0, 0);
  EXPECT_TRUE(observer.stoppedAt(at(700), at(700)));
}

TEST(RobotStopObserver, BoundedHistoryRetainsCausalSamplesWithinFreshnessHorizon)
{
  RobotStopObserver observer;
  for (int ms = 0; ms <= 5000; ms += 50) {
    observer.feed(at(ms), 0, 0, 0);
  }
  EXPECT_TRUE(observer.stoppedAt(at(4800), at(5000)));
  EXPECT_FALSE(observer.stoppedAt(at(3000), at(5000)));
}
}  // namespace
}  // namespace my_dwa_controller
