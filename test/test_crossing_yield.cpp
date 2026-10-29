#include <gtest/gtest.h>
#include "my_dwa_controller/crossing_yield.hpp"
using namespace my_dwa_controller;
Pose2D pose(double x, double y = 0) {Pose2D p;p.x = x;p.y = y;return p;}
ScanSnapshot frame(double t)
{
  ScanSnapshot s;s.stamp.sec = int(t);s.stamp.nanosec = int((t - int(t)) * 1e9 + .5);
  s.frame_id = "odom";
  s.laser_pose = {0, 0, 0};s.angle_increment = 2 * std::acos(-1.) / 360;s.range_min = .12;
  s.range_max = 20;s.ranges.assign(360, 5.f);return s;
}
ObstacleObservation actor()
{
  ObstacleObservation o;o.points = {{{1.35, -.8}}, {{1.85, -.4}}};o.partial_motion_rank = 1;
  o.partial_motion_vy = .2;return o;
}
struct YieldTest : testing::Test {CrossingYield policy;std::vector<Pose2D> path{pose(0), pose(4)};
  Trajectory brake;
  void SetUp()override {brake.poses = {pose(.5), pose(.565)};}
  YieldDecision step(ScanSnapshot & s, bool admission = true)
  {
    int64_t now = int64_t(s.stamp.sec) * 1000000000 + s.stamp.nanosec;return policy.update(
      &s, path, pose(
        .5), {0, 0}, brake, now, admission);
  }
  void trigger() {auto s = frame(1);s.obstacles = {actor()};EXPECT_TRUE(step(s).hold);}
};
TEST_F(YieldTest, UsesPartialNormalEvidenceWithoutFullVelocity) {
  auto s = frame(1);s.obstacles = {actor()};auto r = step(s);EXPECT_TRUE(r.triggered);EXPECT_TRUE(
    r.hold);EXPECT_TRUE(r.safe_stop);EXPECT_EQ(r.evidence_rank, 1);
}
TEST_F(YieldTest, UnknownDoesNotCancelIntent) {
  trigger();auto s = frame(1.2);s.obstacles = {actor()};s.obstacles[0].partial_motion_rank = 0;
  EXPECT_TRUE(step(s).hold);
}
TEST_F(YieldTest, ClearBeforeCrossingDoesNotRelease) {
  trigger();auto s = frame(1.2);EXPECT_TRUE(step(s).hold);s = frame(1.4);EXPECT_TRUE(step(s).hold);
}
TEST_F(YieldTest, OccupiedThenTwoFreshClearAndNewDwaAdmissionRelease) {
  trigger();auto s = frame(1.2);auto o = actor();o.partial_motion_rank = 0;o.points = {{{1.5, 0}}};
  s.obstacles = {o};s.ranges[0] = 1.5;EXPECT_TRUE(step(s).occupied);
  s = frame(1.4);EXPECT_TRUE(step(s).hold);EXPECT_TRUE(step(s).hold);s = frame(1.6);EXPECT_TRUE(
    step(
      s,
      false).hold);auto r = step(s, true);EXPECT_TRUE(r.released);EXPECT_FALSE(r.hold);
}
TEST_F(YieldTest, StaleClearCannotRelease) {
  trigger();auto s = frame(2);auto r = policy.update(
    &s, path, pose(
      .5), {0, 0}, brake, 2500000000, true);EXPECT_TRUE(r.hold);
}
TEST_F(YieldTest, StaticAndLongitudinalMotionDoNotTrigger) {
  auto s = frame(1);s.obstacles = {actor()};s.obstacles[0].partial_motion_rank = 0;EXPECT_FALSE(
    step(
      s).hold);s = frame(1.2);auto o = actor();o.partial_motion_vx = .2;o.partial_motion_vy = 0;
  s.obstacles = {o};
  EXPECT_FALSE(step(s).hold);
}
TEST_F(YieldTest, LateTriggerIsReportedWithoutCallingZeroSafe) {
  brake.poses = {pose(1.1), pose(1.17)};auto s = frame(1);s.obstacles = {actor()};auto r = step(s);
  EXPECT_TRUE(r.hold);EXPECT_FALSE(r.safe_stop);EXPECT_LT(r.margin, 0);
}
TEST_F(YieldTest, ResetClearsIntent) {
  trigger();policy.reset();auto s = frame(2);EXPECT_FALSE(step(s).hold);
}

TEST_F(YieldTest, AssociationExtendsUnionWithoutFullOrPartialVelocity)
{
  trigger();
  auto s = frame(1.2);
  s.previous_stamp_ns = 1000000000;
  auto o = actor();
  o.partial_motion_rank = 0;
  o.previous_cluster_index = 0;
  o.points = {{{1.1, -.8}}, {{2.1, -.4}}};
  s.obstacles = {o};
  const auto extended = step(s);
  EXPECT_DOUBLE_EQ(extended.entry, 1.1 - .27);
  EXPECT_DOUBLE_EQ(extended.exit, 2.1 + .27);
  EXPECT_TRUE(extended.hold);
  // No association freezes the union even if a new cluster has motion evidence.
  s = frame(1.4);
  o.points = {{{.8, -.8}}, {{2.8, -.4}}};
  o.partial_motion_rank = 2;
  o.previous_cluster_index = -1;
  s.obstacles = {o};
  const auto lost = step(s);
  EXPECT_DOUBLE_EQ(lost.entry, extended.entry);
  EXPECT_DOUBLE_EQ(lost.exit, extended.exit);
  EXPECT_TRUE(lost.hold);
}

TEST_F(YieldTest, SkippedAssociationBaselineCannotExtendRegion)
{
  trigger();
  auto s = frame(1.4);
  s.previous_stamp_ns = 1200000000;
  auto o = actor();
  o.previous_cluster_index = 0;
  o.points = {{{.8, -.8}}, {{2.8, -.4}}};
  s.obstacles = {o};
  const auto r = step(s);
  EXPECT_DOUBLE_EQ(r.entry, 1.35 - .27);
  EXPECT_DOUBLE_EQ(r.exit, 1.85 + .27);
  EXPECT_TRUE(r.hold);
}

TEST_F(YieldTest, DuplicateScanDoesNotIncrementClearCount)
{
  trigger();
  auto s = frame(1.2);
  auto o = actor();o.points = {{{1.5, 0}}};
  s.obstacles = {o};s.ranges[0] = 1.5;
  ASSERT_TRUE(step(s).occupied);
  s = frame(1.4);
  ASSERT_EQ(step(s).clear_scans, 1);
  for (int i = 0; i < 5; ++i) {
    const auto r = step(s);
    EXPECT_TRUE(r.hold);
    EXPECT_FALSE(r.released);
    EXPECT_FALSE(r.clear);
    EXPECT_EQ(r.clear_scans, 1);
  }
}

TEST_F(YieldTest, NullFutureAndExpiredScansPreserveIntentAndClearCount)
{
  trigger();
  auto s = frame(1.2);
  auto o = actor();o.points = {{{1.5, 0}}};
  s.obstacles = {o};s.ranges[0] = 1.5;
  ASSERT_TRUE(step(s).occupied);
  s = frame(1.4);
  ASSERT_EQ(step(s).clear_scans, 1);
  for (const int64_t now : {1399999999LL, 1750000001LL}) {
    const auto r = policy.update(&s, path, pose(.5), {0, 0}, brake, now, true);
    EXPECT_TRUE(r.hold);EXPECT_EQ(r.clear_scans, 1);EXPECT_FALSE(r.released);
  }
  const auto r = policy.update(nullptr, path, pose(.5), {0, 0}, brake, 1600000000, true);
  EXPECT_TRUE(r.hold);EXPECT_EQ(r.clear_scans, 1);EXPECT_FALSE(r.released);
}

TEST_F(YieldTest, ReleaseRequiresBothMeasuredSpeedsBelowLimits)
{
  trigger();
  auto s = frame(1.2);
  auto o = actor();o.points = {{{1.5, 0}}};
  s.obstacles = {o};s.ranges[0] = 1.5;
  ASSERT_TRUE(step(s).occupied);
  s = frame(1.4);ASSERT_EQ(step(s).clear_scans, 1);
  s = frame(1.6);
  for (const Velocity2D speed : {Velocity2D{.02, 0}, Velocity2D{0, .05}}) {
    const auto r = policy.update(&s, path, pose(.5), speed, brake, 1600000000, true);
    EXPECT_TRUE(r.hold);EXPECT_FALSE(r.released);EXPECT_EQ(r.clear_scans, 2);
  }
  EXPECT_TRUE(step(s).released);
}

TEST_F(YieldTest, BlockedFreshRayResetsConsecutiveClearEvidence)
{
  trigger();
  auto s = frame(1.2);
  auto o = actor();o.points = {{{1.5, 0}}};
  s.obstacles = {o};s.ranges[0] = 1.5;
  ASSERT_TRUE(step(s).occupied);
  s = frame(1.4);ASSERT_EQ(step(s).clear_scans, 1);
  s = frame(1.6);s.ranges[0] = 1.2;
  EXPECT_EQ(step(s).clear_scans, 0);
  s = frame(1.8);EXPECT_TRUE(step(s).hold);
  s = frame(2.0);EXPECT_TRUE(step(s).released);
}

TEST_F(YieldTest, RegionUsesFrozenPathBasisWhileNewPathStillWaits)
{
  trigger();
  path = {pose(0, 0), pose(0, 4)};
  auto s = frame(1.2);
  const auto r = step(s);
  EXPECT_TRUE(r.hold);
  EXPECT_FALSE(r.released);
  EXPECT_DOUBLE_EQ(r.entry, 1.35 - .27);
  EXPECT_DOUBLE_EQ(r.exit, 1.85 + .27);
}

TEST_F(YieldTest, OutsideForwardPathDoesNotTrigger)
{
  auto s = frame(1);
  auto o = actor();o.points = {{{-.8, -.8}}, {{-.4, -.4}}};
  s.obstacles = {o};EXPECT_FALSE(step(s).triggered);
  s = frame(1.2);o.points = {{{4.4, -.8}}, {{4.8, -.4}}};
  s.obstacles = {o};EXPECT_FALSE(step(s).triggered);
}

TEST_F(YieldTest, DetourAndReentryBothRetainPendingHold)
{
  trigger();
  auto s = frame(1.2);
  path = {pose(0), pose(0, 1), pose(4, 1)};
  const auto bypass = step(s);
  EXPECT_TRUE(bypass.hold);
  EXPECT_FALSE(bypass.released);
  EXPECT_FALSE(bypass.triggered);
  EXPECT_DOUBLE_EQ(bypass.entry, 1.35 - .27);
  path = {pose(0), pose(4)};
  const auto reentry = step(s);
  EXPECT_TRUE(reentry.hold);
  EXPECT_FALSE(reentry.triggered);
  EXPECT_DOUBLE_EQ(reentry.exit, 1.85 + .27);
}

TEST_F(YieldTest, PathGeometryChangesCannotCancelPendingHold)
{
  trigger();
  auto s = frame(1.2);
  path = {pose(1.5, -1), pose(1.5, 1)};
  EXPECT_TRUE(step(s).hold);  // Pending risk also holds for this crossing-shaped path.
  path = {pose(0, .27), pose(4, .27)};
  EXPECT_TRUE(step(s).hold);  // Boundary changes do not cancel pending risk.
  path = {pose(0, .271), pose(4, .271)};
  EXPECT_TRUE(step(s).hold);
  path = {pose(1.1, -1), pose(1.1, 1)};
  EXPECT_TRUE(step(s).hold);  // Moving across the envelope does not cancel pending risk.
  path = {pose(1.07, -1), pose(1.07, 1)};
  EXPECT_TRUE(step(s).hold);
}

TEST_F(YieldTest, MissingObservationKeepsPendingHoldDespitePathChanges)
{
  trigger();
  path = {pose(0), pose(0, 2)};
  EXPECT_TRUE(policy.update(nullptr, path, pose(.5), {0, 0}, brake, 2000000000, true).hold);
  path = {pose(0), pose(4)};
  EXPECT_TRUE(policy.update(nullptr, path, pose(.5), {0, 0}, brake, 2000000000, true).hold);
}

TEST_F(YieldTest, SinglePointAndEmptyPathsKeepPendingHold)
{
  trigger();
  auto s = frame(1.2);
  path = {pose(1.5)};
  EXPECT_TRUE(step(s).hold);
  path = {pose(.1)};
  EXPECT_TRUE(step(s).hold);
  path.clear();
  EXPECT_TRUE(step(s).hold);
  path = {pose(0), pose(4)};
  EXPECT_TRUE(step(s).hold);
}

TEST_F(YieldTest, FourSecondsOfCoveredEmptyScansClearNeverOccupiedRegion)
{
  trigger();
  for (int i = 1; i < 20; ++i) {
    auto s = frame(1 + .2 * i);
    const auto r = step(s, false);
    ASSERT_TRUE(r.hold) << i;
    ASSERT_FALSE(r.released);
  }
  auto s = frame(5);
  const auto released = step(s, false);
  EXPECT_TRUE(released.released);
  EXPECT_FALSE(released.hold);
  EXPECT_FALSE(released.occupied);
  EXPECT_EQ(released.clear_scans, 0);
  EXPECT_EQ(released.evidence_rank, 0);
  s = frame(5.2);
  EXPECT_FALSE(step(s).hold);
  s = frame(5.4);s.obstacles = {actor()};
  EXPECT_TRUE(step(s).triggered);  // Fully cleaned; a new region can form.
}

TEST_F(YieldTest, EmptyFallbackReleasesWaitDespiteDetour)
{
  trigger();
  path = {pose(0), pose(0, 2)};
  for (int i = 1; i <= 20; ++i) {
    auto s = frame(1 + .2 * i);
    const auto r = step(s, false);
    EXPECT_EQ(r.hold, i < 20);
    EXPECT_EQ(r.released, i == 20);
  }
  path = {pose(0), pose(4)};
  auto s = frame(5.2);
  EXPECT_FALSE(step(s).hold);
}

TEST_F(YieldTest, MissingStaleFutureAndUncoveredObservationsPauseEmptyTime)
{
  for (int mode = 0; mode < 4; ++mode) {
    SCOPED_TRACE(mode);
    policy.reset();trigger();
    for (int i = 1; i <= 10; ++i) {
      auto s = frame(1 + .2 * i);
      ASSERT_TRUE(step(s, false).hold);
    }  // Two seconds credited.
    auto s = frame(3.2);
    if (mode == 0) {
      EXPECT_TRUE(policy.update(nullptr, path, pose(.5), {0, 0}, brake, 3200000000, false).hold);
    } else if (mode == 1) {
      EXPECT_TRUE(policy.update(&s, path, pose(.5), {0, 0}, brake, 3600000000, false).hold);
    } else if (mode == 2) {
      EXPECT_TRUE(policy.update(&s, path, pose(.5), {0, 0}, brake, 3100000000, false).hold);
    } else {
      s.ranges.clear();EXPECT_TRUE(step(s, false).hold);
    }
    s = frame(10);ASSERT_TRUE(step(s, false).hold);
    for (int i = 1; i < 10; ++i) {
      s = frame(10 + .2 * i);ASSERT_TRUE(step(s, false).hold);
    }
    s = frame(12);
    EXPECT_TRUE(step(s, false).released);  // Preserve two seconds, skip interruption.
  }
}

TEST_F(YieldTest, SourceGapAndDuplicateReadsDoNotAccumulateMissingTime)
{
  trigger();
  auto s = frame(1);
  for (int i = 1; i <= 3; ++i) {
    EXPECT_TRUE(
      policy.update(
        &s, path, pose(.5), {0, 0}, brake,
        1000000000 + i * 100000000, false).hold);
  }
  s = frame(10);ASSERT_TRUE(step(s, false).hold);
  for (int i = 1; i < 20; ++i) {
    s = frame(10 + .2 * i);ASSERT_TRUE(step(s, false).hold);
  }
  s = frame(14);EXPECT_TRUE(step(s, false).released);
}

TEST_F(YieldTest, ObstacleReappearanceRestartsFullEmptyDuration)
{
  trigger();
  for (int i = 1; i <= 10; ++i) {
    auto s = frame(1 + .2 * i);ASSERT_TRUE(step(s, false).hold);
  }
  auto s = frame(3.2);
  auto o = actor();o.partial_motion_rank = 0;o.points = {{{1.5, 0}}};
  s.obstacles = {o};s.ranges[0] = 1.5;
  ASSERT_TRUE(step(s, false).occupied);
  for (int i = 0; i < 20; ++i) {
    s = frame(3.4 + .2 * i);ASSERT_TRUE(step(s, false).hold);
  }
  s = frame(7.4);
  EXPECT_TRUE(step(s, false).released);
}

TEST_F(YieldTest, OccludingRayRestartsEmptyDurationWithoutTrackedOccupancy)
{
  trigger();
  for (int i = 1; i <= 10; ++i) {
    auto s = frame(1 + .2 * i);ASSERT_TRUE(step(s, false).hold);
  }
  auto s = frame(3.2);s.ranges[0] = 1.0;
  EXPECT_FALSE(step(s, false).clear);
  for (int i = 0; i < 20; ++i) {
    s = frame(3.4 + .2 * i);ASSERT_TRUE(step(s, false).hold);
  }
  s = frame(7.4);EXPECT_TRUE(step(s, false).released);
}

TEST_F(YieldTest, NarrowRegionCannotAccumulateWithoutRayCoverage)
{
  auto s = frame(1);auto o = actor();
  o.points = {{{1.5, -.8}}, {{1.51, -.4}}};s.obstacles = {o};
  s.ranges.assign(360, 1.f);
  ASSERT_TRUE(step(s).triggered);
  for (int i = 1; i <= 25; ++i) {
    s = frame(1 + .2 * i);s.ranges.assign(360, 1.f);
    const auto r = step(s, false);
    EXPECT_FALSE(r.clear);
    EXPECT_TRUE(r.hold);
  }
}

TEST_F(YieldTest, RegionExpansionRestartsEmptyDuration)
{
  trigger();
  for (int i = 1; i <= 10; ++i) {
    auto s = frame(1 + .2 * i);
    s.previous_stamp_ns = 1000000000LL + (i - 1) * 200000000LL;
    auto o = actor();o.previous_cluster_index = 0;s.obstacles = {o};
    ASSERT_TRUE(step(s, false).hold);
  }
  auto s = frame(3.2);s.previous_stamp_ns = 3000000000;
  auto o = actor();o.previous_cluster_index = 0;o.points = {{{1, -.8}}, {{2, -.4}}};
  s.obstacles = {o};ASSERT_TRUE(step(s, false).hold);
  for (int i = 1; i < 20; ++i) {
    s = frame(3.2 + .2 * i);ASSERT_TRUE(step(s, false).hold);
  }
  s = frame(7.2);EXPECT_TRUE(step(s, false).released);
}

TEST_F(YieldTest, InvalidRaysAndLimitedFieldOfViewPauseEmptyDuration)
{
  for (int mode = 0; mode < 3; ++mode) {
    SCOPED_TRACE(mode);
    policy.reset();trigger();
    for (int i = 1; i <= 10; ++i) {
      auto s = frame(1 + .2 * i);ASSERT_TRUE(step(s, false).hold);
    }
    auto s = frame(3.2);
    if (mode == 0) {s.ranges.assign(360, std::numeric_limits<float>::quiet_NaN());}
    if (mode == 1) {s.ranges.assign(360, .01);}
    if (mode == 2) {s.ranges.resize(2);}
    const auto unavailable = step(s, false);
    EXPECT_FALSE(unavailable.clear);
    EXPECT_TRUE(unavailable.hold);
    for (int i = 0; i < 10; ++i) {
      s = frame(10 + .2 * i);ASSERT_TRUE(step(s, false).hold);
    }
    s = frame(12);EXPECT_TRUE(step(s, false).released);
  }
}

TEST_F(YieldTest, UnknownMotionDoesNotReleasePersistentOccupiedRegion)
{
  trigger();
  for (int i = 1; i <= 25; ++i) {
    auto s = frame(1 + .2 * i);
    auto o = actor();o.partial_motion_rank = 0;o.velocity_valid = false;
    o.points = {{{1.5, 0}}};s.obstacles = {o};s.ranges[0] = 1.5;
    const auto r = step(s);
    EXPECT_TRUE(r.occupied);
    EXPECT_TRUE(r.hold);
    EXPECT_FALSE(r.released);
  }
}

TEST_F(YieldTest, VisibleUnclusteredObstacleResetsEvenWhenOtherRaysAreUnavailable)
{
  trigger();
  for (int i = 1; i <= 10; ++i) {
    auto s = frame(1 + .2 * i);ASSERT_TRUE(step(s, false).hold);
  }
  auto s = frame(3.2);
  // An invalid early ray must not hide a valid later return. A single return
  // need not form a Tracker cluster, so the ray pass must detect it directly.
  s.ranges[350] = std::numeric_limits<float>::quiet_NaN();
  s.ranges[0] = 1.5;
  EXPECT_FALSE(step(s, false).clear);
  for (int i = 0; i < 20; ++i) {
    s = frame(3.4 + .2 * i);ASSERT_TRUE(step(s, false).hold);
  }
  s = frame(7.4);EXPECT_TRUE(step(s, false).released);
}

TEST_F(YieldTest, DetourMustKeepWaitingUntilNormalRelease)
{
  trigger();
  path = {pose(0), pose(0, 1), pose(4, 1)};
  auto s = frame(1.2);
  auto o = actor();o.partial_motion_rank = 0;o.points = {{{1.5, 0}}};
  s.obstacles = {o};s.ranges[0] = 1.5;
  const auto occupied = step(s);
  EXPECT_TRUE(occupied.hold);
  ASSERT_TRUE(occupied.occupied);
  s = frame(1.4);
  const auto first_clear = step(s);
  EXPECT_TRUE(first_clear.hold);
  EXPECT_EQ(first_clear.clear_scans, 1);
  EXPECT_TRUE(step(s).hold);  // Same source must not satisfy two-frame release.
  s = frame(1.6);
  const auto not_admitted = step(s, false);
  EXPECT_TRUE(not_admitted.hold);
  EXPECT_EQ(not_admitted.clear_scans, 2);
  const auto still_moving = policy.update(
    &s, path, pose(.5), {.1, 0}, brake, 1600000000, true);
  EXPECT_TRUE(still_moving.hold);
  EXPECT_FALSE(still_moving.released);
  const auto released = step(s);
  EXPECT_TRUE(released.released);
  EXPECT_FALSE(released.hold);
  path = {pose(0), pose(4)};
  EXPECT_FALSE(step(s).hold);  // Returning to the path cannot resurrect released risk.
}

TEST_F(YieldTest, DetourMustKeepWaitingUntilFourSecondFallback)
{
  trigger();
  path = {pose(0), pose(0, 2)};
  for (int i = 1; i < 20; ++i) {
    auto s = frame(1 + .2 * i);
    const auto r = step(s, false);
    EXPECT_TRUE(r.hold) << i;
    EXPECT_FALSE(r.released);
  }
  auto s = frame(5);
  const auto released = step(s, false);
  EXPECT_TRUE(released.released);
  EXPECT_FALSE(released.hold);
  EXPECT_FALSE(released.occupied);
  EXPECT_EQ(released.evidence_rank, 0);
  EXPECT_EQ(released.clear_scans, 0);
}
#include "my_dwa_controller/motion_predictor.hpp"

struct ApproachTest : YieldTest
{
  MotionConfig config;
  void SetUp() override {YieldTest::SetUp(); config.angular_acceleration = 3.2; trigger();}
  bool allowed(
    double x, double speed, double target, double horizon = 1.7,
    double omega = 0)
  {
    config.horizon = horizon;
    MotionPredictor predictor(config);
    return policy.canApproach(
      path,
      predictor.predict(pose(x), {speed, omega}, {}, 0),
      predictor.predict(pose(x), {speed, omega}, {target, omega}, .05),
      horizon, config.step, std::max(std::abs(speed), std::abs(target)));
  }
};
TEST_F(ApproachTest, FarRegionAllowsForwardMotion) {
  EXPECT_TRUE(allowed(.5, .26, .26));
}
TEST_F(ApproachTest, NearBoundaryWaits) {
  EXPECT_FALSE(allowed(1.03, .26, .26));
}
TEST_F(ApproachTest, SafeCurrentBrakeDoesNotImplySafeNextAction)
{
  MotionPredictor predictor(config);
  const auto current = predictor.predict(pose(1.0), {.26, 0}, {}, 0);
  EXPECT_LT(current.trajectory.poses.back().x, 1.08);
  EXPECT_FALSE(allowed(1.0, .26, .26));
}
TEST_F(ApproachTest, SpeedAndAccelerationChangeAdmission)
{
  EXPECT_TRUE(allowed(.8, .1, .1));
  EXPECT_FALSE(allowed(.8, .26, .26));
  EXPECT_FALSE(allowed(.867, 0, .26));
}
TEST_F(ApproachTest, TruncatedPredictionCannotProveStop)
{
  EXPECT_FALSE(allowed(.5, .26, .26, .3));
  EXPECT_TRUE(allowed(.5, .26, .26, .43));
}
TEST_F(ApproachTest, DetourCannotBypassFrozenEntry)
{
  path = {pose(0), pose(0, 1), pose(4, 1)};
  EXPECT_FALSE(allowed(.5, .1, .1));
}
TEST_F(ApproachTest, ReverseOrMissingPathCannotApproach)
{
  EXPECT_FALSE(allowed(.5, -.1, -.1));
  path.clear(); EXPECT_FALSE(allowed(.5, .1, .1));
}
TEST_F(ApproachTest, ReleaseRestoresUnrestrictedDwa)
{
  policy.reset(); path = {pose(0), pose(0, 4)};
  EXPECT_TRUE(allowed(1.1, .26, .26));
}
TEST_F(ApproachTest, RotatedFrozenBasisUsesMatchingWorldCoordinates)
{
  policy.reset(); path = {pose(0, 0), pose(0, 4)};
  auto s = frame(2);auto o = actor();
  for (auto & p : o.points) {
    const double x = p[0];p[0] = -p[1];p[1] = x;
  }
  o.partial_motion_vx = -.2;o.partial_motion_vy = 0;s.obstacles = {o};
  Trajectory brake; brake.poses = {pose(0, .5)};
  ASSERT_TRUE(policy.update(&s, path, pose(0, .5), {}, brake, 2000000000, true).triggered);
  auto p = pose(0, .5);p.theta = std::acos(-1.) / 2;
  MotionPredictor predictor(config);
  EXPECT_TRUE(
    policy.canApproach(
      path, predictor.predict(p, {.26, 0}, {}, 0),
      predictor.predict(p, {.26, 0}, {.26, 0}, .05), 1.7, .05, .26));
}
TEST_F(ApproachTest, SidewaysBackupStillCannotCrossEntry)
{
  MotionPredictor predictor(config);
  auto current = predictor.predict(pose(.5), {.1, 0}, {}, 0);
  auto candidate = predictor.predict(pose(.5), {.1, 0}, {.1, 0}, .05);
  candidate.trajectory.poses.back() = pose(1.09, .1);
  EXPECT_FALSE(policy.canApproach(path, current, candidate, 1.7, .05, .1));
}
TEST_F(ApproachTest, NormalReleaseRestoresMotionBeyondFormerBoundary)
{
  auto s = frame(1.2);auto o = actor();o.points = {{{1.5, 0}}};
  s.obstacles = {o};s.ranges[0] = 1.5;ASSERT_TRUE(step(s).occupied);
  s = frame(1.4);ASSERT_TRUE(step(s).hold);
  s = frame(1.6);ASSERT_TRUE(step(s).released);
  EXPECT_TRUE(allowed(1.1, .26, .26));
}
TEST_F(ApproachTest, SafeApproachDoesNotLatchStop)
{
  EXPECT_FALSE(policy.latchHold(!allowed(.5, .26, .26)));
  EXPECT_TRUE(allowed(.6, .26, .26));
}
TEST_F(ApproachTest, FirstRejectedApproachLatchesDuringDeceleration)
{
  ASSERT_TRUE(policy.latchHold(!allowed(1.03, .26, .26)));
  EXPECT_FALSE(allowed(.5, .05, .1));
  EXPECT_TRUE(policy.latchHold(false));
}
TEST_F(ApproachTest, RepeatedScanAndPathChangesCannotUnlockStop)
{
  ASSERT_TRUE(policy.latchHold(true));
  auto s = frame(1);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(step(s).hold);
    path = {pose(0), pose(0, 2)};
    EXPECT_TRUE(policy.latchHold(false));
    path = {pose(0), pose(4)};
    EXPECT_FALSE(allowed(.5, 0, .1));
  }
}
TEST_F(ApproachTest, NormalReleaseClearsLatchWithoutBlockingObservation)
{
  ASSERT_TRUE(policy.latchHold(true));
  auto s = frame(1.2);auto o = actor();o.points = {{{1.5, 0}}};
  s.obstacles = {o};s.ranges[0] = 1.5;ASSERT_TRUE(step(s).occupied);
  s = frame(1.4);EXPECT_EQ(step(s).clear_scans, 1);
  s = frame(1.6);ASSERT_TRUE(step(s).released);
  EXPECT_FALSE(policy.latchHold(false));
  s = frame(1.8);s.obstacles = {actor()};ASSERT_TRUE(step(s).triggered);
  EXPECT_TRUE(allowed(.5, .1, .1));
}
TEST_F(ApproachTest, FourSecondFallbackClearsLatchAndAllowsNewApproach)
{
  ASSERT_TRUE(policy.latchHold(true));
  for (int i = 1; i < 20; ++i) {
    auto s = frame(1 + .2 * i);ASSERT_TRUE(step(s, false).hold);
    EXPECT_TRUE(policy.latchHold(false));
  }
  auto s = frame(5);ASSERT_TRUE(step(s, false).released);
  EXPECT_FALSE(policy.latchHold(false));
  s = frame(5.2);s.obstacles = {actor()};ASSERT_TRUE(step(s).triggered);
  EXPECT_TRUE(allowed(.5, .1, .1));
}
TEST_F(ApproachTest, FixedReserveHasStrictTwentyCentimeterBoundary)
{
  MotionRollout current;current.trajectory.poses = {pose(.5)};
  MotionRollout candidate;candidate.trajectory.command = {.1, 0};
  for (const auto & sample : std::vector<std::pair<double, bool>>{
    {.875 - .000001, true}, {.875, false}, {.875 + .000001, false}, {.88, false}})
  {
    candidate.trajectory.poses = {pose(.5), pose(sample.first)};
    EXPECT_EQ(policy.canApproach(path, current, candidate, 1.7, .05, .1), sample.second)
      << sample.first;
  }
}
TEST_F(ApproachTest, FixedReserveRetainsSamplingAllowanceAndStopLatch)
{
  MotionRollout current;current.trajectory.poses = {pose(.5)};
  MotionRollout candidate;candidate.trajectory.command = {.1, 0};
  candidate.trajectory.poses = {pose(.5), pose(.876)};
  // Raw margin .204 m exceeds .20, but leaves less than the old .005 sample allowance.
  EXPECT_FALSE(policy.canApproach(path, current, candidate, 1.7, .05, .1));
  EXPECT_TRUE(policy.latchHold(true));
  candidate.trajectory.poses = {pose(.5), pose(.6)};
  EXPECT_FALSE(policy.canApproach(path, current, candidate, 1.7, .05, .1));
}
