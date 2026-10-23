#include <gtest/gtest.h>
#include <stdexcept>
#include "my_dwa_controller/risk_governor.hpp"

namespace my_dwa_controller
{
namespace
{
ConflictEvidence clear()
{
  ConflictEvidence e;
  e.map_state = ConflictMapState::Changed;
  e.new_map_state = true;
  e.corridor_known = true;
  e.path_aligned = true;
  e.known_inner_distance = 2.0;
  return e;
}

ConflictEvidence conflict(double distance)
{
  auto e = clear();
  e.inner_conflict = true;
  e.inner = {distance, distance + 0.2};
  return e;
}

TEST(RiskGovernor, OuterPresenceAloneIsNormalAndWarningCapsSpeed)
{
  RiskGovernor governor;
  auto e = clear();
  e.outer_core_cells = 40;
  EXPECT_DOUBLE_EQ(governor.decide(e, {0.26, 0}, 0.26).max_linear_speed, 0.26);
  e.outer_risk = true;
  const auto d = governor.decide(e, {0.26, 0}, 0.26);
  EXPECT_EQ(d.state, RiskState::Limited);
  EXPECT_DOUBLE_EQ(d.max_linear_speed, 0.12);
}

TEST(RiskGovernor, SafeConflictBrakesBeforeWaitingAndAccountsForReactionAndRadius)
{
  RiskGovernor governor;
  const auto d = governor.decide(conflict(1.0), {0.26, 0.0}, 0.26);
  EXPECT_EQ(d.state, RiskState::Braking);
  EXPECT_NEAR(d.stopping_distance, 0.26 * 0.35 + 0.26 * 0.26 / 0.6, 1e-12);
  EXPECT_NEAR(d.available_distance, 0.68, 1e-12);
  EXPECT_NEAR(d.max_linear_speed, 0.245, 1e-12);
  EXPECT_FALSE(d.allow_wait);
  EXPECT_EQ(governor.decide(conflict(1.0), {0.0, 0.2}, 0.26).state, RiskState::Braking);
  const auto stopped = governor.decide(conflict(1.0), {0.0, 0.0}, 0.26);
  EXPECT_EQ(stopped.state, RiskState::Wait);
  EXPECT_TRUE(stopped.allow_wait);
  EXPECT_DOUBLE_EQ(stopped.max_linear_speed, 0.0);
}

TEST(RiskGovernor, InsufficientStoppingRoomPreservesDwaInsteadOfForcingZero)
{
  RiskGovernor governor;
  const auto d = governor.decide(conflict(0.35), {0.26, 0}, 0.26);
  EXPECT_EQ(d.state, RiskState::DwaFallback);
  EXPECT_TRUE(d.cannot_stop);
  EXPECT_FALSE(d.allow_wait);
  EXPECT_DOUBLE_EQ(d.max_linear_speed, 0.26);
}

TEST(RiskGovernor, OnlyDifferentFullyKnownClearStatesReleaseWait)
{
  RiskGovernor governor;
  EXPECT_TRUE(governor.decide(conflict(1.0), {0, 0}, 0.26).allow_wait);
  auto duplicate = clear();
  duplicate.new_map_state = false;
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(governor.decide(duplicate, {0, 0}, 0.26).state, RiskState::Wait);
  }
  auto unknown = clear();
  unknown.corridor_known = false;
  EXPECT_EQ(governor.decide(unknown, {0, 0}, 0.26).state, RiskState::Wait);
  EXPECT_EQ(governor.decide(clear(), {0, 0}, 0.26).state, RiskState::Wait);
  EXPECT_EQ(governor.decide(clear(), {0, 0}, 0.26).state, RiskState::Wait);
  auto d = governor.decide(clear(), {0, 0}, 0.26);
  EXPECT_EQ(d.state, RiskState::Recovering);
  EXPECT_GT(d.max_linear_speed, 0.0);
  EXPECT_LE(d.max_linear_speed, 0.015 + 1e-12);
  EXPECT_FALSE(d.allow_wait);
  for (int i = 0; i < 30; ++i) {
    d = governor.decide(clear(), {0, 0}, 0.26);
  }
  EXPECT_EQ(d.state, RiskState::Normal);
}

TEST(RiskGovernor, RecurringRiskResetsClearCountAndResetRestoresNormal)
{
  RiskGovernor governor;
  governor.decide(conflict(1.0), {0, 0}, 0.26);
  governor.decide(clear(), {0, 0}, 0.26);
  governor.decide(clear(), {0, 0}, 0.26);
  governor.decide(conflict(1.0), {0, 0}, 0.26);
  EXPECT_EQ(governor.decide(clear(), {0, 0}, 0.26).state, RiskState::Wait);
  governor.reset();
  EXPECT_EQ(governor.decide(clear(), {0, 0}, 0.26).state, RiskState::Normal);
}

TEST(RiskGovernor, ExternalLimitAlwaysBoundsDecisionAndInvalidConfigIsRejected)
{
  RiskGovernor governor;
  auto e = clear();
  e.outer_risk = true;
  EXPECT_DOUBLE_EQ(governor.decide(e, {0, 0}, 0.05).max_linear_speed, 0.05);
  RiskGovernorConfig config;
  config.brake_acceleration = 0.0;
  EXPECT_THROW(RiskGovernor{config}, std::invalid_argument);
}

TEST(RiskGovernor, UnknownOrMisalignedPathCannotCertifyStopAndWaitHandlesDrift)
{
  RiskGovernor governor;
  auto e = conflict(1.0);
  e.path_aligned = false;
  EXPECT_TRUE(governor.decide(e, {0.26, 0}, 0.26).cannot_stop);
  e.path_aligned = true;
  e.corridor_known = false;
  e.known_inner_distance = 0.0;
  EXPECT_TRUE(governor.decide(e, {0.26, 0}, 0.26).cannot_stop);
  EXPECT_TRUE(governor.decide(conflict(1.0), {0, 0}, 0.26).allow_wait);
  auto unavailable = clear();
  unavailable.corridor_known = false;
  const auto drift = governor.decide(unavailable, {0.1, 0}, 0.26);
  EXPECT_EQ(drift.state, RiskState::Braking);
  EXPECT_FALSE(drift.allow_wait);
}

TEST(RiskGovernor, MapReinitializationIsNotASequenceOfClearConfirmations)
{
  RiskGovernor governor;
  governor.decide(conflict(1.0), {0, 0}, 0.26);
  auto first = clear();
  first.map_state = ConflictMapState::First;
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(governor.decide(first, {0, 0}, 0.26).state, RiskState::Wait);
  }
}
TEST(RiskGovernor, RecoveryDoesNotAccelerateWhenEvidenceIsUnavailable)
{
  for (int missing = 0; missing < 3; ++missing) {
    RiskGovernor governor;
    governor.decide(conflict(1.0), {0, 0}, 0.26);
    governor.decide(clear(), {0, 0}, 0.26);
    governor.decide(clear(), {0, 0}, 0.26);
    const auto recovering = governor.decide(clear(), {0, 0}, 0.26);
    auto e = clear();
    if (missing == 0) {e.corridor_known = false;}
    if (missing == 1) {e.path_aligned = false;}
    if (missing == 2) {e.map_state = ConflictMapState::First;}
    for (int i = 0; i < 5; ++i) {
      EXPECT_LE(
        governor.decide(e, {0, 0}, 0.26).max_linear_speed,
        recovering.max_linear_speed);
    }
  }
}

TEST(RiskGovernor, SmallResidualMotionStillNeedsReachableZeroBeforeWait)
{
  RiskGovernor governor;
  governor.decide(conflict(1.0), {0, 0}, 0.26);
  auto missing = clear();
  missing.new_map_state = false;
  const auto d = governor.decide(missing, {0.019, 0}, 0.26);
  EXPECT_EQ(d.state, RiskState::Braking);
  EXPECT_FALSE(d.allow_wait);
}

TEST(RiskGovernor, WaitRequiresReachableZeroAngularVelocity)
{
  RiskGovernorConfig config;
  config.angular_acceleration = 0.1;
  RiskGovernor governor(config);
  const auto d = governor.decide(conflict(1.0), {0, 0.04}, 0.26);
  EXPECT_EQ(d.state, RiskState::Braking);
  EXPECT_FALSE(d.allow_wait);
}

TEST(RiskGovernor, OnlyTheObservedInnerPrefixIsNeededToCertifyStopping)
{
  RiskGovernor governor;
  auto e = conflict(1.0);
  e.corridor_known = false;  // Unobserved Outer or far end; still observed through conflict.
  EXPECT_EQ(governor.decide(e, {0.26, 0}, 0.26).state, RiskState::Braking);
  e.known_inner_distance = 0.9;
  EXPECT_TRUE(governor.decide(e, {0.26, 0}, 0.26).cannot_stop);
  e.known_inner_distance = 1.0;  // Touching unknown interval also rejects certification.
  EXPECT_TRUE(governor.decide(e, {0.26, 0}, 0.26).cannot_stop);
}

TEST(RiskGovernor, LoggedFallbackExitRequiresAlignedDistinctClearStates)
{
  // prefix_fix_D01_t2/launch.log:332,333,342. Absolute times 8.0,8.2,8.6.
  RiskGovernorConfig config;
  config.robot_radius = 0.23414213530762959;
  RiskGovernor governor(config);
  auto e = conflict(0.580);
  e.corridor_known = false;
  e.known_inner_distance = 1.5;
  EXPECT_EQ(governor.decide(e, {0.26, 0.013}, 0.26).state, RiskState::Braking);
  e.inner.start = 0.105;
  e.known_inner_distance = 1.464;
  EXPECT_EQ(governor.decide(e, {0.245086, 0.004447}, 0.26).state, RiskState::DwaFallback);
  e = clear();
  e.path_aligned = false;  // A replan turns the path away; the robot has not aligned yet.
  auto d = governor.decide(e, {0.017342, 0.1152}, 0.26);
  EXPECT_EQ(d.state, RiskState::DwaFallback);
  EXPECT_DOUBLE_EQ(d.max_linear_speed, 0.26);  // Do not replace DWA with unchecked braking.
  EXPECT_FALSE(d.allow_wait);
  e.path_aligned = true;
  e.corridor_known = false;
  EXPECT_EQ(governor.decide(e, {0.02, 0}, 0.26).state, RiskState::DwaFallback);
  e = clear();
  e.map_state = ConflictMapState::First;
  EXPECT_EQ(governor.decide(e, {0.02, 0}, 0.26).state, RiskState::DwaFallback);
  EXPECT_EQ(governor.decide(clear(), {0.02, 0}, 0.26).state, RiskState::DwaFallback);
  e = clear();
  e.new_map_state = false;
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(governor.decide(e, {0.02, 0}, 0.26).state, RiskState::DwaFallback);
  }
  EXPECT_EQ(governor.decide(clear(), {0.02, 0}, 0.26).state, RiskState::DwaFallback);
  EXPECT_EQ(governor.decide(clear(), {0.02, 0}, 0.26).state, RiskState::Normal);
}

TEST(RiskGovernor, FallbackReacquiresWarningAndRestartsClearCountOnNewConflict)
{
  RiskGovernor governor;
  governor.decide(conflict(0.1), {0.26, 0}, 0.26);
  auto e = clear();
  e.outer_risk = true;
  const auto warning = governor.decide(e, {0.26, 0}, 0.26);
  EXPECT_EQ(warning.state, RiskState::Limited);
  EXPECT_DOUBLE_EQ(warning.max_linear_speed, 0.12);
  governor.decide(conflict(0.1), {0.12, 0}, 0.26);
  governor.decide(clear(), {0.12, 0}, 0.26);
  governor.decide(clear(), {0.12, 0}, 0.26);
  governor.decide(conflict(0.1), {0.12, 0}, 0.26);
  EXPECT_EQ(governor.decide(clear(), {0.12, 0}, 0.26).state, RiskState::DwaFallback);
}

TEST(RiskGovernor, LoggedLimitedToFallbackKeepsDwaRangeAndExternalLimit)
{
  // prefix_fix_D01_t4/launch.log:339,346. Keep the hard escape maximum, while
  // preferring feasible non-accelerating candidates in the Controller.
  RiskGovernor governor;
  auto e = clear();
  e.outer_risk = true;
  EXPECT_DOUBLE_EQ(governor.decide(e, {0.26, 0}, 0.26).max_linear_speed, 0.12);
  e = conflict(0.002);
  e.corridor_known = false;
  e.known_inner_distance = 1.459;
  auto d = governor.decide(e, {0.120076, 0}, 0.26);
  EXPECT_EQ(d.state, RiskState::DwaFallback);
  EXPECT_DOUBLE_EQ(d.max_linear_speed, 0.26);
  ASSERT_TRUE(d.preferred_max_linear_speed);
  EXPECT_DOUBLE_EQ(*d.preferred_max_linear_speed, 0.120076);
  EXPECT_FALSE(d.allow_wait);
  EXPECT_TRUE(d.cannot_stop);
  e = clear();
  e.path_aligned = false;
  d = governor.decide(e, {0.12, 0}, 0.15);
  EXPECT_DOUBLE_EQ(d.max_linear_speed, 0.15);
  ASSERT_TRUE(d.preferred_max_linear_speed);
  EXPECT_DOUBLE_EQ(*d.preferred_max_linear_speed, 0.12);
}

TEST(RiskGovernor, FallbackPreferenceHandlesStandstillExternalCapAndClearRelease)
{
  RiskGovernor governor;
  auto d = governor.decide(conflict(0.1), {0.26, 0.1}, 0.15);
  ASSERT_TRUE(d.preferred_max_linear_speed);
  EXPECT_DOUBLE_EQ(*d.preferred_max_linear_speed, 0.15);
  d = governor.decide(conflict(0.1), {0, 0.1}, 0.26);
  ASSERT_TRUE(d.preferred_max_linear_speed);
  EXPECT_DOUBLE_EQ(*d.preferred_max_linear_speed, 0.0);
  EXPECT_FALSE(d.allow_wait);
  governor.decide(clear(), {0.12, 0}, 0.26);
  EXPECT_TRUE(governor.decide(clear(), {0.12, 0}, 0.26).preferred_max_linear_speed);
  d = governor.decide(clear(), {0.12, 0}, 0.26);
  EXPECT_EQ(d.state, RiskState::Normal);
  EXPECT_FALSE(d.preferred_max_linear_speed);
}
}  // namespace
}  // namespace my_dwa_controller

namespace my_dwa_controller
{
TEST(ScanRiskGovernor, StaleOrLostScanCannotConfirmClearOrRaisePriorBrakeCap)
{
  RiskGovernor governor;
  ConflictEvidence e;
  e.map_state = ConflictMapState::Changed;
  e.corridor_known = e.path_aligned = e.new_map_state = true;
  e.known_inner_distance = 2.0;
  e.inner_conflict = true;
  e.inner = {1.0, 1.3};
  const auto braking = governor.decide(e, {.15, 0}, .26);
  e.inner.start = .25;  // New near conflict invalidates formerly safe stop.
  e.scan_enabled = true;
  e.scan_fresh = true;
  auto d = governor.decide(e, {.14, 0}, .26);
  EXPECT_EQ(d.state, RiskState::DwaFallback);
  EXPECT_LE(d.max_linear_speed, braking.max_linear_speed);
  e.inner_conflict = false;
  e.scan_fresh = false;
  for (int i = 0; i < 10; ++i) {
    d = governor.decide(e, {.1, 0}, .26);
    EXPECT_EQ(d.state, RiskState::DwaFallback);
    EXPECT_LE(d.max_linear_speed, braking.max_linear_speed);
  }
}
}  // namespace my_dwa_controller

namespace my_dwa_controller
{
TEST(ScanEmergencyGovernor, LocalAreaProofClearsDespiteUnrelatedUnknownClustersAndFarMapEdge)
{
  RiskGovernor governor;
  ConflictEvidence e;
  e.map_state = ConflictMapState::Changed;
  e.scan_enabled = e.scan_fresh = e.scan_area_active = true;
  e.scan_unresolved = true;  // Unrelated ordinary cluster, not the old risk region.
  e.path_aligned = true; e.known_inner_distance = 2;
  e.inner_conflict = true; e.inner = {1, 1.5};
  EXPECT_EQ(governor.decide(e, {0, 0}, .26).state, RiskState::Wait);
  e.inner_conflict = false;
  e.scan_area_known = e.scan_area_clear = true;
  e.corridor_known = false;
  e.scan_new_state = false;
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(governor.decide(e, {0, 0}, .26).state, RiskState::Wait);
  }
  e.scan_new_state = true;
  EXPECT_EQ(governor.decide(e, {0, 0}, .26).state, RiskState::Wait);
  EXPECT_EQ(governor.decide(e, {0, 0}, .26).state, RiskState::Wait);
  auto d = governor.decide(e, {0, 0}, .26);
  ASSERT_EQ(d.state, RiskState::Recovering);
  EXPECT_FALSE(d.release_scan_risk);
  e.scan_fresh = false;
  EXPECT_LE(governor.decide(e, {0, 0}, .26).max_linear_speed, d.max_linear_speed);
  e.scan_fresh = true;
  for (int i = 0; i < 30 && d.state != RiskState::Normal; ++i) {
    d = governor.decide(e, {0, 0}, .26);
  }
  EXPECT_EQ(d.state, RiskState::Normal);
  EXPECT_TRUE(d.release_scan_risk);
}
}  // namespace my_dwa_controller

namespace my_dwa_controller
{
TEST(ScanEmergencyGovernor, OriginalAreaProofDoesNotDependOnCurrentPathAlignment)
{
  RiskGovernor governor;
  ConflictEvidence e;
  e.map_state = ConflictMapState::Changed;
  e.scan_enabled = e.scan_fresh = e.scan_area_active = true;
  e.path_aligned = true; e.known_inner_distance = 2;
  e.inner_conflict = true; e.inner = {1, 1.5};
  EXPECT_EQ(governor.decide(e, {-.0003, 0}, .26).state, RiskState::Wait);
  e.inner_conflict = false;
  e.scan_area_clear = true; e.scan_new_state = true;
  e.path_aligned = false;  // Same-goal replan/yaw cannot invalidate old-region visibility.
  RiskDecision d;
  for (int i = 0; i < 30; ++i) {
    d = governor.decide(e, {-.0003, 0}, .26);
  }
  EXPECT_EQ(d.state, RiskState::Normal);
  EXPECT_TRUE(d.release_scan_risk);
}
}  // namespace my_dwa_controller
