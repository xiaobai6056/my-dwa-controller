#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "my_dwa_controller/controller_diagnostics.hpp"

namespace my_dwa_controller
{
namespace
{
TEST(ControllerDiagnostics, PreservesLegacyDefaultFieldsIncludingFailureSnapshots)
{
  CycleDiagnostics cycle;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.0;
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.2;
  velocity.angular.z = -0.3;
  cycle.setInput(pose, velocity, {0.6, 0.6, {}});
  cycle.terminal_radius = 0.25;
  cycle.yaw_tolerance = 0.05;
  ControllerDiagnostics diagnostics;
  diagnostics.report(rclcpp::get_logger("diagnostic_test"), cycle, "test failure", true);
  const auto text = diagnostics.format(cycle, 1.25, "test failure");
  for (const char * field : {
        "[DWA_DEBUG] stage=state error=test failure frame=odom pose=(1.000000,2.000000,nan)",
        "follow_lookahead=0.600000 heading_lookahead=0.600000",
        "true_goal_distance=nan terminal_radius=0.250000 heading_enabled=0 goal_phase=normal",
        "yaw_tolerance=0.050000 tolerance_fallback[xy,yaw]=[1,1]",
        "input_vel=(0.200000,-0.300000)",
        "sampled=0 processed=0 valid=0 valid_forward=0 scored=0",
        "v_range=(inf,-inf) w_range=(inf,-inf)",
        "reject[empty,nonfinite,bounds,inscribed,lethal,unknown]=[0,0,0,0,0,0]",
        "best={present:0", "best_forward={present:0", "fastest_valid={present:0",
        "rotation_cost:0.000000", "costs[ob,speed,goal,path,heading]:[",
        "best_ties=0 ms[total,state_wait,path,map_wait,predict,obstacle,score]=[1.250000",
        "cumulative[calls,over50,mean_ms,max_ms]=[1,",
        "weights=[0.100000,0.100000,0.600000,0.200000,0.050000]"})
  {
    EXPECT_NE(text.find(field), std::string::npos) << field;
  }
}

TEST(ControllerDiagnostics, RecordsCandidatesWithoutChangingInputsOrRetainingTheirContainers)
{
  CycleDiagnostics cycle;
  std::vector<Velocity2D> samples{{0.0, -0.2}, {0.2, 0.1}, {0.4, 0.2}};
  cycle.recordSamples(samples);
  EXPECT_EQ(cycle.sampled, 3u);
  EXPECT_DOUBLE_EQ(cycle.v_min, 0.0);
  EXPECT_DOUBLE_EQ(cycle.v_max, 0.4);
  EXPECT_DOUBLE_EQ(cycle.w_min, -0.2);
  EXPECT_DOUBLE_EQ(cycle.w_max, 0.2);
  const ObstacleEvaluation obstacle{true, 100, 25.0};
  ScoreResult score{};
  score.total_cost = 1.0;
  cycle.recordValid(samples[1]);
  cycle.recordScored(samples[1], score, obstacle, 0.5);
  cycle.recordBest(true);
  score.total_cost = 2.0;
  cycle.recordValid(samples[2]);
  cycle.recordScored(samples[2], score, obstacle, 0.7);
  EXPECT_EQ(cycle.valid, 2u);
  EXPECT_EQ(cycle.valid_forward, 2u);
  EXPECT_EQ(cycle.scored, 2u);
  EXPECT_EQ(cycle.ties, 1u);
  samples.clear();
  EXPECT_DOUBLE_EQ(cycle.chosen.command.v, 0.2);
  EXPECT_DOUBLE_EQ(cycle.forward.command.v, 0.2);
  EXPECT_DOUBLE_EQ(cycle.fastest.command.v, 0.4);
  EXPECT_DOUBLE_EQ(cycle.chosen.rotation_cost, 0.5);
  EXPECT_DOUBLE_EQ(cycle.chosen.mean_obstacle, 25.0);
  EXPECT_EQ(cycle.chosen.max_obstacle, 100u);
}

TEST(ControllerDiagnostics, CountsSuppressedAndFailedCyclesAndResetsOnlyOnExplicitReset)
{
  ControllerDiagnostics diagnostics;
  CycleDiagnostics cycle;
  const auto logger = rclcpp::get_logger("diagnostic_test");
  diagnostics.report(logger, cycle, "none");
  diagnostics.report(logger, cycle, "none");  // Native throttle may suppress output.
  diagnostics.report(logger, cycle, "failure", true);
  EXPECT_EQ(diagnostics.totals().calls, 3u);
  EXPECT_GE(diagnostics.totals().sum_ms, diagnostics.totals().max_ms);
  EXPECT_GE(diagnostics.totals().max_ms, 0.0);
  diagnostics.reset();
  EXPECT_EQ(diagnostics.totals().calls, 0u);
  EXPECT_EQ(diagnostics.totals().over_50ms, 0u);
  EXPECT_DOUBLE_EQ(diagnostics.totals().sum_ms, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.totals().max_ms, 0.0);
}

TEST(ControllerDiagnostics, KeepsStageTimingAndPartiallyProcessedCandidateCounters)
{
  CycleTiming timing;
  timing.stateLockAcquired();
  CycleDiagnostics cycle(timing);
  cycle.beginTiming(DiagnosticTiming::Path);
  cycle.finishTiming();
  EXPECT_STREQ(cycle.stage, "path");
  cycle.beginTiming(DiagnosticTiming::MapLock);
  cycle.finishTiming();
  cycle.beginTiming(DiagnosticTiming::Prediction);
  cycle.finishTiming();
  cycle.beginTiming(DiagnosticTiming::Obstacle);
  cycle.finishTiming();
  cycle.beginTiming(DiagnosticTiming::Scoring);
  cycle.finishTiming();
  EXPECT_STREQ(cycle.stage, "scoring");
  EXPECT_GE(cycle.elapsedMs(), cycle.score_ms);
  ++cycle.processed;
  ++cycle.terminal_rejected;
  ++cycle.rejected[static_cast<std::size_t>(ObstacleRejection::Lethal)];
  EXPECT_EQ(cycle.processed, 1u);
  EXPECT_EQ(cycle.valid, 0u);
  EXPECT_EQ(cycle.scored, 0u);
  EXPECT_EQ(cycle.terminal_rejected, 1u);
  EXPECT_EQ(cycle.rejected[5], 1u);
}
TEST(ControllerDiagnostics, FormatsFallbackPreferenceOnlyWhenPresent)
{
  ControllerDiagnostics diagnostics;
  CycleDiagnostics cycle;
  cycle.conflict_enabled = true;
  EXPECT_EQ(
    diagnostics.format(cycle, 1.0, "none").find("fallback_preference["),
    std::string::npos);
  cycle.risk.preferred_max_linear_speed = 0.12;
  cycle.preferred_valid = 0;
  cycle.accelerating_escape = true;
  const auto text = diagnostics.format(cycle, 1.0, "none");
  EXPECT_NE(
    text.find("fallback_preference[cap,valid,accelerating_escape]=[0.120000,0,1]"),
    std::string::npos);
}

}  // namespace
}  // namespace my_dwa_controller
