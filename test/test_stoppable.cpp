#include <cmath>
#include <limits>
#include <gtest/gtest.h>

#include "my_dwa_controller/dynamic_filter.hpp"
#include "my_dwa_controller/motion_predictor.hpp"
#include "my_dwa_controller/obstacle_evaluator.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace my_dwa_controller;

TEST(Motion, NominalMotionStartsFromMeasuredVelocity)
{
  MotionConfig config;
  config.response_delay = 0.1;
  const auto r = MotionPredictor(config).predict(Pose2D{}, {0.1, 0}, {0.3, 0}, 1.7);
  ASSERT_EQ(r.trajectory.poses.size(), 35u);
  EXPECT_NEAR(r.trajectory.poses[2].x, 0.01, 1e-12);
  EXPECT_NEAR(r.trajectory.poses[3].x, 0.01625, 1e-12);
  EXPECT_NEAR(r.trajectory.poses.back().x, 0.47, 1e-10);
}

TEST(Motion, CandidateBackupIncludesExecutionAndStoppedOccupancy)
{
  const auto r = MotionPredictor({}).predict(Pose2D{}, {0.26, 0}, {0.26, 0}, 0.05);
  ASSERT_EQ(r.trajectory.poses.size(), 35u);
  EXPECT_NEAR(r.stop_time, 0.43, 1e-12);
  EXPECT_NEAR(r.trajectory.poses.back().x, 0.078, 1e-10);
  EXPECT_DOUBLE_EQ(r.trajectory.poses[10].x, r.trajectory.poses.back().x);
}

TEST(Motion, ImmediateBrakeUsesTheSameDynamics)
{
  const auto r = MotionPredictor({}).predict(Pose2D{}, {0.26, 0}, {}, 0.0);
  ASSERT_EQ(r.trajectory.poses.size(), 35u);
  EXPECT_NEAR(r.stop_time, 0.38, 1e-12);
  EXPECT_NEAR(r.trajectory.poses.back().x, 0.065, 1e-10);
  EXPECT_DOUBLE_EQ(r.trajectory.command.v, 0.0);
}

TEST(Motion, ZeroTargetDoesNotAddAnotherCoastingPeriod)
{
  const MotionPredictor model({});
  const auto now = model.predict(Pose2D{}, {0.26, 0}, {}, 0.0);
  const auto next = model.predict(Pose2D{}, {0.26, 0}, {}, 0.05);
  ASSERT_EQ(next.trajectory.poses.size(), 35u);
  ASSERT_EQ(now.trajectory.poses.size(), 35u);
  EXPECT_NEAR(now.trajectory.poses.back().x, next.trajectory.poses.back().x, 1e-12);
  EXPECT_NEAR(now.stop_time, next.stop_time, 1e-12);
}

TEST(Motion, ReverseAndAngularVelocityBrakeTowardZero)
{
  const auto r = MotionPredictor({}).predict(Pose2D{}, {-0.26, -0.4}, {}, 0);
  ASSERT_EQ(r.trajectory.poses.size(), 35u);
  EXPECT_LT(r.trajectory.poses.back().x, 0);
  EXPECT_NEAR(r.trajectory.poses.back().theta, -0.128, 1e-10);
  EXPECT_NEAR(r.stop_time, 0.52, 1e-12);
}

TEST(Motion, StopsInsideMeshInterval)
{
  MotionConfig config;
  config.response_delay = 0.075;
  const auto r = MotionPredictor(config).predict(Pose2D{}, {0.26, 0}, {}, 0);
  ASSERT_EQ(r.trajectory.poses.size(), 35u);
  EXPECT_NEAR(r.trajectory.poses.back().x, 0.0533, 1e-10);
  EXPECT_NEAR(r.stop_time, 0.335, 1e-12);
}

TEST(Motion, HorizonDoesNotInventCompletedStopping)
{
  MotionConfig config;
  config.linear_acceleration = 0.1;
  const auto r = MotionPredictor(config).predict(Pose2D{}, {0.26, 0}, {}, 0);
  ASSERT_EQ(r.trajectory.poses.size(), 35u);
  EXPECT_GT(r.stop_time, config.horizon);
  EXPECT_GT(r.trajectory.poses.back().x, r.trajectory.poses[33].x);
}

TEST(Motion, StationaryRobotHasNoFictitiousDelayMotion)
{
  const auto r = MotionPredictor({}).predict(Pose2D{}, {}, {}, 0);
  ASSERT_EQ(r.trajectory.poses.size(), 35u);
  EXPECT_DOUBLE_EQ(r.stop_time, 0.0);
  EXPECT_DOUBLE_EQ(r.trajectory.poses.back().x, 0.0);
}

TEST(Motion, InvalidConfigurationFailsAtConstruction)
{
  MotionConfig config;
  config.linear_acceleration = 0;
  EXPECT_THROW(MotionPredictor{config}, std::invalid_argument);
}

TEST(StaticAdmissibility, CanStopBeforeWallEvenWhenLongNominalReachesIt)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.01, -0.5, -0.5, 0);
  for (unsigned int x = 70; x < 100; ++x) {
    for (unsigned int y = 0; y < 100; ++y) {
      map.setCost(x, y, 254);
    }
  }
  const MotionPredictor model({});
  const auto normal = model.predict(Pose2D{}, {0.26, 0}, {0.26, 0}, 1.7);
  const auto backup = model.predict(Pose2D{}, {0.26, 0}, {0.26, 0}, 0.05);
  EXPECT_FALSE(evaluateTrajectory(normal.trajectory, map).collision_free);
  EXPECT_TRUE(evaluateTrajectory(backup.trajectory, map).collision_free);
  EXPECT_GT(trajectoryObstacleCost(normal.trajectory, map), 0.0);
}

TEST(StaticScoring, LethalUnknownAndOutsideAreNotZeroCost)
{
  nav2_costmap_2d::Costmap2D map(20, 20, 0.05, -0.5, -0.5, 254);
  Trajectory trajectory{{0, 0}, {Pose2D{}}};
  EXPECT_DOUBLE_EQ(trajectoryObstacleCost(trajectory, map), 252.0);
  trajectory.poses.back().x = 5;
  EXPECT_DOUBLE_EQ(trajectoryObstacleCost(trajectory, map), 252.0);
}

TEST(StaticScoring, EmptyTrajectoryIsNotAnObstacleFreeMeasurement)
{
  nav2_costmap_2d::Costmap2D map(20, 20, 0.05, -0.5, -0.5, 0);
  EXPECT_THROW(trajectoryObstacleCost({}, map), std::invalid_argument);
}

ScanSnapshot pointScan(double x, double y, double vx, double vy)
{
  ScanSnapshot scan;
  scan.stamp = rclcpp::Time(2000000000LL, RCL_ROS_TIME);
  scan.frame_id = "odom";
  ObstacleObservation o;
  o.points = {{x, y}};
  o.aabb.min_x = o.aabb.max_x = x;
  o.aabb.min_y = o.aabb.max_y = y;
  o.vx = vx; o.vy = vy; o.velocity_valid = true;
  scan.obstacles.push_back(o);
  return scan;
}

TEST(DynamicAdmissibility, ForwardMotionCanPassWhileItsBackupIsHitAfterStopping)
{
  const MotionPredictor model({});
  const auto normal = model.predict(Pose2D{}, {0.26, 0}, {0.26, 0}, 1.7);
  const auto backup = model.predict(Pose2D{}, {0.26, 0}, {0.26, 0}, 0.05);
  const DynamicFilter filter(0.22, 1.7, 0.05);
  auto scan = pointScan(0, 0.65, 0, -0.4);
  const auto prepared = filter.prepare(
    {&normal.trajectory, &backup.trajectory}, &scan,
    "odom", rclcpp::Time(2000000000LL, RCL_ROS_TIME));
  const auto n = filter.evaluate(normal.trajectory, prepared);
  const auto b = filter.evaluate(backup.trajectory, prepared);
  ASSERT_TRUE(n.available);
  ASSERT_TRUE(b.available);
  EXPECT_TRUE(n.conflicts.empty());
  EXPECT_FALSE(b.conflicts.empty());
}

TEST(DynamicAdmissibility, ContinuousInteriorEncounterSurvivesBroadphase)
{
  MotionConfig config;
  config.horizon = 0.2; config.step = 0.1;
  const auto r = MotionPredictor(config).predict(Pose2D{}, {}, {}, 0);
  auto scan = pointScan(0, 0.5, 0, -10);
  const DynamicFilter filter(0.22, 0.2, 0.1);
  const auto prepared = filter.prepare(
    {&r.trajectory}, &scan, "odom",
    rclcpp::Time(2000000000LL, RCL_ROS_TIME));
  const auto result = filter.evaluate(r.trajectory, prepared);
  ASSERT_TRUE(result.available);
  EXPECT_FALSE(result.conflicts.empty());
}

TEST(ObservedGeometry, UnknownNominalCrossingRemainsConstrainedBeyondClearBackup)
{
  const MotionPredictor model({});
  const auto nominal = model.predict(Pose2D{}, {0.26, 0}, {0.26, 0}, 1.7);
  const auto backup = model.predict(Pose2D{}, {0.26, 0}, {0.26, 0}, 0.05);
  auto scan = pointScan(0.5, 0, 99, -99);
  scan.obstacles[0].velocity_valid = false;
  const DynamicFilter filter(0.22, 1.7, 0.05);
  const auto prepared = filter.prepare(
    {&nominal.trajectory, &backup.trajectory}, &scan, "odom",
    rclcpp::Time(2000000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(prepared.input.input_available);
  EXPECT_TRUE(filter.observedGeometryCollision(nominal.trajectory, prepared));
  EXPECT_FALSE(filter.observedGeometryCollision(backup.trajectory, prepared));
  EXPECT_TRUE(filter.evaluate(nominal.trajectory, prepared).conflicts.empty());
  EXPECT_EQ(prepared.input.unknown_clusters, 1u);
}

TEST(ObservedGeometry, ChecksWholeSegmentsAndEachTrajectoryIndependently)
{
  Trajectory crossing{{0, 0}, std::vector<Pose2D>(3)};
  for (std::size_t i = 0; i < 3; ++i) {
    crossing.poses[i].x = i;
  }
  auto clear = crossing;
  for (auto & pose : clear.poses) {
    pose.y = 1;
  }

  auto scan = pointScan(0.5, 0, 0, 0);
  scan.obstacles[0].velocity_valid = false;
  const DynamicFilter filter(0.1, 0.2, 0.1);
  const auto prepared = filter.prepare(
    {&clear, &crossing}, &scan, "odom", rclcpp::Time(2000000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(prepared.input.input_available);
  // Every sampled pose is outside radius+margin. The segment interior crosses.
  EXPECT_TRUE(filter.observedGeometryCollision(crossing, prepared));
  EXPECT_FALSE(filter.observedGeometryCollision(clear, prepared));
}

TEST(ObservedGeometry, FarUnknownPointsDoNotRejectAndDoNotUseVelocityFields)
{
  const auto stopped = MotionPredictor({}).predict(Pose2D{}, {}, {}, 0);
  auto scan = pointScan(3, 3, std::numeric_limits<double>::quiet_NaN(), 99);
  scan.obstacles[0].velocity_valid = false;
  const DynamicFilter filter(0.22, 1.7, 0.05);
  const auto prepared = filter.prepare(
    {&stopped.trajectory}, &scan, "odom", rclcpp::Time(2000000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(prepared.input.input_available);
  EXPECT_FALSE(filter.observedGeometryCollision(stopped.trajectory, prepared));
  EXPECT_EQ(prepared.unknown_points.size(), 1u);
  EXPECT_TRUE(filter.evaluate(stopped.trajectory, prepared).conflicts.empty());
}

TEST(ObservedGeometry, MixedKnownPredictionAndUnknownSnapshotRemainIndependent)
{
  MotionConfig config;
  config.horizon = 0.2; config.step = 0.1;
  const auto stopped = MotionPredictor(config).predict(Pose2D{}, {}, {}, 0);
  auto scan = pointScan(0, 0.5, 0, -10);
  auto unknown = pointScan(0.1, 0, 123, 456).obstacles.front();
  unknown.velocity_valid = false;
  scan.obstacles.push_back(unknown);
  const DynamicFilter filter(0.1, 0.2, 0.1);
  const auto prepared = filter.prepare(
    {&stopped.trajectory}, &scan, "odom", rclcpp::Time(2000000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(prepared.input.input_available);
  const auto known = filter.evaluate(stopped.trajectory, prepared);
  ASSERT_TRUE(known.available);
  ASSERT_EQ(known.conflicts.size(), 1u);
  EXPECT_EQ(known.conflicts[0].index, 0u);
  EXPECT_TRUE(filter.observedGeometryCollision(stopped.trajectory, prepared));
  EXPECT_EQ(prepared.input.predicted_clusters, 1u);
  EXPECT_EQ(prepared.input.unknown_clusters, 1u);
}

TEST(ObservedGeometry, BrakingSweepAndStoppedOccupancyRemainConstrained)
{
  const MotionPredictor model({});
  const auto braking = model.predict(Pose2D{}, {0.26, 0}, {}, 0);
  const auto stopped = model.predict(Pose2D{}, {}, {}, 0);
  auto scan = pointScan(0.32, 0, 0, 0);
  scan.obstacles[0].velocity_valid = false;
  const DynamicFilter filter(0.22, 1.7, 0.05);
  auto prepared = filter.prepare(
    {&braking.trajectory, &stopped.trajectory}, &scan, "odom",
    rclcpp::Time(2000000000LL, RCL_ROS_TIME));
  EXPECT_TRUE(filter.observedGeometryCollision(braking.trajectory, prepared));
  EXPECT_FALSE(filter.observedGeometryCollision(stopped.trajectory, prepared));
  scan.obstacles[0].points = {{0.26, 0}};
  prepared = filter.prepare(
    {&stopped.trajectory}, &scan, "odom", rclcpp::Time(2000000000LL, RCL_ROS_TIME));
  EXPECT_TRUE(filter.observedGeometryCollision(stopped.trajectory, prepared));
}
