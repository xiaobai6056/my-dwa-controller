#include <cmath>
#include <limits>
#include <gtest/gtest.h>

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
