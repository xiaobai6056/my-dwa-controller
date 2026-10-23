#include <gtest/gtest.h>

#include <array>
#include <initializer_list>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "my_dwa_controller/obstacle_evaluator.hpp"

namespace my_dwa_controller
{
namespace
{

Pose2D makePose(double x, double y, double theta = 0.0)
{
  Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  return pose;
}

Trajectory makeTrajectory(std::initializer_list<Pose2D> poses)
{
  Trajectory trajectory{};
  trajectory.poses = poses;
  return trajectory;
}

void expectInfeasible(const ObstacleEvaluation & evaluation)
{
  EXPECT_FALSE(evaluation.collision_free);
  EXPECT_EQ(evaluation.max_cost, 0u);
  EXPECT_DOUBLE_EQ(evaluation.mean_cost, 0.0);
}

class ObstacleEvaluatorTest : public ::testing::Test
{
protected:
  // 世界坐标范围：x 属于 [-1,1)，y 属于 [-2,-0.5)，分辨率为 0.5 m。
  nav2_costmap_2d::Costmap2D costmap_{4, 3, 0.5, -1.0, -2.0, nav2_costmap_2d::FREE_SPACE};
};

TEST_F(ObstacleEvaluatorTest, AllFreePointsAreFeasible)
{
  const auto trajectory = makeTrajectory(
      {
        makePose(-0.75, -1.75), makePose(-0.25, -1.75), makePose(0.75, -0.75, 1.2)});
  const auto evaluation = evaluateTrajectory(trajectory, costmap_);

  EXPECT_TRUE(evaluation.collision_free);
  EXPECT_EQ(evaluation.max_cost, nav2_costmap_2d::FREE_SPACE);
  EXPECT_DOUBLE_EQ(evaluation.mean_cost, 0.0);
}

TEST_F(ObstacleEvaluatorTest, RejectsLethalObstacleAfterSafePoint)
{
  costmap_.setCost(0, 0, 80);
  costmap_.setCost(2, 1, nav2_costmap_2d::LETHAL_OBSTACLE);
  expectInfeasible(
    evaluateTrajectory(
      makeTrajectory({makePose(-0.75, -1.75), makePose(0.25, -1.25)}), costmap_));
}

TEST_F(ObstacleEvaluatorTest, RejectsInscribedInflatedObstacle)
{
  costmap_.setCost(2, 1, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  expectInfeasible(evaluateTrajectory(makeTrajectory({makePose(0.25, -1.25)}), costmap_));
}

TEST_F(ObstacleEvaluatorTest, RejectsUnknownCell)
{
  costmap_.setCost(2, 1, nav2_costmap_2d::NO_INFORMATION);
  expectInfeasible(evaluateTrajectory(makeTrajectory({makePose(0.25, -1.25)}), costmap_));
}

TEST_F(ObstacleEvaluatorTest, RejectsEachMapBoundaryViolation)
{
  const std::array<Pose2D, 4> outside{{
    makePose(-1.01, -1.75), makePose(1.0, -1.75),
    makePose(-0.75, -2.01), makePose(-0.75, -0.5)}};
  for (const auto & pose : outside) {
    SCOPED_TRACE(::testing::Message() << "x=" << pose.x << ", y=" << pose.y);
    expectInfeasible(evaluateTrajectory(makeTrajectory({pose}), costmap_));
  }
}

TEST_F(ObstacleEvaluatorTest, IncludesLowerMapBoundary)
{
  costmap_.setCost(0, 0, 42);
  const auto evaluation = evaluateTrajectory(makeTrajectory({makePose(-1.0, -2.0)}), costmap_);
  EXPECT_TRUE(evaluation.collision_free);
  EXPECT_EQ(evaluation.max_cost, 42u);
  EXPECT_DOUBLE_EQ(evaluation.mean_cost, 42.0);
}

TEST_F(ObstacleEvaluatorTest, RejectsEmptyTrajectory)
{
  expectInfeasible(evaluateTrajectory(Trajectory{}, costmap_));
}

TEST_F(ObstacleEvaluatorTest, RejectsEveryNonFinitePoseComponent)
{
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<double, 3> invalid{{
    infinity, -infinity, std::numeric_limits<double>::quiet_NaN()}};
  const std::array<double Pose2D::*, 3> fields{{&Pose2D::x, &Pose2D::y, &Pose2D::theta}};
  for (std::size_t i = 0; i < fields.size(); ++i) {
    for (const double value : invalid) {
      SCOPED_TRACE(::testing::Message() << "component=" << i << ", value=" << value);
      auto pose = makePose(0.25, -1.25);
      pose.*fields[i] = value;
      expectInfeasible(evaluateTrajectory(makeTrajectory({pose}), costmap_));
    }
  }
}

TEST_F(ObstacleEvaluatorTest, InvalidLatePointDoesNotReturnPartialStatistics)
{
  costmap_.setCost(0, 0, 100);
  expectInfeasible(
    evaluateTrajectory(
      makeTrajectory({makePose(-0.75, -1.75), makePose(5.0, 5.0)}), costmap_));
}

TEST_F(ObstacleEvaluatorTest, ComputesMaximumAndFractionalMean)
{
  // 三个不同栅格的代价为 10、20、70，均值为 100/3，不能使用整数除法。
  costmap_.setCost(0, 0, 10);
  costmap_.setCost(1, 0, 20);
  costmap_.setCost(2, 1, 70);
  const auto evaluation = evaluateTrajectory(
    makeTrajectory({makePose(-0.75, -1.75), makePose(-0.25, -1.75), makePose(0.25, -1.25)}),
    costmap_);

  EXPECT_TRUE(evaluation.collision_free);
  EXPECT_EQ(evaluation.max_cost, 70u);
  EXPECT_NEAR(evaluation.mean_cost, 100.0 / 3.0, 1e-12);
}

TEST_F(ObstacleEvaluatorTest, CostSumDoesNotOverflowByte)
{
  costmap_.setCost(0, 0, 200);
  costmap_.setCost(1, 0, 220);
  costmap_.setCost(2, 1, 240);
  const auto evaluation = evaluateTrajectory(
    makeTrajectory({makePose(-0.75, -1.75), makePose(-0.25, -1.75), makePose(0.25, -1.25)}),
    costmap_);

  EXPECT_TRUE(evaluation.collision_free);
  EXPECT_EQ(evaluation.max_cost, 240u);
  EXPECT_DOUBLE_EQ(evaluation.mean_cost, 220.0);
}

TEST_F(ObstacleEvaluatorTest, AcceptsHighestNonObstacleCost)
{
  costmap_.setCost(2, 1, nav2_costmap_2d::MAX_NON_OBSTACLE);
  const auto evaluation = evaluateTrajectory(makeTrajectory({makePose(0.25, -1.25)}), costmap_);
  EXPECT_TRUE(evaluation.collision_free);
  EXPECT_EQ(evaluation.max_cost, nav2_costmap_2d::MAX_NON_OBSTACLE);
  EXPECT_DOUBLE_EQ(evaluation.mean_cost, nav2_costmap_2d::MAX_NON_OBSTACLE);
}

TEST_F(ObstacleEvaluatorTest, CountsEveryPoseEvenWhenCellsRepeat)
{
  costmap_.setCost(0, 0, 20);
  costmap_.setCost(2, 1, 80);
  const auto evaluation = evaluateTrajectory(
    makeTrajectory({makePose(-0.75, -1.75), makePose(-0.6, -1.6), makePose(0.25, -1.25)}),
    costmap_);

  EXPECT_TRUE(evaluation.collision_free);
  EXPECT_EQ(evaluation.max_cost, 80u);
  EXPECT_DOUBLE_EQ(evaluation.mean_cost, 40.0);
}

TEST_F(ObstacleEvaluatorTest, ReadsCurrentMapWithoutCaching)
{
  const auto trajectory = makeTrajectory({makePose(0.25, -1.25)});
  EXPECT_TRUE(evaluateTrajectory(trajectory, costmap_).collision_free);
  costmap_.setCost(2, 1, nav2_costmap_2d::LETHAL_OBSTACLE);
  expectInfeasible(evaluateTrajectory(trajectory, costmap_));
  costmap_.setCost(2, 1, 60);
  const auto evaluation = evaluateTrajectory(trajectory, costmap_);
  EXPECT_TRUE(evaluation.collision_free);
  EXPECT_EQ(evaluation.max_cost, 60u);
  EXPECT_DOUBLE_EQ(evaluation.mean_cost, 60.0);
}

TEST_F(ObstacleEvaluatorTest, EvaluatesPosesWithoutValidatingCommand)
{
  auto trajectory = makeTrajectory({makePose(-0.75, -1.75)});
  trajectory.command = {std::numeric_limits<double>::quiet_NaN(), 0.0};
  // 速度信息不参与障碍物评估，合法性由生成候选速度的调用层负责。
  EXPECT_TRUE(evaluateTrajectory(trajectory, costmap_).collision_free);
}

}  // 匿名命名空间
}  // 命名空间 my_dwa_controller
