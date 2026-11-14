#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "my_dwa_controller/path_handler.hpp"
#include "rclcpp/clock.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace my_dwa_controller
{
namespace
{
constexpr double kTolerance = 1e-9;
const double kPi = std::acos(-1.0);

geometry_msgs::msg::PoseStamped makePose(
  double x, double y, double yaw = 0.0, const std::string & frame = "odom", int seconds = 15)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp.sec = seconds;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

nav_msgs::msg::Path makePath(
  std::initializer_list<geometry_msgs::msg::PoseStamped> poses,
  const std::string & frame = "odom")
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp.sec = 1;
  path.poses = poses;
  return path;
}

void expectPose(const Pose2D & pose, double x, double y, double yaw = 0.0)
{
  EXPECT_NEAR(pose.x, x, kTolerance);
  EXPECT_NEAR(pose.y, y, kTolerance);
  EXPECT_NEAR(std::remainder(pose.theta - yaw, 2.0 * kPi), 0.0, kTolerance);
}

class PathHandlerTest : public ::testing::Test
{
protected:
  std::shared_ptr<tf2_ros::Buffer> buffer_ = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  PathHandler handler_{buffer_};

  void addTransform(
    const std::string & parent, const std::string & child, double x, double y,
    double yaw = 0.0, int seconds = 15, bool is_static = true)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.header.stamp.sec = seconds;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    transform.transform.rotation = tf2::toMsg(q);
    ASSERT_TRUE(buffer_->setTransform(transform, "path_handler_test", is_static));
  }
};

TEST_F(PathHandlerTest, TranslatesMapPathToRobotLocalFrame)
{
  addTransform("map", "odom", 10.0, 20.0);
  const auto path = makePath(
    {makePose(10.0, 20.0, 0.0, "map"), makePose(14.0, 20.0, 0.0, "map")}, "map");
  const auto robot = makePose(1.0, 0.0);
  const auto local = handler_.getLocalPath(path, robot, "odom", 2.0);
  ASSERT_EQ(local.size(), 2u);
  // 机器人在局部系的 (1,0)，输出起点也必须为 (1,0)，不能返回地图系的 (11,20)。
  expectPose(local.front(), robot.pose.position.x, robot.pose.position.y);
  expectPose(local.back(), 3.0, 0.0);
}

TEST_F(PathHandlerTest, RotatesAndTranslatesPositionAndOrientation)
{
  addTransform("map", "odom", 10.0, 20.0, kPi / 2.0);
  const auto path = makePath(
    {makePose(10.0, 20.0, kPi / 2.0, "map"), makePose(10.0, 24.0, kPi / 2.0, "map")},
    "map");
  const auto local = handler_.getLocalPath(path, makePose(1.0, 0.0), "odom", 2.0);
  ASSERT_EQ(local.size(), 2u);
  expectPose(local.front(), 1.0, 0.0);
  expectPose(local.back(), 3.0, 0.0);
}

TEST_F(PathHandlerTest, UsesDeclaredFramesInsteadOfHardcodedMapOrOdom)
{
  addTransform("world", "tracking", -5.0, 2.0);
  const auto path = makePath(
    {makePose(-5.0, 2.0, 0.0, "world"), makePose(-1.0, 2.0, 0.0, "world")}, "world");
  const auto local = handler_.getLocalPath(path, makePose(1.0, 0.0, 0.0, "tracking"), "tracking");
  ASSERT_EQ(local.size(), 2u);
  expectPose(local.front(), 1.0, 0.0);
  expectPose(local.back(), 3.0, 0.0);
}

TEST_F(PathHandlerTest, SameFrameNeedsNoTransformEvenWithEmptyBufferAndZeroTime)
{
  const auto path = makePath({makePose(0.0, 0.0), makePose(4.0, 0.0)});
  const auto local = handler_.getLocalPath(path, makePose(1.0, 0.0, 0.0, "odom", 0), "odom");
  ASSERT_EQ(local.size(), 2u);
  expectPose(local.front(), 1.0, 0.0);
  expectPose(local.back(), 3.0, 0.0);
}

TEST_F(PathHandlerTest, ExtractsFromProjectionInMiddleAcrossCorner)
{
  const auto path = makePath({makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(2.0, 3.0)});
  const auto local = handler_.getLocalPath(path, makePose(1.0, -0.2), "odom", 2.0);
  ASSERT_EQ(local.size(), 3u);
  expectPose(local[0], 1.0, 0.0);
  expectPose(local[1], 2.0, 0.0);
  expectPose(local[2], 2.0, 1.0);
}

TEST_F(PathHandlerTest, StartsAtCornerAndStopsAtPathEnd)
{
  const auto path = makePath({makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(2.0, 1.0)});
  const auto local = handler_.getLocalPath(path, makePose(2.0, 0.0), "odom", 2.0);
  ASSERT_EQ(local.size(), 2u);
  expectPose(local.front(), 2.0, 0.0);
  expectPose(local.back(), 2.0, 1.0);
  const auto at_end = handler_.getLocalPath(path, makePose(2.0, 1.0), "odom", 2.0);
  ASSERT_EQ(at_end.size(), 1u);
  expectPose(at_end.front(), 2.0, 1.0);
}

TEST_F(PathHandlerTest, ZeroLengthReturnsProjectionAndSinglePointPathIsValid)
{
  const auto path = makePath({makePose(0.0, 0.0), makePose(4.0, 0.0)});
  const auto local = handler_.getLocalPath(path, makePose(1.0, 1.0), "odom", 0.0);
  ASSERT_EQ(local.size(), 1u);
  expectPose(local.front(), 1.0, 0.0);
  const auto single = handler_.getLocalPath(
    makePath({makePose(2.0, 3.0)}), makePose(0.0, 0.0), "odom", 2.0);
  ASSERT_EQ(single.size(), 1u);
  expectPose(single.front(), 2.0, 3.0);
}

TEST_F(PathHandlerTest, InheritsEmptyPoseFramesAndSkipsRepeatedCoordinates)
{
  const auto path = makePath(
    {makePose(0.0, 0.0, 0.0, ""), makePose(0.0, 0.0), makePose(3.0, 0.0, 0.0, "")});
  const auto local = handler_.getLocalPath(path, makePose(0.0, 0.0), "odom", 2.0);
  ASSERT_EQ(local.size(), 2u);
  expectPose(local.front(), 0.0, 0.0);
  expectPose(local.back(), 2.0, 0.0);
}

TEST_F(PathHandlerTest, PreservesShortestAngleInterpolationInReturnedPath)
{
  const auto path = makePath({makePose(0.0, 0.0, 3.0), makePose(2.0, 0.0, -3.0)});
  const auto local = handler_.getLocalPath(path, makePose(1.0, 0.0), "odom", 0.5);
  ASSERT_EQ(local.size(), 2u);
  expectPose(local.front(), 1.0, 0.0, kPi);
  expectPose(local.back(), 1.5, 0.0, 3.0 + 0.75 * (2.0 * kPi - 6.0));
}

TEST_F(PathHandlerTest, UsesRobotTimeForOneSnapshotAndIgnoresOldPlanTimes)
{
  addTransform("map", "odom", 10.0, 0.0, 0.0, 10, false);
  addTransform("map", "odom", 20.0, 0.0, 0.0, 20, false);
  auto path = makePath(
    {makePose(15.0, 0.0, 0.0, "map", 2), makePose(18.0, 0.0, 0.0, "map", 3)}, "map");
  const auto original = path;
  const auto robot = makePose(0.0, 0.0);
  const auto local = handler_.getLocalPath(path, robot, "odom", 2.0);
  ASSERT_EQ(local.size(), 2u);
  expectPose(local.front(), 0.0, 0.0);
  expectPose(local.back(), 2.0, 0.0);
  EXPECT_EQ(path, original);
  // 下一周期重新取 TF，不缓存上一周期的变换或路径投影。
  const auto later = handler_.getLocalPath(path, makePose(0.0, 0.0, 0.0, "odom", 20), "odom");
  ASSERT_EQ(later.size(), 1u);
  expectPose(later.front(), -2.0, 0.0);
}

TEST_F(PathHandlerTest, RejectsUnavailableTransform)
{
  const auto path = makePath({makePose(0.0, 0.0, 0.0, "map")}, "map");
  EXPECT_THROW(handler_.getLocalPath(path, makePose(0.0, 0.0), "odom"), tf2::TransformException);
}

TEST_F(PathHandlerTest, RejectsPastAndFutureExtrapolationInsteadOfUsingLatest)
{
  addTransform("map", "odom", 10.0, 0.0, 0.0, 10, false);
  addTransform("map", "odom", 20.0, 0.0, 0.0, 20, false);
  const auto path = makePath({makePose(15.0, 0.0, 0.0, "map")}, "map");
  for (int seconds : {9, 21}) {
    EXPECT_THROW(
      handler_.getLocalPath(path, makePose(0.0, 0.0, 0.0, "odom", seconds), "odom"),
      tf2::ExtrapolationException);
  }
}

TEST_F(PathHandlerTest, RejectsZeroRobotTimeWhenTransformIsRequired)
{
  addTransform("map", "odom", 10.0, 0.0);
  const auto path = makePath({makePose(15.0, 0.0, 0.0, "map")}, "map");
  EXPECT_THROW(
    handler_.getLocalPath(path, makePose(0.0, 0.0, 0.0, "odom", 0), "odom"),
    std::invalid_argument);
}

TEST_F(PathHandlerTest, RejectsEmptyPathAndInvalidLength)
{
  const auto robot = makePose(0.0, 0.0);
  EXPECT_THROW(handler_.getLocalPath(makePath({}), robot, "odom"), std::invalid_argument);
  const auto path = makePath({makePose(0.0, 0.0)});
  const std::array<double, 4> invalid{{
    -1.0, std::nan(""), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()}};
  for (double length : invalid) {
    EXPECT_THROW(handler_.getLocalPath(path, robot, "odom", length), std::invalid_argument);
  }
}

TEST_F(PathHandlerTest, RejectsEmptyOrInvalidFrameIdentifiers)
{
  const auto path = makePath({makePose(0.0, 0.0)});
  for (const std::string frame : {"", "/odom"}) {
    auto invalid_path = path;
    invalid_path.header.frame_id = frame;
    EXPECT_THROW(
      handler_.getLocalPath(invalid_path, makePose(0.0, 0.0), "odom"), std::invalid_argument);
    EXPECT_THROW(
      handler_.getLocalPath(path, makePose(0.0, 0.0, 0.0, frame), "odom"), std::invalid_argument);
    EXPECT_THROW(
      handler_.getLocalPath(path, makePose(0.0, 0.0), frame), std::invalid_argument);
  }
}

TEST_F(PathHandlerTest, RejectsConflictingPathPointAndRobotFrames)
{
  EXPECT_THROW(
    handler_.getLocalPath(
      makePath({makePose(0.0, 0.0, 0.0, "map")}), makePose(0.0, 0.0), "odom"),
    std::invalid_argument);
  EXPECT_THROW(
    handler_.getLocalPath(
      makePath({makePose(0.0, 0.0)}), makePose(0.0, 0.0, 0.0, "base_link"), "odom"),
    std::invalid_argument);
}

TEST_F(PathHandlerTest, RejectsInvalidRobotTimestamp)
{
  const auto path = makePath({makePose(0.0, 0.0)});
  auto robot = makePose(0.0, 0.0, 0.0, "odom", -1);
  EXPECT_THROW(handler_.getLocalPath(path, robot, "odom"), std::invalid_argument);
  robot.header.stamp.sec = 15;
  robot.header.stamp.nanosec = 1000000000u;
  EXPECT_THROW(handler_.getLocalPath(path, robot, "odom"), std::invalid_argument);
}

TEST_F(PathHandlerTest, RejectsNonFinitePositionsAndInvalidQuaternions)
{
  const auto good = makePose(0.0, 0.0);
  auto invalid = good;
  invalid.pose.position.x = std::nan("");
  EXPECT_THROW(handler_.getLocalPath(makePath({good}), invalid, "odom"), std::invalid_argument);
  invalid = good;
  invalid.pose.position.z = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    handler_.getLocalPath(makePath({good, invalid}), good, "odom", 0.0), std::invalid_argument);
  invalid = good;
  invalid.pose.orientation.w = 0.0;
  EXPECT_THROW(handler_.getLocalPath(makePath({invalid}), good, "odom"), std::invalid_argument);
  EXPECT_THROW(handler_.getLocalPath(makePath({good}), invalid, "odom"), std::invalid_argument);
  invalid = good;
  invalid.pose.orientation.x = std::nan("");
  EXPECT_THROW(handler_.getLocalPath(makePath({invalid}), good, "odom"), std::invalid_argument);
}

TEST_F(PathHandlerTest, NormalizesNonzeroInputQuaternions)
{
  auto pose = makePose(0.0, 0.0, kPi / 2.0);
  pose.pose.orientation.z *= 2.0;
  pose.pose.orientation.w *= 2.0;
  const auto local = handler_.getLocalPath(makePath({pose}), makePose(0.0, 0.0), "odom");
  ASSERT_EQ(local.size(), 1u);
  expectPose(local.front(), 0.0, 0.0, kPi / 2.0);
}

TEST(PathHandler, RejectsNullBuffer)
{
  EXPECT_THROW(PathHandler{nullptr}, std::invalid_argument);
}


TEST_F(PathHandlerTest, TrueGoalDistanceUsesSameTransformAndNotCroppedTail)
{
  addTransform("map", "odom", 10.0, 20.0, kPi / 2.0);
  const auto path = makePath(
    {makePose(10.0, 20.0, 0.0, "map"), makePose(10.0, 24.0, 0.0, "map")}, "map");
  double distance = -1.0;
  const auto local = handler_.getLocalPath(path, makePose(0.0, 0.0), "odom", 0.1, &distance);
  ASSERT_FALSE(local.empty());
  expectPose(local.back(), 0.1, 0.0, -kPi / 2.0);
  EXPECT_NEAR(distance, 4.0, kTolerance);
}

TEST_F(PathHandlerTest, DoesNotOverwriteGoalDistanceWhenPathConstructionFails)
{
  double distance = 42.0;
  EXPECT_THROW(
    handler_.getLocalPath(makePath({}), makePose(0.0, 0.0), "odom", 2.0, &distance),
    std::invalid_argument);
  EXPECT_DOUBLE_EQ(distance, 42.0);
}


TEST_F(PathHandlerTest, ReturnsTrueGoalPoseUsingSameTfInsteadOfCroppedTail)
{
  addTransform("map", "odom", 10.0, 20.0, kPi / 2.0);
  const auto path = makePath(
    {makePose(10.0, 20.0, 0.0, "map"), makePose(10.0, 24.0, kPi, "map")}, "map");
  Pose2D goal;
  goal.x = 42.0;
  goal.y = 43.0;
  goal.theta = 44.0;
  const auto local = handler_.getLocalPath(
    path, makePose(0.0, 0.0), "odom", 0.1, nullptr, &goal);
  ASSERT_FALSE(local.empty());
  EXPECT_NEAR(local.back().x, 0.1, kTolerance);
  expectPose(goal, 4.0, 0.0, kPi / 2.0);
}

TEST_F(PathHandlerTest, DoesNotOverwriteTrueGoalWhenTfFails)
{
  const auto path = makePath({makePose(10.0, 20.0, 0.0, "map")}, "map");
  Pose2D goal;
  goal.x = 42.0;
  goal.y = 43.0;
  goal.theta = 44.0;
  double distance = 45.0;
  EXPECT_THROW(
    handler_.getLocalPath(
      path, makePose(0.0, 0.0), "odom", 0.1, &distance, &goal), tf2::TransformException);
  expectPose(goal, 42.0, 43.0, 44.0);
  EXPECT_DOUBLE_EQ(distance, 45.0);
}
}  // 匿名命名空间
}  // 命名空间 my_dwa_controller
