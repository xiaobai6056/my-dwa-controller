// Diagnostic only: reuse the existing fixture and installed, unchanged plugins.
#include "../../../test/test_controller.cpp"

namespace my_dwa_controller
{
namespace
{
TEST_F(ControllerTest, AuditTerminalLatchAgainstResetChecker)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", MY_DWA_TEST_PARAMS_FILE});
  options.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
  tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  configure();
  setGoal(0.1, 0.0, 0.6);
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  auto checker = loader.createSharedInstance("nav2_controller::SimpleGoalChecker");
  checker->initialize(node_, "general_goal_checker", costmap_);
  const auto goal = makePose(0.1, 0.0, 0.6);
  const auto outside = makePose(-0.2, 0.0, 0.6);

  // Before entering the terminal region, the same free geometry allows translation.
  const auto normal = controller_->computeVelocityCommands(outside, stopped_, checker.get());
  EXPECT_GT(normal.twist.linear.x, 0.0);

  const auto inside = makePose(0.0, 0.0, 0.0);
  const auto entry = controller_->computeVelocityCommands(inside, stopped_, checker.get());
  EXPECT_DOUBLE_EQ(entry.twist.linear.x, 0.0);
  EXPECT_FALSE(checker->isGoalReached(inside.pose, goal.pose, stopped_));
  // Stateful checker has latched XY; without reset it accepts this changed pose.
  EXPECT_TRUE(checker->isGoalReached(outside.pose, goal.pose, stopped_));

  // Match Nav2 1.1.20 setPlannerPath order: same endpoint setPlan, then checker reset.
  setGoal(0.1, 0.0, 0.6);
  checker->reset();
  const auto result = controller_->computeVelocityCommands(outside, stopped_, checker.get());
  EXPECT_DOUBLE_EQ(result.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.twist.angular.z, 0.0);
  EXPECT_FALSE(checker->isGoalReached(outside.pose, goal.pose, stopped_));
  std::cout << "[LATCH_PROBE] distance=0.3 tolerance=0.25 normal_v="
            << normal.twist.linear.x << " latched_v=" << result.twist.linear.x
            << " latched_w=" << result.twist.angular.z << " checker_after_reset=0"
            << std::endl;
}
}  // namespace
}  // namespace my_dwa_controller
