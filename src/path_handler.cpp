#include "my_dwa_controller/path_handler.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "my_dwa_controller/path_geometry.hpp"

namespace my_dwa_controller
{
namespace
{

void validateFrame(const std::string & frame)
{
  if (frame.empty() || frame.front() == '/') {
    throw std::invalid_argument("frame_id must be nonempty and must not start with '/'");
  }
}

geometry_msgs::msg::Pose normalizedPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  for (double component : {p.x, p.y, p.z, q.x, q.y, q.z, q.w}) {
    if (!std::isfinite(component)) {
      throw std::invalid_argument("pose position and quaternion components must be finite");
    }
  }
  tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
  if (!std::isfinite(rotation.length2()) || rotation.length2() == 0.0) {
    throw std::invalid_argument("pose quaternion must have a finite nonzero norm");
  }
  auto result = pose;
  result.orientation = tf2::toMsg(rotation.normalized());
  return result;
}

}  // 匿名命名空间

PathHandler::PathHandler(std::shared_ptr<tf2_ros::Buffer> tf)
: tf_(std::move(tf))
{
  if (!tf_) {
    throw std::invalid_argument("TF buffer must not be null");
  }
}

std::vector<Pose2D> PathHandler::getLocalPath(
  const nav_msgs::msg::Path & global_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::string & local_frame, double forward_length, double * distance_to_goal,
  Pose2D * goal_in_local) const
{
  if (global_plan.poses.empty()) {
    throw std::invalid_argument("global plan must not be empty");
  }
  validateFrame(global_plan.header.frame_id);
  validateFrame(robot_pose.header.frame_id);
  validateFrame(local_frame);
  if (robot_pose.header.frame_id != local_frame) {
    throw std::invalid_argument("robot pose frame must match local_frame");
  }
  if (!std::isfinite(forward_length) || forward_length < 0.0) {
    throw std::invalid_argument("forward_length must be finite and nonnegative");
  }
  const auto & stamp = robot_pose.header.stamp;
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {
    throw std::invalid_argument("robot pose timestamp is invalid");
  }
  auto robot_in_global = normalizedPose(robot_pose.pose);
  std::vector<Pose2D> path;
  path.reserve(global_plan.poses.size());
  for (const auto & pose : global_plan.poses) {
    if (!pose.header.frame_id.empty() && pose.header.frame_id != global_plan.header.frame_id) {
      throw std::invalid_argument("path pose frame does not match global plan frame");
    }
    path.push_back(toPose2D(pose.pose));
  }

  const bool needs_transform = global_plan.header.frame_id != local_frame;
  geometry_msgs::msg::TransformStamped local_from_global;
  if (needs_transform) {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      throw std::invalid_argument("robot pose timestamp must be nonzero for TF");
    }
    // 使用当前机器人时间，不使用旧路径时间，也不回退到最新 TF。
    const auto global_from_local = tf_->lookupTransform(
      global_plan.header.frame_id, local_frame, tf2_ros::fromMsg(stamp));
    geometry_msgs::msg::Pose transformed_robot;
    tf2::doTransform(robot_in_global, transformed_robot, global_from_local);
    robot_in_global = transformed_robot;

    // 同一次 TF 查询的逆变换供所有截取点共用，避免周期内两次查询得到不同快照。
    tf2::Transform transform;
    tf2::fromMsg(global_from_local.transform, transform);
    local_from_global.header.frame_id = local_frame;
    local_from_global.header.stamp = stamp;
    local_from_global.child_frame_id = global_plan.header.frame_id;
    local_from_global.transform = tf2::toMsg(transform.inverse());
  }

  const auto projection = projectOntoPath(toPose2D(robot_in_global), path);
  if (!projection) {
    throw std::invalid_argument("global plan has no valid projection");
  }
  auto local_path = extractForwardPath(path, *projection, forward_length);
  if (needs_transform) {
    for (auto & pose : local_path) {
      geometry_msgs::msg::Pose transformed;
      tf2::doTransform(toPose(pose), transformed, local_from_global);
      pose = toPose2D(transformed);
    }
  }
  Pose2D local_goal = path.back();
  if (goal_in_local && needs_transform) {
    geometry_msgs::msg::Pose transformed_goal;
    tf2::doTransform(toPose(local_goal), transformed_goal, local_from_global);
    local_goal = toPose2D(transformed_goal);
  }
  if (goal_in_local) {
    *goal_in_local = local_goal;
  }
  if (distance_to_goal) {
    *distance_to_goal = std::hypot(
      robot_in_global.position.x - path.back().x,
      robot_in_global.position.y - path.back().y);
  }
  return local_path;
}

}  // 命名空间 my_dwa_controller
