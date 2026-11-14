#include "my_dwa_controller/path_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace my_dwa_controller
{
namespace
{

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

void validatePose(const Pose2D & pose)
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
    throw std::invalid_argument("pose x, y, and theta must be finite");
  }
}

void validatePath(const std::vector<Pose2D> & path)
{
  for (const auto & pose : path) {
    validatePose(pose);
  }
}

Pose2D interpolate(const Pose2D & a, const Pose2D & b, long double t)
{
  if (t == 0.0L) {
    return a;
  }
  if (t == 1.0L) {
    return b;
  }
  Pose2D point;
  point.x = static_cast<double>((1.0L - t) * a.x + t * b.x);
  point.y = static_cast<double>((1.0L - t) * a.y + t * b.y);
  // 朝向沿最短角度差插值，避免跨越正负圆周率时绕行大半圈。
  const long double angle_delta = std::remainder(
    static_cast<long double>(b.theta) - a.theta, 2.0L * std::acos(-1.0L));
  point.theta = static_cast<double>(a.theta + t * angle_delta);
  return point;
}

long double squaredDistance(const Pose2D & a, const Pose2D & b)
{
  const long double dx = static_cast<long double>(a.x) - b.x;
  const long double dy = static_cast<long double>(a.y) - b.y;
  return dx * dx + dy * dy;
}

}  // 匿名命名空间

Pose2D toPose2D(const geometry_msgs::msg::Pose & pose)
{
  const auto normalized = normalizedPose(pose);
  Pose2D result;
  result.x = normalized.position.x;
  result.y = normalized.position.y;
  result.theta = tf2::getYaw(normalized.orientation);
  return result;
}

geometry_msgs::msg::Pose toPose(const Pose2D & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.x;
  result.position.y = pose.y;
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, pose.theta);
  result.orientation = tf2::toMsg(rotation);
  return result;
}

std::optional<PathProjection> projectOntoPath(
  const Pose2D & robot, const std::vector<Pose2D> & path)
{
  validatePose(robot);
  validatePath(path);
  if (path.empty()) {
    return std::nullopt;
  }

  PathProjection nearest{path.front(), 0, 0.0, 0.0};
  long double nearest_distance = squaredDistance(robot, nearest.point);
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto & a = path[i];
    const auto & b = path[i + 1];
    const long double dx = static_cast<long double>(b.x) - a.x;
    const long double dy = static_cast<long double>(b.y) - a.y;
    const long double segment_squared = dx * dx + dy * dy;
    double t = 0.0;
    if (segment_squared > 0.0L) {
      const long double rx = static_cast<long double>(robot.x) - a.x;
      const long double ry = static_cast<long double>(robot.y) - a.y;
      t = static_cast<double>(std::clamp((rx * dx + ry * dy) / segment_squared, 0.0L, 1.0L));
    }
    const auto point = interpolate(a, b, t);
    const long double distance = squaredDistance(robot, point);
    // 首段直接建立投影基准；后续仅在严格更近时替换，等距时保留最早线段。
    if (i == 0 || distance < nearest_distance) {
      nearest = {point, i, t, 0.0};
      nearest_distance = distance;
    }
  }
  if (nearest_distance > std::numeric_limits<double>::max()) {
    throw std::overflow_error("nearest squared distance exceeds double range");
  }
  nearest.squared_distance = static_cast<double>(nearest_distance);
  return nearest;
}

std::optional<PathLookahead> lookAheadOnPath(
  const std::vector<Pose2D> & path, const PathProjection & projection, double length)
{
  if (!std::isfinite(length) || length < 0.0) {
    throw std::invalid_argument("lookahead length must be finite and nonnegative");
  }
  if (!std::isfinite(projection.t) || projection.t < 0.0 || projection.t > 1.0) {
    throw std::invalid_argument("projection parameter t must be finite and within [0, 1]");
  }
  validatePath(path);
  if (path.empty()) {
    return std::nullopt;
  }
  if (path.size() == 1) {
    if (projection.segment_index != 0 || projection.t != 0.0) {
      throw std::invalid_argument("single-point paths require index zero and parameter zero");
    }
    return PathLookahead{path.front(), std::nullopt, 0.0};
  }
  if (projection.segment_index >= path.size() - 1) {
    throw std::invalid_argument("projection segment_index is out of range for the path");
  }

  const auto index = projection.segment_index;
  PathLookahead result{
    interpolate(path[index], path[index + 1], projection.t), std::nullopt, 0.0};
  long double remaining = length;
  long double advanced = 0.0L;
  for (std::size_t i = index + 1; i < path.size(); ++i) {
    const auto start = result.point;
    const auto & end = path[i];
    const long double dx = static_cast<long double>(end.x) - start.x;
    const long double dy = static_cast<long double>(end.y) - start.y;
    const long double segment_length = std::hypot(dx, dy);
    if (segment_length == 0.0L) {
      continue;
    }
    result.forward_heading = static_cast<double>(std::atan2(dy, dx));
    if (remaining < segment_length) {
      result.point = interpolate(start, end, remaining / segment_length);
      result.distance = static_cast<double>(advanced + remaining);
      return result;
    }
    result.point = end;
    remaining -= segment_length;
    advanced += segment_length;
    result.distance = static_cast<double>(advanced);
    if (remaining == 0.0L) {
      return result;
    }
  }

  // At an exhausted projection, look backward only to recover forward geometry.
  // Never point the robot back from its predicted endpoint toward the path end.
  if (!result.forward_heading) {
    for (std::size_t i = index + 1; i > 0; --i) {
      const long double dx = static_cast<long double>(path[i].x) - path[i - 1].x;
      const long double dy = static_cast<long double>(path[i].y) - path[i - 1].y;
      if (dx != 0.0L || dy != 0.0L) {
        result.forward_heading = static_cast<double>(std::atan2(dy, dx));
        break;
      }
    }
  }
  return result;
}

std::optional<double> distanceToPath(const Pose2D & point, const std::vector<Pose2D> & path)
{
  const auto projection = projectOntoPath(point, path);
  if (!projection) {
    return std::nullopt;
  }
  return std::sqrt(projection->squared_distance);
}

std::vector<Pose2D> extractForwardPath(
  const std::vector<Pose2D> & path, const PathProjection & projection, double length)
{
  if (!std::isfinite(length) || length < 0.0) {
    throw std::invalid_argument("extraction length must be finite and nonnegative");
  }
  if (!std::isfinite(projection.t) || projection.t < 0.0 || projection.t > 1.0) {
    throw std::invalid_argument("projection parameter t must be finite and within [0, 1]");
  }
  validatePath(path);
  if (path.empty()) {
    return {};
  }
  if (path.size() == 1) {
    if (projection.segment_index != 0 || projection.t != 0.0) {
      throw std::invalid_argument("single-point paths require segment_index = 0 and t = 0");
    }
    return {path.front()};
  }
  if (projection.segment_index >= path.size() - 1) {
    throw std::invalid_argument("projection segment_index is out of range for the path");
  }

  const auto index = projection.segment_index;
  std::vector<Pose2D> result{interpolate(path[index], path[index + 1], projection.t)};
  long double remaining = length;
  for (std::size_t i = index + 1; i < path.size() && remaining > 0.0L; ++i) {
    // 首段从投影点开始，后续使用原路径端点，保留重复点后的插值基准。
    const auto & start = i == index + 1 ? result.front() : path[i - 1];
    const auto & end = path[i];
    const long double dx = static_cast<long double>(end.x) - start.x;
    const long double dy = static_cast<long double>(end.y) - start.y;
    const long double segment_length = std::hypot(dx, dy);
    if (segment_length == 0.0L) {
      continue;
    }
    if (remaining < segment_length) {
      const auto endpoint = interpolate(start, end, remaining / segment_length);
      // 极小剩余长度可能舍入到已有顶点；不追加坐标完全相同的点。
      if (endpoint.x != result.back().x || endpoint.y != result.back().y) {
        result.push_back(endpoint);
      }
      break;
    }
    result.push_back(end);
    remaining -= segment_length;
  }
  return result;
}

}  // 命名空间 my_dwa_controller
