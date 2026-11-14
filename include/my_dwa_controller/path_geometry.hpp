#ifndef MY_DWA_CONTROLLER__PATH_GEOMETRY_HPP_
#define MY_DWA_CONTROLLER__PATH_GEOMETRY_HPP_

#include <cstddef>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "my_dwa_controller/types.hpp"

namespace my_dwa_controller
{

/// 提取平面位置和 yaw；位置及四元数分量必须有限，四元数模长必须有限且非零。
/// 先归一化四元数再提取 yaw，非法输入抛出 std::invalid_argument。
Pose2D toPose2D(const geometry_msgs::msg::Pose & pose);

/// 将平面位姿转换为 z=0 的 Pose，使用 theta 构造仅含 yaw 的四元数。
geometry_msgs::msg::Pose toPose(const Pose2D & pose);

struct PathProjection
{
  Pose2D point{};
  std::size_t segment_index = 0;
  double t = 0.0;
  double squared_distance = 0.0;
};

/// 查找机器人位置到整条折线的最近投影，只使用 x、y 计算距离。
/// 空路径返回 std::nullopt；单点路径约定 segment_index=0、t=0。
/// 零长度线段按起点处理，非零线段的 t 限制在 [0,1]；等距时选择最早线段。
/// 机器人和路径的 x、y、theta 必须有限，否则抛出 std::invalid_argument。
/// 生成点的 theta 沿最短角度差插值，保留端点原值，不强制归一化，也不影响最近距离。
/// 最小距离平方超出 double 可表示范围时，抛出 std::overflow_error。
std::optional<PathProjection> projectOntoPath(
  const Pose2D & robot, const std::vector<Pose2D> & path);

/// Allocation-free lookahead result. Heading is a geometric forward tangent,
/// never the stored path yaw. distance is the actual advanced arc length.
struct PathLookahead
{
  Pose2D point{};
  std::optional<double> forward_heading;
  double distance = 0.0;
};

/// Same projection/length contract as extractForwardPath; empty path returns nullopt.
/// Clamps at the path end and retains the last nonzero forward tangent there.
std::optional<PathLookahead> lookAheadOnPath(
  const std::vector<Pose2D> & path, const PathProjection & projection, double length);

/// 返回点到整条折线的最短欧氏距离，仅复用最近投影结果的 squared_distance 开平方。
/// 空路径返回 std::nullopt；非法输入和距离平方溢出异常沿用 projectOntoPath 的约定。
std::optional<double> distanceToPath(const Pose2D & point, const std::vector<Pose2D> & path);

/// 从投影位置沿原路径前向截取 length 指定的弧长，不足时截取至原路径终点。
/// 投影必须对应同一条未修改路径；仅使用 segment_index、t 重建截取起点，
/// point 和 squared_distance 只提供查询信息，不作为截取定位依据。
/// 空路径返回空集合；长度为零、单点路径或投影在末端时，仅返回起点。
/// 连续相同坐标只保留一个点；原有中间点保留，生成点的 theta 沿最短角度差插值。
/// 路径分量必须有限，length 必须为有限非负数，t 必须有限且位于 [0,1]；
/// 非空路径中索引必须有效，单点路径要求索引为 0 且 t 为 0。
/// 上述非法输入均抛出 std::invalid_argument；本函数不修改原路径。
std::vector<Pose2D> extractForwardPath(
  const std::vector<Pose2D> & path, const PathProjection & projection, double length);

}  // 命名空间 my_dwa_controller

#endif  // 头文件保护 MY_DWA_CONTROLLER__PATH_GEOMETRY_HPP_
