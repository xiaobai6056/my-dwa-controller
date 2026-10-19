#ifndef MY_DWA_CONTROLLER__PATH_HANDLER_HPP_
#define MY_DWA_CONTROLLER__PATH_HANDLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.hpp"
#include "my_dwa_controller/types.hpp"

namespace my_dwa_controller
{

/// 无状态路径适配器，仅持有共享 TF Buffer；全局路径仍由 Controller 保存。
class PathHandler
{
public:
  /// 空 Buffer 抛出 std::invalid_argument，不创建监听器或节点。
  explicit PathHandler(std::shared_ptr<tf2_ros::Buffer> tf);

  /// 按二维导航约定使用 x、y、yaw；返回的全部 Pose2D 均属于 local_frame。
  /// 路径与局部坐标系之间预期为平面 TF，不支持倾斜坐标系的三维路径处理。
  /// robot_pose 的 frame_id 必须等于 local_frame；路径点空 frame_id 继承整体坐标系，
  /// 非空时必须与 global_plan.header.frame_id 一致。坐标系标识不能以斜线开头。
  /// 跨帧时按 robot_pose.header.stamp 查询一次 TF 并复用其逆变换；要求非零时间戳。
  /// 路径视为其声明坐标系中的几何参考，不使用旧路径时间查询 TF，不修改输入消息。
  /// 同帧时不查询 TF，允许机器人时间为零；TF 不可用或过期时传播 tf2 异常。
  /// 空路径、非法帧、非有限位置或四元数、零四元数、非法机器人时间戳以及
  /// 非有限或负的 forward_length 均抛出 std::invalid_argument。
  /// 非零四元数会先归一化；forward_length 为 0 时仅返回最近投影点。
  /// 长度单位为米，末端可返回较短路径；不保证所有参考点均在 Costmap 边界内。
  /// Optional goal distance uses the true global plan endpoint and this same TF snapshot,
  /// not the cropped local tail. Optional goal_in_local includes final yaw in local_frame.
  /// Outputs are written only after successful path and goal construction.
  std::vector<Pose2D> getLocalPath(
    const nav_msgs::msg::Path & global_plan,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const std::string & local_frame, double forward_length = 2.0,
    double * distance_to_goal = nullptr, Pose2D * goal_in_local = nullptr) const;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // 命名空间 my_dwa_controller

#endif  // 头文件保护 MY_DWA_CONTROLLER__PATH_HANDLER_HPP_
