# PathHandler 设计与调用约定

## 职责和接口

PathHandler 只持有共享的 tf2_ros::Buffer，不保存路径、进度或其他算法模块。
MyDWAController::setPlan() 继续保存唯一的 global_plan_，本次不修改 Controller。

```cpp
explicit PathHandler(std::shared_ptr<tf2_ros::Buffer> tf);
std::vector<Pose2D> getLocalPath(
  const nav_msgs::msg::Path & global_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::string & local_frame, double forward_length = 2.0) const;
```

复用既有 Pose2D、projectOntoPath 和 extractForwardPath，不复制几何算法。
相较每点查询 TF，一次查询并取逆可保证单次调用使用相同变换，减少查询。
相较接受任意机器人坐标系，第一版检查 robot_pose.header.frame_id == local_frame，明确拒绝调用方传错坐标系。

## 消息和时间

路径整体 frame_id、机器人 frame_id、local_frame 必须非空且不以斜线开头；路径点非空的 frame_id 必须与整体一致，空值继承整体。
输入位置和四元数分量必须有限，四元数不能为零；归一化四元数后提取 yaw。按二维导航约定使用 x、y、yaw。
坐标系之间预期为二维平面 TF；不支持含 roll/pitch 的倾斜坐标系三维路径处理。
forward_length 必须为有限非负数；0 返回最近投影点。空路径报错，单点路径返回该点，接近末端允许短路径。

跨帧时，机器人位姿必须有合法非零时间戳；零时间戳不能被隐式解释为最新 TF。
按该时间查询 global_frame <- local_frame 一次，将机器人转换到路径坐标系；投影、截取后用同一变换的逆转换回 local_frame。
全局路径视为其声明坐标系中的几何参考，允许生成时间和各路径点时间早于当前周期；不用这些旧时间查询 TF，不修改输入消息。
不采用 latest 回退、时间外推容差或等待重试；TF 不可用或时间超出缓存时传播 tf2::TransformException。
同帧时直接做几何处理，完全不查询 TF，可接受零机器人时间戳。
错误输入抛出 std::invalid_argument；几何工具异常按原约定传播。

## 朝向插值修正

现有 3 -> -3 rad 的线性插值在中点得到 0，方向反转，影响返回朝向。
仅修改 interpolate 中的 theta：使用 remainder(b.theta-a.theta, 2*pi) 得到最短角度差，再插值；保持端点原值，不强制归一化最终角度。
位置、最近投影、截取长度和评分器的距离行为不变。同步修改旧朝向测试并补充正反向跨界和截取端点测试。

## 截取距离

默认 2.0 m 可通过函数参数覆盖，不增加重复配置对象。
当前 YAML 的 max_vel_x=0.26、sim_time=1.7，预测行驶距离上限约 0.442 m；采样器默认 max_v=0.22。
局部地图为 3 x 3 m，机器人居中时直线轴向边界约 1.5 m，2 m 参考段可能超出地图，但参考段不是候选轨迹。
模块没有地图边界输入，也不声称输出全在地图内部；后续 Controller 应结合实际预测设置、机器人位置和地图尺寸配置 forward_length。

## 验证与工程

新增 PIC 静态库 path_handler 和独立 GTest，公开链接 path_geometry 与已安装的消息/TF 库。
CMake/package.xml 声明 nav_msgs、tf2、tf2_ros、tf2_geometry_msgs 及 rclcpp 所需依赖并导出。
测试用 Buffer::setTransform 注入 TF，无需 Gazebo、节点、监听器、网络或 Nav2 启动。
覆盖平移、旋转、任意帧名、同帧空 TF、路径中间/拐角/末端、旧路径时间、动态 TF 插值和时间外推失败、非法帧和消息、单点/重复点及 0 长度。

## 构建、测试和实际结果

在 WSL Bash 中执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select my_dwa_controller --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/my_dwa_controller -R '^test_path_handler$' --output-on-failure
ctest --test-dir build/my_dwa_controller -L gtest --output-on-failure
colcon test --packages-select my_dwa_controller
colcon test-result --test-result-base build/my_dwa_controller/test_results --verbose
# 当前系统的 cppcheck 2.7 默认被 ament 跳过，临时启用后单独检查。
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 ctest --test-dir build/my_dwa_controller -R '^cppcheck$' --output-on-failure
```

2026-10-17 实测：构建成功；PathHandler 20/20、PathGeometry 33/33，全部 6 个算法目标共 127 个用例通过。
cppcheck、lint_cmake、xmllint 通过；本次新增/修改的六份 C++ 文件格式检查通过。
cppcheck 的结果来自临时设置 AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 后的实际执行，未修改系统或工程默认环境。
全包 uncrustify 仍报告未修改的 include/my_dwa_controller/controller.hpp 与 src/controller.cpp 两个文件的既有格式问题。
另建临时独立 consumer，仅 find_package(my_dwa_controller REQUIRED) 并链接 my_dwa_controller::path_handler，编译、链接和运行成功，验证安装后的依赖传递；未在源码中保留临时 consumer。
只读代码复核未发现阻断问题。最短角度插值的回归测试先在旧实现下失败，再在新实现下通过。

## 后续 Controller 调用示例

下面是后续接入示意，本次没有修改 Controller。
配置阶段用已有 tf_ 构造成员 path_handler_，从 Controller 参数读取 forward_length_（初始值可取 2.0 m），不要在 PathHandler 中再次保存 global_plan_。

```cpp
// 配置阶段：path_handler_ 的类型为 std::unique_ptr<PathHandler>。
path_handler_ = std::make_unique<my_dwa_controller::PathHandler>(tf_);

// 后续控制周期：pose 为 Nav2 传入的当前机器人位姿。
const auto local_frame = costmap_ros_->getGlobalFrameID();
const auto local_path = path_handler_->getLocalPath(
  global_plan_, pose, local_frame, forward_length_);
```

调用前确保 pose.header.frame_id == local_frame，跨帧时 pose 带当前非零时间戳。
调用方负责捕获并按 Nav2 Controller 约定处理 std::invalid_argument、tf2::TransformException 等失败；不能在异常后继续使用本次局部路径。
同包 Controller 目标后续链接 path_handler；其他包链接导出的 my_dwa_controller::path_handler。
该模块只返回参考路径，不调度 Predictor、ObstacleEvaluator 或 TrajectoryScorer。
