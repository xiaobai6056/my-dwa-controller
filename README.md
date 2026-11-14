# MyDWA

MyDWA 是 ROS 2 Humble / Nav2 的 DWA 局部控制器，通过 `nav2_core::Controller`
接入。控制器在速度和加速度约束内采样，检查常规与制动轨迹，结合路径、航向、目标、
速度及障碍物代价选择指令。

正式配置消费视觉引导 LiDAR 融合发布的多行人状态。每个行人使用独立的 Joseph KF，
已收敛缓存最多外推 0.85 s。行人硬占用圆淘汰冲突轨迹，速度自适应增长区参与软代价评分。
近距预测丢失后，控制器在 0.60 s 内以 0.12 m/s 为采样限速目标，保留制动约束，线速度非负。

## Release 构建和测试

首先安装 ROS 2 Humble、Nav2 和 `vision_msgs`。本项目的 Gazebo 实验复用的
`sort_ros2_prototype_20261112/environment.sh` 也可提供 `vision_msgs` 环境。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ctest --test-dir build/my_dwa_controller --output-on-failure
```

`config/nav2_params.yaml` 以 `my_dwa_controller::MyDWAController` 为 FollowPath，
默认路径权重 0.22、进度时限 18 s、角速度采样 40，启用软代价和丢失目标限速。
`config/nav2_params_dwb.yaml` 提供 DWB 参数。

## 运行入口

已有扫描、相机、SORT 和 TF 输入时，通过 `ros2 run my_dwa_controller pedestrian_fusion`
启动融合节点。随包安装的 `pedestrian_scene.launch.py` 复用场景、检测和 SORT 运行时。
启动参数和接口见 [融合说明](docs/PEDESTRIAN_FUSION.md)。

本机标准验收在 `~/ros2_experiments/final_mydwa/`。该目录包含六个静态场景、
PERSON、far、same-direction、M1、M2，运行工具和验收归档。
