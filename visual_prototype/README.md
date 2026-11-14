# MyDWA 视觉行人检测原型

V01 动态单人场景的 GUI 启动、时间线和实测截图见 [V01.md](V01.md)。

在独立 Gazebo Classic 场景中，用 TurtleBot3 Waffle 的 RGB 相机和 COCO 预训练 YOLOv8n 检测一个行人。每个场景只加载一个人形实例。ROS2 Python 节点订阅相机图像，每帧输出 person 框、置信度与标注画面。源码和实验均在本目录，运行时使用 ROS Humble 系统安装作为 underlay。实测结论见 [RESULTS.md](RESULTS.md)。

## 从空环境准备并检查

命令在 WSL Ubuntu 22.04 终端执行。已有 ROS2 Humble、Gazebo Classic 11、TurtleBot3 仿真、`gazebo_ros`、`cv_bridge`、`robot_state_publisher` 和 `rqt_image_view` 的环境可直接运行下面的命令。缺少对应 ROS 包时可用 apt 安装 `ros-humble-turtlebot3-gazebo ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge ros-humble-robot-state-publisher ros-humble-rqt-image-view`。WSLg 或其他可用 X11 服务为 Gazebo 相机提供 OpenGL 上下文。

```bash
cd ~/ros2_ws/src/my_dwa_controller/visual_prototype
source /opt/ros/humble/setup.bash
mkdir -p .cache/ultralytics weights
export YOLO_CONFIG_DIR="$PWD/.cache/ultralytics" PIP_CACHE_DIR="$PWD/.cache/pip-cache"

# 使用 ROS Humble 对应的系统 Python，继承 ROS 和 apt 的 Python 模块。
python3 -m venv --system-site-packages --without-pip .venv
curl -fLsS --retry 2 https://bootstrap.pypa.io/pip/pip.pyz -o .cache/pip.pyz
.venv/bin/python .cache/pip.pyz install \
  'torch==2.5.1+cu124' 'torchvision==0.20.1+cu124' \
  --index-url https://download.pytorch.org/whl/cu124
.venv/bin/python .cache/pip.pyz install \
  'numpy==1.26.4' 'opencv-python==4.11.0.86' 'ultralytics==8.3.230'

curl -fLsS --retry 2 \
  https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt \
  -o weights/yolov8n.pt
printf '%s  %s\n' f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36 \
  weights/yolov8n.pt | sha256sum -c -
curl -fLsS --retry 2 https://ultralytics.com/images/bus.jpg -o .cache/bus.jpg
.venv/bin/python - <<'PY'
import torch
from ultralytics import YOLO
assert torch.cuda.is_available()
print(torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0))
model = YOLO("weights/yolov8n.pt", task="detect")
result = model.predict(".cache/bus.jpg", classes=[0], conf=0.25, imgsz=640,
                       device="cuda:0", verbose=False)[0]
assert result.boxes.data.is_cuda and set(result.boxes.cls.tolist()) == {0}
print("person", result.boxes.conf.tolist(), result.speed)
PY

# 下载两个候选各自的依赖，生成本地 SDF、URDF 和 world。
python3 prepare_scene.py
export GAZEBO_MODEL_PATH="$PWD/models:/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models"
export GAZEBO_MODEL_DATABASE_URI=""
gz sdf -k .cache/gems.world
gz sdf -k .cache/subject.world
check_urdf .cache/waffle.urdf
python3 -m py_compile *.py
bash -n run_matrix.sh
```

在纯 CPU 环境中，PyTorch 安装命令可改用 `torch==2.5.1+cpu`、`torchvision==0.20.1+cpu` 和索引 `https://download.pytorch.org/whl/cpu`，本地验证相应使用 `device="cpu"` 并检查输出张量在 CPU。原型使用 Python 源码入口，构建步骤是生成本地仿真资源，无需 colcon 安装。模型资源、权重、虚拟环境和运行输出均由本目录的 `.gitignore` 排除。

## 启动单人场景并实时查看

终端 1 在视觉原型目录中执行。用 `model:=subject` 启动另一个候选，切换前先停止当前场景。

```bash
cd ~/ros2_ws/src/my_dwa_controller/visual_prototype
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11386 LIBGL_ALWAYS_SOFTWARE=1
export YOLO_CONFIG_DIR="$PWD/.cache/ultralytics" OMP_NUM_THREADS=2
mkdir -p "$YOLO_CONFIG_DIR"
ros2 launch "$PWD/prototype.launch.py" model:=gems device:=cuda:0 conf:=0.25 imgsz:=640
```

终端 2 使用同一个 ROS 域。打开图像查看器后选择 `/visual_prototype/annotated`。用 `gui:=true` 启动 Gazebo 客户端，用 `detector:=false` 只运行相机和仿真。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1
ros2 run rqt_image_view rqt_image_view
```

另一个终端可用同一 ROS 域检查接口。

```bash
ros2 topic list -t
ros2 topic echo /visual_prototype/camera/camera_info --once
ros2 topic echo /visual_prototype/detections --once
ros2 run tf2_ros tf2_echo base_link camera_rgb_optical_frame
```

独立检测节点可接其他 RGB 图像源。参数在启动时读取，修改后重启节点。可分别调整 `conf`（大于 0 且不超过 1）、`imgsz`（32 的正整数倍）、`device`（例如 `cuda:0` 或 `cpu`）、`weights` 和 `image_topic`。例如在 `detector:=false` 的场景中手动启动：

```bash
.venv/bin/python person_detector.py --ros-args -p use_sim_time:=true \
  -p image_topic:=/visual_prototype/camera/image_raw \
  -p "weights:=$PWD/weights/yolov8n.pt" -p device:=cuda:0 -p conf:=0.35 -p imgsz:=512
```

## 相机、TF 和检测接口

基线脚本使用 `nav2_bringup` 的 `worlds/waffle.model`。对该系统模型单独实测，深度传感器同时发布 `/intel_realsense_r200_depth/image_raw`（320×240、`rgb8`）和 `/intel_realsense_r200_depth/camera_info`，SDF 配置为 5 Hz，消息帧是 `camera_depth_frame`。原型复用 `turtlebot3_gazebo` 提供的同型号 Waffle SDF、专用 RGB 相机插件及配套 URDF。该 SDF 的相机原配置为 1920×1080、30 Hz。图像调整到 640×480、15 Hz 仿真时间、水平 FOV 1.57 rad，光轴上仰 0.445 rad，使离低位相机 1 m 的站立成人尽量完整入画。相机光心在 `base_link` 下为 `(0.069, -0.047, 0.107)` m。保留系统相机的高斯噪声配置。

TF 链为 `base_footprint → base_link → camera_link → camera_rgb_frame → camera_rgb_optical_frame`。末级固定旋转为 RPY `(-π/2, 0, -π/2)`，光学帧 +x 向右、+y 向下、+z 向前。图像、CameraInfo 和检测都使用同一光学帧。检测和标注复用输入图像的 ROS 时间戳。

| Topic | 类型和内容 |
| --- | --- |
| `/visual_prototype/camera/image_raw` | `sensor_msgs/msg/Image`，`rgb8` |
| `/visual_prototype/camera/camera_info` | `sensor_msgs/msg/CameraInfo`，含针孔内参 |
| `/visual_prototype/detections` | 可导入 `vision_msgs` 时使用 `Detection2DArray`，类 ID 为 COCO 字符串 `"0"`。当前环境默认发布 `std_msgs/msg/String` JSON |
| `/visual_prototype/annotated` | `sensor_msgs/msg/Image`，标注后的彩色图像，`rgb8` |
| `/visual_prototype/metrics` | `std_msgs/msg/String` JSON，含时间戳、帧名、`predict_ms`、`forward_ms` 和 `boxes`，与回退检测消息内容一致 |

JSON 中的 `xyxy` 是输入图像上的像素坐标 `[x_min, y_min, x_max, y_max]`，原点在左上。阈值以下或无检测的帧发布空框数组和正常图像。图像订阅与标注发布采用 SensorData QoS。订阅回调保留最新一帧，单独的工作线程负责推理。过载时允许覆盖排队帧，每 5 秒日志报告处理率、耗时、检出帧、覆盖帧和异常数。MyDWA 进程无需订阅或等待视觉结果。

## 资产来源与本地文件

| 资产 | 来源和许可 | 视觉文件与尺寸 |
| --- | --- | --- |
| 3DGEMS `human_female_1` | 3DGEMS 官网按类别发布归档。选用明确标注来自 3DGEMS 的镜像，固定到 `12ff07b6ad6e7e6031b8227854dffeb6f02dffa0`，逐文件下载该人物。3DGEMS 声明多数 mesh 来自 3D Warehouse，此人物的原作者及单独许可未确认，按本地实验资产处理。 [3DGEMS](https://data.nvision2.eecs.yorku.ca/3DGEMS/) · [镜像来源声明](https://github.com/npu-ius-lab/Gazebo-Models) | `models/human_female_1/` 包含上游 SDF、model.config、13 个可见 Collada mesh、皮肤和头发的 2 个 OGRE 材质脚本及 2 张 JPG。保留服装颜色和纹理。用 Gazebo MeshManager 量得局部包围盒约 0.675 × 0.307 × 1.660 m。 |
| gbionics Human Subject with Meshes | 固定到 `223146b8ab48fc43c376fe0678e24c59090fa279`。仓库 LICENSE 文本为 LGPL 2.1，项目 README 对该人体的 mesh 单独注明 CC BY-SA 2.0。已保留 LICENSE 与原 README。 [上游模型](https://github.com/gbionics/human-gazebo/tree/223146b8ab48fc43c376fe0678e24c59090fa279/humanSubjectWithMeshes) · [许可声明](https://github.com/gbionics/human-gazebo/blob/223146b8ab48fc43c376fe0678e24c59090fa279/README.md#human-subject-with-meshes) | `models/human_subject_with_meshes/` 含上游 URDF、51 个 STL 视觉 mesh、SDF 和 model.config。STL 使用上游 URDF 肤色，双肩各收至离竖直方向约 15°，视觉 mesh 贴地摆放。变换后的局部包围盒约 0.325 × 0.832 × 1.695 m。 |
| YOLOv8n | Ultralytics 官方 COCO detection 权重，取自 assets v8.3.0。Ultralytics 软件与模型按其 AGPL-3.0 / Enterprise 双许可提供，本地实验使用官方公开权重。 [YOLOv8 文档](https://docs.ultralytics.com/models/yolov8/) · [权重](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt) · [Ultralytics 许可](https://www.ultralytics.com/license) | `weights/yolov8n.pt`，6,549,796 字节。环境使用 Ultralytics 8.3.230。 |

两个 SDF 都封装为单个静态视觉实体，保留原 mesh。3DGEMS 的原始腰带 URI 指向另一个人物目录，已改为当前人物的同哈希腰带。解剖模型将 URDF 的连杆与姿态折算到一个静态 SDF link，原材质色和 mesh 缩放仍在。平移通过 Gazebo 服务设置整个人形位姿，模型不需要关节控制或行走动画。

## 复跑对照实验

```bash
cd ~/ros2_ws/src/my_dwa_controller/visual_prototype
bash run_matrix.sh "runs/run-$(date +%Y%m%d-%H%M%S)"
# 可选参数示例，取一个新的输出目录。
VISUAL_DEVICE=cpu VISUAL_CONF=0.35 VISUAL_IMGSZ=512 \
  bash run_matrix.sh "runs/cpu-$(date +%Y%m%d-%H%M%S)"
```

脚本依次运行两个独立场景，使用 ROS 域 86/87 和 Gazebo 端口 11386/11387。端口冲突时可设置 `VISUAL_DOMAIN_BASE` 和 `VISUAL_PORT_BASE`。如需对手动启动的单个场景采样，在相同 ROS 域执行 `python3 sweep.py gems runs/gems-manual`，模型参数和运行场景一致。脚本保留每帧 JSONL、统计 CSV、相机接口记录，以及匹配同一时间戳的原始与标注 PNG。
