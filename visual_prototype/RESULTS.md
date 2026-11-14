# Gazebo 行人检测实测

运行于 2026-11-11，使用 `runs/gpu-final-20261111/` 的原始记录。启动和资产来源见 [README.md](README.md)。

## 运行环境

- WSL2 Ubuntu 22.04.5 LTS，ROS2 Humble，Gazebo Classic 11.10.2，TurtleBot3 Gazebo 2.3.8。CPU 为 AMD Ryzen 9 7940HX，GPU 为 **NVIDIA GeForce RTX 4070 Laptop GPU**，驱动 610.62，显存 8188 MiB。
- Python 3.10.12、NumPy 1.26.4、OpenCV 4.11.0、PyTorch 2.5.1+cu124、torchvision 0.20.1+cu124、Ultralytics 8.3.230。PyTorch CUDA 12.4 可用，实测模型输出张量位于 CUDA，计算能力为 8.9。Gazebo 用 WSLg OpenGL 软件渲染，YOLO 用 `cuda:0`，CPU 线程环境变量 `OMP_NUM_THREADS=2`。
- 相机为 640×480 `rgb8`，仿真时钟目标 15 Hz。两组均用官方 YOLOv8n、`imgsz=640`、`conf=0.25`、`classes=[0]`。本地先对 Ultralytics 样图验证，输出 4 个 person 框，置信度 0.866、0.853、0.825、0.261，预热后模型前向中位数 5.66 ms。 [环境记录](runs/gpu-final-20261111/environment.txt) · [CUDA 验证](runs/gpu-final-20261111/local-gpu-smoke.log) · [本地样图标注](runs/gpu-final-20261111/bus-person-gpu.jpg)

## 静止距离和视角

距离是相机光心水平投影到行人模型原点的纵向间距，机器人静止。模型依次设为正面、侧面（90°）、斜向（45°）。每次设置位姿后等待 0.45 s，再采集 15 个时间戳匹配的原图、标注图与检测结果。两次 Gazebo 运行分别验证了世界里只有一个名为 `pedestrian` 的人体。

每格依次为 **有 person 框的帧数 / 采样数 · 检出帧最高置信度的中位数 · 相应框宽×高的中位数（px）**。一个输出框可能只覆盖人体的一部分。框位置、连续漏检与全部候选框见原始记录。

| 距离 | 视角 | 3DGEMS | gbionics |
| --- | --- | --- | --- |
| 1 m | 正面 | 15/15 · 0.85 · 176×430 | 3/15 · 0.36 · 132×55 |
| 1 m | 侧面 | 15/15 · 0.80 · 116×447 | 0/15 · — |
| 1 m | 斜向 | 15/15 · 0.85 · 155×436 | 15/15 · 0.57 · 150×418 |
| 2 m | 正面 | 15/15 · 0.92 · 100×252 | 15/15 · 0.73 · 126×257 |
| 2 m | 侧面 | 15/15 · 0.82 · 53×257 | 15/15 · 0.75 · 57×258 |
| 2 m | 斜向 | 15/15 · 0.89 · 77×257 | 15/15 · 0.92 · 98×256 |
| 3 m | 正面 | 15/15 · 0.89 · 72×183 | 14/15 · 0.46 · 91×183 |
| 3 m | 侧面 | 15/15 · 0.76 · 37×183 | 15/15 · 0.65 · 38×184 |
| 3 m | 斜向 | 15/15 · 0.83 · 50×184 | 15/15 · 0.90 · 67×184 |
| 4 m | 正面 | 15/15 · 0.91 · 53×141 | 15/15 · 0.81 · 66×142 |
| 4 m | 侧面 | 15/15 · 0.83 · 24×141 | 15/15 · 0.52 · 29×142 |
| 4 m | 斜向 | 15/15 · 0.85 · 38×142 | 15/15 · 0.91 · 52×144 |

## 2 m 横向平移

模型保持正面，沿 Gazebo 世界 y 轴平移 0.90 m。18 次位姿设置将模型从相机横向 −0.45 m 移到 +0.45 m，记录了服务返回后的仿真时间和确认的位置。相机始终固定。每步等待约 0.14 s，屏幕上的人体从右向左移动。

| 模型 | 有框 / 采样 | 漏检 | 最长连续漏检 | 最高置信度中位数 | 匹配输出 Hz | 平均 predict ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 3DGEMS human_female_1 | 35/35 | 0 | 0 | 0.9196 | 13.8 | 17.0 |
| gbionics Human Subject with Meshes | 28/31 | 3 | 1 | 0.6235 | 12.7 | 23.7 |

## 实时性能

`predict_ms` 从调用 YOLO predict 到检测框转换回主机数据，包含预处理、模型前向、后处理和取框。它在绘制与 ROS 发布之前结束。`forward_ms` 是 Ultralytics 的模型推理计时。匹配输出频率由每个条件首尾匹配结果的墙钟时间计算，受仿真帧率、渲染和 ROS 传输影响。表中耗时汇总包含静止与平移采样，启动预热帧另在日志中。

| 模型 | predict 平均 / 中位 / P95 (ms) | forward 平均 (ms) | 每条件墙钟 Hz 中位数（最小–最大） |
| --- | ---: | ---: | ---: |
| 3DGEMS human_female_1 | 14.3 / 12.8 / 23.8 | 9.4 | 14.8（12.3–15.5） |
| gbionics Human Subject with Meshes | 15.5 / 12.6 / 26.7 | 11.0 | 12.4（10.3–15.0） |

## 模型选择和限制

- 3DGEMS 带头发、皮肤材质和服装，静止 180/180 帧有框，平移 35/35。侧面在 4 m 只剩约 24 px 宽，仍持续给出 person 框。对于本次原型，3DGEMS 的相机检测表现更稳定，单模型许可仍待核实。
- gbionics 的头部、躯干和四肢均可见，统一肤色和关节接缝接近解剖模型。静止 152/180 帧有框。1 m 正面的 3 个框中，2 个只框住脚，1 个覆盖大部分人体且置信度约 0.26。1 m 侧面连续 15 帧漏检。28 帧对同一个行人输出了两个 person 框，主要出现在 1m-oblique, 3m-side。
- 每个条件只在同一固定光照、背景和相机几何下短时采样。未标注逐帧真值，未计算 IoU、精确率、召回率或 mAP。这里的有框率只统计实际匹配到的帧，采样和传输缺口单独通过频率与节点日志判断。未测试真实机器人、不同环境、遮挡或与 MyDWA 同时运行时的资源竞争。
- 旧基线的 Waffle 同时提供 320×240 彩色图和深度图，CameraInfo 与图像头使用 `camera_depth_frame`。原型复用了 TurtleBot3 同型号 SDF 的专用 RGB 相机。两组都实测了 RGB 相机的 CameraInfo、光学 TF 与逐帧匹配；内参 fx=fy=320.255，cx=320.5，cy=240.5，D 全为零。 [基线相机](runs/gpu-final-20261111/baseline-camera.json) · [基线 TF](runs/gpu-final-20261111/baseline-camera-tf.txt)
- 当前安装没有 `vision_msgs`，主矩阵使用 JSON 回退。另将 Humble 4.1.1 的 `vision_msgs` 解包到项目缓存，用真实 ROS Topic 端到端验证标准 `Detection2DArray`，包括有框、空数组、时间戳、bbox 与空 ID。 [标准消息测试](runs/gpu-final-20261111/typed-test.log)

## 原图、标注与逐帧记录

- [1 m 侧面对照（含漏检）](runs/gpu-final-20261111/comparison-1m-side.png) · [2 m 正面对照](runs/gpu-final-20261111/comparison-2m-front.png)
- 3DGEMS human_female_1 · [距离、视角、框和耗时](runs/gpu-final-20261111/gems/summary.csv) · [逐帧 JSONL](runs/gpu-final-20261111/gems/frames.jsonl) · [相机与 ROS 接口](runs/gpu-final-20261111/gems/interfaces.json) · [平移轨迹](runs/gpu-final-20261111/gems/motion.csv) · [平移起点原图](runs/gpu-final-20261111/gems/2m-lateral-start-raw.png) · [平移终点标注](runs/gpu-final-20261111/gems/2m-lateral-end-annotated.png) · [节点与 Gazebo 日志](runs/gpu-final-20261111/gems-launch.log)
- gbionics Human Subject with Meshes · [距离、视角、框和耗时](runs/gpu-final-20261111/subject/summary.csv) · [逐帧 JSONL](runs/gpu-final-20261111/subject/frames.jsonl) · [相机与 ROS 接口](runs/gpu-final-20261111/subject/interfaces.json) · [平移轨迹](runs/gpu-final-20261111/subject/motion.csv) · [平移起点原图](runs/gpu-final-20261111/subject/2m-lateral-start-raw.png) · [平移终点标注](runs/gpu-final-20261111/subject/2m-lateral-end-annotated.png) · [节点与 Gazebo 日志](runs/gpu-final-20261111/subject-launch.log)

每个模型的目录包含 12 组静态条件的原图和标注图，以及平移开始、中段和末段的三组图。静态截图固定取匹配到的第 8 帧，漏检场景照常保存。
