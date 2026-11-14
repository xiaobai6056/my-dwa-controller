# 视觉引导 LiDAR 行人状态

融合节点消费 `/scan`、`/sort_prototype/tracks`、相机内参和 TF，向
`/my_dwa/pedestrians` 发布类型化状态。它按图像与 LiDAR 的源时间戳配对，
SORT 墙钟等待上限为 110 ms，最近图像时间差容差为 ±50 ms。等距时选更早图像。
每束回波使用其源时间，编码器插值括号最大 70 ms。时钟回拨会清理动态状态并更新 epoch。

默认位置为截尾质心，状态估计为使用已安装 `config/kf_frozen.json` 的 Joseph KF。
运行时通过包索引解析该配置。位置与速度标记独立，`(epoch, track_id)` 标识融合历史。
SORT ID 供诊断，融合运动历史结合几何和预测残差维持关联。重叠框可用同图像戳、同 epoch
和同框坐标的 IPM 观测辅助消歧。KF 按目标独立推进，执行 NIS 门控和静止死区判断。
控制器对已收敛后验的 0.85 s 缓存同样按目标维护。

## 启动

在已有输入的 ROS 域使用 `ros2 run my_dwa_controller pedestrian_fusion`，
仿真时加 `--ros-args -p use_sim_time:=true`。无参数覆盖时，独立节点和 Gazebo 启动均采用
`position_estimator=trimmed_centroid`、`state_estimator=kf` 和安装后的冻结配置。
两种估计器参数可显式选择历史 AABB/OLS，用于离线或接口回归。

也可以运行 `ros2 launch my_dwa_controller pedestrian_fusion.launch.py`，
该入口使用已有 ROS 话题。设置 `use_sim_time:=true` 可启用仿真时间。

静态场景通过 `ros2 launch my_dwa_controller static_scenario.launch.py scene:=S01` 启动。
`scene` 接受 S01–S06。Nav2 启动后，使用 RViz 或 NavigateToPose action 发送
`scenarios/static/scenarios.yaml` 中的目标。

动态场景模板在 `scenarios/dynamic/`。World 引用 TurtleBot3、GEMS 行人模型及
`libv01_pose.so`，由 Gazebo 的模型与插件搜索路径解析。`visual_prototype/prepare_scene.py`
从 ROS 包索引生成 Waffle 外参，并准备行人模型。Python 依赖和权重路径见
`visual_prototype/requirements.txt`。`MYDWA_VISUAL_ASSETS` 可指向仓库外运行资产目录。
在运行时目录构建好 `.cache/v01-gazebo-build/libv01_pose.so` 后，可用
`ros2 launch visual_prototype/prototype.launch.py world:=scenarios/dynamic/PERSON/v01.world detector:=false`
加载场景。运动时序和目标按同目录的 `config.json`、`trial.json` 接入试验驱动。

可选的 Gazebo 感知录制脚本 `scripts/run_pedestrian_scene.sh --help` 列出 R2、SORT、
LiDAR 运行时目录的配置方式。闭环计时、bag 和逐帧录制保存在仓库外。
