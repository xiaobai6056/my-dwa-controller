离线回归使用第三轮 D01 2s 的原始录制，原 bag 保持只读。

在 WSL Ubuntu-22.04 中执行（工作目录 `/home/user/ros2_ws/src/my_dwa_controller`）：

```bash
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
# 以下三个命令重新生成新实验目录内的派生文件，不发布 ROS 数据。
python3 benchmark/scan_emergency_20261022/diagnose.py
python3 benchmark/scan_emergency_20261022/export_replay.py
cd /home/user/ros2_ws
colcon --log-base log_scan_emergency build --packages-select my_dwa_controller \
  --build-base build_scan_emergency --install-base install_scan_emergency
source install_scan_emergency/setup.bash
cd src/my_dwa_controller
/home/user/ros2_ws/build_scan_emergency/my_dwa_controller/replay_scan_emergency \
  benchmark/scan_emergency_20261022/prior_replay.input \
  benchmark/scan_emergency_20261022/replay_final
python3 benchmark/scan_emergency_20261022/test_offline.py
cd /home/user/ros2_ws/build_scan_emergency/my_dwa_controller
ctest -R '^test_' --output-on-failure
ctest --output-on-failure
```

回放的 408 帧包含扫描原始时间戳、对应 odom←laser TF、记录中的机器人速度、
局部路径和 Costmap。没有 latest-TF 替代；9 帧因缺少可用路径/TF/地图未导出。
旧绘图 TF 插值上限为 0.200 秒，但 AMCL 的真实最大间隔为 0.201 秒；导出工具
仅在本进程将该离线插值上限设为 0.250 秒，不修改旧工具或线上 TF 处理。
OccupancyGrid 的 255/254/253 类别保持，其他膨胀代价从已发布的压缩值近似还原。
回放每 Scan 一次 Governor 决策，机器人运动固定为旧录制；恢复时间不能替代实机闭环。

本次只运行 `round1_S05` 和 `round2_D01_t2`。`run_round.py` 必须使用全新 `--tag`，
保留每次参数副本、源文件快照、插件副本及运行时映射，已有输出目录会拒绝覆盖。
如需另行复现实验，先确认 Gazebo/ROS 资源可用，再使用新的 tag/domain：

```bash
python3 benchmark/scan_emergency_20261022/run_round.py --cases S05 --tag new_check --domain 105
python3 benchmark/scan_emergency_20261022/run_round.py --cases D01_t2 --tag new_check --domain 105
```

这些闭环命令会真正启动 Gazebo；离线工具与 CTest 不启动该 Benchmark。
`sources_before.zip` 保留开始时的未提交源码；`incremental.patch` 仅表示本次增量。
`sources_final.zip` 包含源码/测试/构建配置及原生回放程序，需与本目录配套使用。
正式场景与旧结果未更改，构建和安装不写入原前缀。
