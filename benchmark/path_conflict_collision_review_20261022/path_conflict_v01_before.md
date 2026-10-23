# MyDWA Path Conflict V0.1

这是默认关闭的路径冲突策略原型。模块可构建、可测试；本轮 Gazebo 验证
**尚未证明改善 D01 避撞**。不要将停车距离计算解释为动态安全保证。

## 文件及主流程

- `include/my_dwa_controller/path_conflict_monitor.hpp`、`src/path_conflict_monitor.cpp`：
  `ConflictEvidence`、路径走廊、世界栅格比较和有限历史；不决定速度。
- `include/my_dwa_controller/risk_governor.hpp`、`src/risk_governor.cpp`：
  `RiskDecision`、停车条件、等待和恢复；不读实时地图、不发布指令。
- `controller.hpp/.cpp`：原局部路径和终点策略 → 复制地图 → 检测/风险决策 →
  合并本周期可达速度上限 → 原采样/预测/ObstacleEvaluator/评分/输出。
- `controller_diagnostics.hpp/.cpp`：追加启用时的证据、区域、速度上限和耗时；
  `[PATH_CONFLICT]` 记录不同地图状态及状态转换。
- `CMakeLists.txt`：独立静态库及两个 GTest 目标；`test/test_controller.cpp`
  增加真实插件集成和确定性闭环测试。`.gitignore` 仅追加本轮文件的例外。

直接复用 Costmap2D 的复制、worldToMap/mapToWorld 和现有 path_geometry。
没有另建网格基础设施，没有改变 ObstacleEvaluator、轨迹评分或 Nav2 的
setSpeedLimit()。源地图锁仅覆盖复制；检测、历史比较和候选评估都在锁外，
并读取同一份副本。关闭功能后不运行检测器；原 225 组命令/异常签名保持通过。
关闭时仍使用地图副本，以满足缩短源地图锁范围的要求；不承诺位级相同的线程时序。

## 参数与启用

以下键均在 `controller_server.ros__parameters.FollowPath` 下，configure 时读取，
不提供运行时参数回调。原 `config/nav2_params.yaml` 未修改。

|键|默认值|含义|
|---|---:|---|
|path_conflict.enabled|false|独立开关|
|path_conflict.lookahead|1.5 m|沿实际局部路径的最大前视长度|
|path_conflict.outer_margin|0.33 m|Outer 比 Inner 多出的半宽|
|path_conflict.warning_speed|0.12 m/s|预警线速度目标上限|
|path_conflict.brake_acceleration|0.3 m/s²|停车模型减速度，必须≤acc_lim_x|
|path_conflict.reaction_time|0.3 s|延迟余量；模型另加控制周期|
|path_conflict.stop_margin|0.1 m|冲突前停车余量|
|path_conflict.clear_states|3|解除限制所需不同清晰地图状态数|

Inner 半宽采用 configure 时 Costmap 的 padded circumscribed radius，栅格判定
再增加半个栅格对角线。角加速度、控制周期和停止阈值复用现有设置，不另加参数。
仅识别原始代价254为core；253及其他inflation不作为运动证据。

在启动参数副本的 FollowPath 下加入：

```yaml
path_conflict:
  enabled: true
```

修改节点参数后需要重新 cleanup/configure，单纯 `ros2 param set` 不会更新本模块。
回滚为 `enabled: false` 并重新配置；现有 YAML 缺省即关闭。原工作区安装目录未覆盖，
本轮独立前缀为 `/home/user/ros2_ws/install_path_conflict_v01/my_dwa_controller`。

## 检测与状态转换

当前局部路径已经处于 Costmap 坐标系。Inner 当前 core 占据即路径冲突；
Outer 仅有静态占据不会触发预警。历史图用当前路径统一投影，按世界格心匹配
相同分辨率/对齐格网，只在共同已知区域内判断新增core。
折返路径可能有多个相近段：安全区域起点和未知前缀取最早相交段，区域终点取
最后相交段；运动统计仍采用最近投影，不把后段的近投影当作前段已观测证明。

扩展要求：同侧历史/当前均至少2个core、至少2个新增core，平均横向距离减小
至少一个栅格且大于等于纵向质心变化。它是扩展迹象，不是物体关联或速度估计。
保存相邻状态及最多一个产生过正证据的参考状态；参考比较最多保留1秒，
避免无关inflation变化立即清掉预警，同时继续检测新进入的障碍。

完全重复的内容不推进状态；rolling新增覆盖本身不计状态变化或运动；
共同区域的实际代价变化可形成新状态，包括inflation变化，但后者不能形成
core运动证据。未知→已知不作为运动。负索引边界用额外虚拟栅格检查，
不将窗外位置钳到地图边缘。记录的是读取时刻和内容变化，不是地图真实更新stamp。

Governor 状态：

- Normal → Limited：可信Outer扩展，采用预警上限。
- Inner冲突且可停车 → Braking：每周期降低速度上限，保留可达减速边界。
- 实际线/角速度均足够小且零速可达 → Wait；零速候选仍须通过原碰撞检查。
- Inner停车条件不成立 → DwaFallback：保留原DWA候选和避让能力，记录cannot_stop。
  该名称不表示“碰撞不可避免”。
- 连续3个不同、全走廊已知、对齐且无风险的地图状态 → Recovering，渐增上限。
  重复快照不累计；恢复中失去有效证据不提速。

停车模型为 `v*(reaction_time+control_dt)+v²/(2*brake_acceleration)`，可用距离为
`inner.start-robot_radius-stop_margin`。此外，Inner观测前缀必须严格覆盖到
首个冲突区域前缘；未知区间触及该边界即拒绝停车许可。Outer或远端缺失不
单独否定停车前缀，但完整走廊已知仍是恢复条件。

终点状态一旦进入原Slowing/Rotating阶段，本模块让位于原终点控制。
相同目标的重规划保留风险状态，新目标/坐标系变化及deactivate/cleanup重置。
周期上限与终点/外部上限取交集，并保留动态窗口下界；不写持久限速参数，
不重建评分器、不改变其归一化参考。原外部限速导致的不可达窗口仍遵循原异常语义。

## 验证和限制

详见 `benchmark/path_conflict_v01_20261022/README.md`、`results.json` 和原始运行目录。
本轮先做5条件各一次；因停车覆盖检查过强，修正后仅对D01 2s/4s补各一次。
所有结果保留，不使用场景时序作为检测输入，也未调参或修改场景。

Inner前缀修正版两次D01仍碰撞：2s曾减速后因新近处冲突切回DWA；4s曾限速，后续
停车距离不足。没有Gazebo WAIT/恢复成功证据；该链路只在确定性插件闭环中通过。
S04/S05首轮均成功，无风险限速/等待；它们对应覆盖修正前版本，不能冒充最终版本
的同版本闭环结果。修正仅改变Inner停车许可，首轮两场均未出现Inner冲突。
最后还修正了折返路径未知区间/核心占据的最早相交距离，经过红绿单元回归和完整
功能测试，未再补Gazebo；上述两次D01对应此前的Inner前缀版本。最终业务源码
仅 `src/path_conflict_monitor.cpp` 与该闭环版本不同，三个源码包分别保留。

已知限制：

1. 完整Outer覆盖缺失，或清障后地图长期不变，可使限制/WAIT保持到Nav2进度检查器介入。
   本轮未放松恢复条件或延长进度检查超时。
2. 停车模型仅基于当前已观测core与路径。新占据突然出现在更近处时，先前条件
   可能失效；没有障碍速度预测、物体关联、遮挡推理或动态安全保证。参考延迟和
   减速度尚未经过真实机器人标定。
3. 静态墙感知噪声、路径弯折和多个物体混合可产生扩展假象；本轮样本不足。
   实际footprint运行时变化也未跟踪，需要重新配置。保留原中心点/膨胀碰撞检查，
   未新增完整footprint碰撞算法。
4. 地图内容不变不能证明感知流新鲜；不把控制读取频率、发布频率等同内部更新频率。
   大地图/复杂路径上的控制性能、压力测试和实体机器人均未验证。
