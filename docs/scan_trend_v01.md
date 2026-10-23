# MyDWA Scan 短时运动趋势 V0.1

默认关闭的实验功能。已有真实 D01 短时运动估计、冲突判断和控制限速证据；
**没有动态避障已解决的证据**。最终 D01 2s 无接触但90s导航超时。

## 数据流与职责

`ScanObstacleTracker` 在 activate 时订阅、deactivate/cleanup 时注销。回调只向有界
共享 inbox 复制消息，不捕获 Controller/tracker 裸指针；控制周期处理待变换扫描。
使用 Controller 原有 TF buffer，在扫描源时间变换到默认固定 `odom`；不以 latest TF
代替缺失的源时间 TF。若 time_increment>0，对首末光束时刻的 SE(3) 变换插值；
当前 Gazebo 实测 time_increment=0，同一扫描按瞬时观测处理。TF稍晚到达时留在有界
队列重试；没有把 TF 失败当作空扫描，也没有订阅 Gazebo 真值进行控制。

测距过滤 → 扫描顺序相邻距离聚类（含360°接缝合并）→ 观测包围盒中心与可见尺寸
→ 位置预测的最近邻关联（双向歧义检查和尺寸门控）→ 0.8s窗二维最小二乘趋势。
至少4次、0.55s跨度才给可信度；可信度基于跨度和拟合残差。运动门槛0.08m/s、
可信度0.65。它不是概率或误差保证：线性的可见表面漂移也可能得到高分。

`PathConflictMonitor::observe(..., scan)` 复用原前向路径、路径投影和同一地图副本。
保留 Costmap 当前 Inner core 占据与原碰撞检查。scan模式下只由 scan 提供运动趋势，
替代旧 Costmap Outer 扩展迹象，合入同一 ConflictEvidence，不新建风险状态机。
对可信运动簇作0～2s、0.1s步长外推，计算预测圆盘与路径段的解析相交位置；半径
含机器人半径、可见簇半径和随时间/残差增加的余量。对路径采样密度不敏感。
这是短期空间冲突预警，没有完整机器人/障碍联合轨迹规划或碰撞概率。

`RiskGovernor` 继续使用原停车距离、Inner观测前缀、路径对齐、清障确认和恢复。
scan模式下不同新扫描且新鲜、无未决丢失才允许计数；仍要求原完整走廊已知和对齐。
已经减速后停车条件失效，Fallback保留此前硬上限，Controller保持可达减速边界。
原软偏好仍可选择较慢候选，并保留当前采样窗口内的加速退路；补充软边界不得越过
实际硬上限/动态窗口。没有直接发布未经碰撞检查的零速指令。

0.35s外不再新鲜，最多0.8s短时保留/外推。已见运动的簇丢失、关联不可靠或重获后
拟合尚不可信时都不能作为清障证据。超过保留时间的运动簇留下未决标志，直到导航
任务/生命周期重置；当前没有独立可见性清除算法，因此可能长期不恢复。新的目标
和deactivate/cleanup重置，相同目标重规划保留状态。新模式关闭时保留旧行为。

## 配置、构建和诊断

参数在 configure 读取，不支持即时参数回调。原正式 YAML 没有修改。实验副本：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      path_conflict:
        enabled: true
      scan_tracker:
        enabled: true
        topic: /scan
        fixed_frame: odom
```

`scan_tracker.enabled` 默认false，另两键默认如上。启用scan而关闭path_conflict会在
configure报错，避免悄悄启用无效功能。其余感知常量见 ScanTrackerConfig/实现，
本轮没有依据障碍启动时刻写检测条件，没有阈值搜索。scan=false时不创建订阅。
插件独立安装在 `/home/user/ros2_ws/install_scan_trend/my_dwa_controller`。

日志 `[SCAN_STATUS]` 给出扫描stamp、frame、序号、新鲜度和TF失败次数；
`[SCAN_TRACK]` 给出track id、源stamp、位置/速度、可见半径、置信度、残差、拟合
跨度、观测/推算状态、路径距离、纵向位置、接近趋势、预测相交时间和Governor状态。
`[PATH_CONFLICT]` 追加 scan/fresh/unresolved/seq/predicted，保留原有键。
track位置被推算到查询now，速度是以源stamp结束的短窗拟合；age为now-stamp。
预测first_hit是离散外推的首次相交时间，不是经过标定的碰撞时间。

完整实现/实验/失败分析与版本哈希见
`benchmark/scan_trend_20261022/REPORT.md`。本轮只覆盖单圆柱合成单测和原方柱Gazebo，
没有复杂多目标、真实机器人、大地图压力或新坐标系在线切换验证。
