# MyDWA 第一轮参数审计与冻结结论 — 2026-10-19

**保留原始参数，不接受调优配置。历史基线归档冻结完成，可靠性验收未通过。**
本轮没有找到满足“不通过参数掩盖算法缺陷”要求的参数修复路径；不声称所有参数组合
都不可能改变 S04 成功概率。建议先单独修复终点状态与 Nav2 重规划/定位修正的交互，
再进入动态障碍策略开发。

本轮新增 **3 次真实 Gazebo + Nav2 导航**，全部为原参数 S04 诊断复跑；另执行
**1 个确定性插件机制复现**，不是 Gazebo 性能样本。候选配置 **0** 套。
原有 36 次完整保留，没有删除失败、重写原始轨迹或修改原报告。

## 1. S04 首次异常与证据链

核心证据：[V1 MyDWA S04/1 完整日志](../results/v1/static_v1_20261019/logs/mydwa_S04_1_launch.log)。
以下是首次**记录到**的状态，不是完整逐周期记录；诊断 INFO/WARN 独立按约 1 Hz 节流。

|日志行|距离 m|yaw 误差 rad|观测|
|---:|---:|---:|---|
|486|0.315665|−0.938188|Normal，346 个可行候选，输出 v=0.188508；74 条 inscribed 拒绝|
|488|0.231509|−0.860922|已 Rotating，20 个可行候选，v=0、w=−0.506425|
|493|0.281975|−0.294797|已超出实际 0.25 m 容差，仍 Rotating，所有非零 v 被终点规则排除|
|495|0.296858|−0.244694|yaw 已在 0.25 rad 内；输出 v=w=0；1 个可行零轨迹，障碍拒绝全零|
|505|—|—|首次 Failed to make progress，之后重复恢复/重规划仍停滞|

第 488→495 行，机器人 odom 位置变化约 0.47 mm，而同一 map 目标在 odom 中的位置
变化约 77 mm。102 条全局路径的 map 终点全部是 `(2,0,0)`，不是规划器更换目标。
`localization.jsonl` 有同时段 AMCL 更新；相邻 TF/odom 原生时间配对显示 map/odom
关系变化，map-y 从约 −0.205 变至 −0.277，而 odom 平移几乎不动。
因此主要可见触发因素是定位/TF 修正后重新落在 XY 容差外，不是机器人明显驶离目标。

首次 ProgressChecker 失败前，已经存在“容差外但禁止平移”的状态；不能倒因为
ProgressChecker 过严，也没有证据把它归为五项权重导致的无可行轨迹。
原 trial 的最终 Action 是 watchdog 取消后的 CANCELED，119.9 s 是观测跨度，
不是完成导航所需时间。最后误差 0.525 m，恢复反馈最大值 9。

另两条成功记录同样重要：

- V1 S04/2 曾在 0.252–0.254 m、Rotating、零输出状态触发一次 progress failure，
  后续 AMCL 更新后才结束；35.0 s 的 SUCCEEDED 不意味着全程正常。
- V1 S04/3 在终点旋转时约 0.239 m，最终成功；前段曾短暂无可行轨迹，全部 400 条
  被 inscribed 拒绝，警告时间跨度约 0.16 s 后继续。它是另一个瞬态问题，
  不是 S04/1 的持续停滞链，不能把警告跨度当成完整不可行持续时间。
- 新诊断 S04/2 再次出现同机制：首次记录到容差外 Rotating，随后距离 0.300472 m
  时输出零、1 个可行候选、障碍拒绝全零；4 条 progress failure，恢复反馈最大值 5，
  最终在 66.5 s 才成功。相同原参数三次成功不能解释成调优改善。

## 2. 代码机制与推论边界

MyDWA `src/controller.cpp` 的 `prepareGoalControl()` 在距离满足 XY 阈值后锁存
Slowing/Rotating，没有因 XY 距离再次变大退出的分支。`isGoalCandidateAllowed()`
在 Rotating 拒绝所有非零平移；`scoreCandidate()` 在终点阶段只比较预测终点 yaw，
不使用 obstacle/speed/goal/path/heading 五权重。`setPlan()` 对相同 frame、相同终点
（XY/yaw 1e-6 内）保留阶段。

安装的 Nav2 1.1.20 每次设置路径都会调用 Controller.setPlan，再无条件重置
GoalChecker；SimpleGoalChecker 重置后重新检查 XY。因此同终点重规划可能保留
MyDWA 的终点锁存，却清除 GoalChecker 的 XY 锁存。见官方固定版本
[ControllerServer](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_controller/src/controller_server.cpp)、
[SimpleGoalChecker](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_controller/plugins/simple_goal_checker.cpp)。

DWB 的 setPlan 则重置 critics；这解释了为何不能把 MyDWA 的同终点锁存行为视为
与 DWB 相同，但不据此宣称 DWB 永远不会出现类似问题。见
[DWBLocalPlanner](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)。

使用原始 YAML、未修改的 MyDWA 插件和官方 SimpleGoalChecker 的
[确定性机制复现](evidence/terminal_latch_probe.cpp) 已实际编译通过：

```text
distance=0.3 tolerance=0.25 normal_v=0.125 latched_v=0 latched_w=0 checker_after_reset=0
```

人工空地图里，同一容差外位姿在 Normal 可前进；进入终点阶段后，按 Nav2 的顺序
setPlan(同终点)+checker.reset，再查询该位姿，Controller 输出零而 GoalChecker 拒绝成功。
它证明该状态组合存在及其退出能力缺口；没有模拟 AMCL、TF 时序或服务器调度，
不能充当 Gazebo 成功率、耗时或定位误差数据。

原有 `TerminalReplanWithQuaternionSignFlipPreservesLatch` 测试本来就要求同终点
重规划后在容差外保持零平移；原闭环终点测试传入 nullptr GoalChecker，没有覆盖
“Nav2 重规划重置实际 GoalChecker”的组合。本轮不更改实现或原测试期望。

目前最可能根因是**终点状态机与重规划/GoalChecker 重置之间的状态不一致，由定位
修正触发**。机制已独立复现，但原试验没有记录每周期 GoalChecker 内部状态与
成功检查使用的精确 TF，所以原始事件的逐周期因果仍是有证据支持的推断。
也未保存原始 costmap；所列周期有可行轨迹、没有障碍拒绝，不代表已排除全程地图影响。
map/odom 配对允许最大 0.025 s 原生时间差，是诊断近似，不是新的真值或安全净间距。
`controller_errors` 保存原日志，包括关闭时的 lifecycle 错误；不能将清理日志算作导航故障。
FollowPath 的状态 6 可能来自正常重规划抢占，同样不等价于控制器失败次数。

## 3. 参数修改表与停止依据

21 个 MyDWA 参数的默认值、当前值、范围、公式、运动学对照和候选门槛详见
[PARAMETER_AUDIT.md](PARAMETER_AUDIT.md)。本轮所有值均保留，未生成假想“最优 YAML”。

|类别|原值|候选/最终值|依据与潜在副作用|
|---|---|---|---|
|五权重（ob/speed/goal/path/heading）|.10/.10/.60/.20/.05|无候选；原值保留|停滞阶段不使用五项；改变绕障路线可能只掩盖状态问题并影响 S03/S05|
|速度/角速度采样|20/20|无候选；原值保留|停滞不是网格没有有效零轨迹，而是终点阶段禁止前进；降密度损失分辨率，加密度增加负载|
|预测时域/步长|1.7/.05 s|无候选；原值保留|缩短时域可能加快旋转并改变定位更新相对时序，但不消除锁存不一致，也改变普通预测|
|终点 fallback XY/yaw|.25 m/.05 rad|无候选；原值保留|实际 GoalChecker 提供 .25/.25，日志 fallback=[0,0]，改 fallback 无效|
|Goal/ProgressChecker|.25/.25；.5 m/10 s|不调整|放宽到达标准或推迟进度失败无法恢复平移，也破坏原验收语义|

最大速度、加减速度、footprint/robot_radius、膨胀、碰撞判断、全局规划器和恢复
配置全部不变。指令限制一致不代表实测没有超调：V1 S04 的实际 vx 峰值分别为
MyDWA 0.31070、DWB 0.28345 m/s，新原参数组为 0.31189 m/s；两路指令均在
0.26 m/s、1 rad/s 限制内。不能将此归档称为真实机器人运动学/碰撞安全验收。

普通阶段限频诊断记录中，MyDWA 计算常超过 50 ms 控制预算，采样负载值得后续
单独测量；终点停滞时约 0.1 ms，不是算不出速度。没有以减少样本改变本轮问题定义。
DWB YAML 的 `FollowPath.stateful` 未被当前插件声明；实际共用 GoalChecker 的
stateful=true 已核验。原 YAML 保留不动，未将该未声明键伪报为已加载。

## 4. 真实对照：每次都列出

时间为首次规划 Goal 原生时间至观测结束的仿真跨度，约 0.1 s 量化，不是精确
NavigateToPose Action 时长。只有 SUCCEEDED 且数据 valid 的试验进入成功耗时均值，
成功中的等待/恢复时间照计；失败观测跨度另列，不能混入正常完成时间。
长度、RMSE、wz 统计、终点误差为该条完整观测记录，失败的局部轨迹也保留。

|组/编号|Action|成功耗时 s|失败跨度 s|长度 m|横向 RMSE m|wz 标准差 rad/s|wz 反向次数|终点 m/rad|恢复反馈最大值|
|---|---|---:|---:|---:|---:|---:|---:|---|---:|
|V1 MyDWA/1|CANCELED / wall_timeout|—|119.9|5.071|.115|.225|8|.525/.237|9|
|V1 MyDWA/2|SUCCEEDED|35.0|—|4.598|.088|.328|4|.240/.245|1|
|V1 MyDWA/3|SUCCEEDED|22.5|—|4.485|.096|.389|4|.215/.245|0|
|V1 DWB/1|SUCCEEDED|18.9|—|4.117|.079|.255|2|.232/.259|0|
|V1 DWB/2|SUCCEEDED|19.1|—|4.153|.077|.257|4|.220/.267|0|
|V1 DWB/3|SUCCEEDED|19.1|—|4.102|.078|.253|2|.208/.252|0|
|新原参数/1|SUCCEEDED|21.6|—|4.506|.092|.413|4|.198/.254|0|
|新原参数/2|SUCCEEDED|66.5|—|4.898|.084|.304|8|.258/.237|5|
|新原参数/3|SUCCEEDED|22.2|—|4.537|.101|.413|6|.235/.246|0|

|S04 分组|成功/总数|成功耗时均值 ± 样本 SD，s|n|解释|
|---|---:|---:|---:|---|
|原始 MyDWA V1|2/3|28.75 ± 8.84|2|失败 119.9 s 单独保留|
|官方 DWB V1|3/3|19.03 ± .12|3|仅历史参照，本轮未改 DWB/未重测|
|本轮原参数诊断|3/3|36.77 ± 25.75|3|没有调参，第 2 次仍长期停滞并恢复|
|调优候选/调优最终版|未执行|—|0|缺少符合约束的参数修复依据|

不能把相同参数从 2/3 到 3/3 的抽样变化说成可靠性提升，也不根据三次试验宣称
统计显著性。没有跨批合并来稀释历史失败。恢复反馈次数包含 costmap 清理等，
不等于 Spin/BackUp/Wait 的独立 Goal 数，原始 Action/BT/feedback 全部保留。

S03、S05 等场景沿用 V1 的 MyDWA/DWB 各 3/3 结果。本轮参数未变，但未新增这些
场景的导航，不能声称经过一轮新回归排除了退化。用户规定的 S03/S05 防退化和
最终六场景各两次，以候选改善并选中为前提；本轮未达到该门槛，因此未消耗额外试验。

## 5. 冻结范围、文件与校验

- 原始 MyDWA 归档：[configs/mydwa_static_v1_original.yaml](configs/mydwa_static_v1_original.yaml)
- 官方 DWB 参照归档：[configs/dwb_v1_reference.yaml](configs/dwb_v1_reference.yaml)
- 机器可读决策、源码/配置哈希、39 次实际参数核验：[freeze_manifest.json](freeze_manifest.json)
- 原 V1 全量只读复核：[evidence/v1/analysis.json](evidence/v1/analysis.json)
- 本轮三次分析：[evidence/diagnostic/analysis.json](evidence/diagnostic/analysis.json)
- 新原始记录：[三次诊断结果目录](../results/v1/tuning_r1_diagnostic_baseline_20261019)
- 插件机制复现日志/XML：[terminal_latch_probe.log](evidence/terminal_latch_probe.log)
- 回归和 lint：[check.log](evidence/check.log)

原始 MyDWA SHA256：
`0213f8ceb754196697c53c965789e6cc485f267a51d416576076df36920ee689`

官方 DWB SHA256：
`21f6a890657da341a854efce403c895304a3ebc1a8f91f466348e720f2641380`

当前插件二进制 SHA256：
`78018c60ec1c24c5efd5931cb9218fa149bf6bff4fc4589907ff9f804b1eddb4`

V1 没有二进制哈希，不能宣称逐位相同；8 个原 C++ 源文件、原参数、UI 入口的
既有哈希未变，安装软件版本也一致，并已补做新原参数组。S04 的 World、机器人、
地图图像/YAML、BT、共用实际参数/约束/起终点在两批记录中签名一致。
全部 MyDWA 显式 YAML 参数均逐项对比实际运行参数，DWB 未声明键单列而非忽略。

实际回归：原有 34 项 Benchmark Python 测试、8 个 C++ 测试目标共 200 项通过；
新增独立机制复现通过；flake8/pep257/cppcheck 等通过。全包仍只有历史
`src/obstacle_evaluator.cpp` uncrustify 失败，未修改该文件以掩盖故障。
本轮没有重建或修改控制插件，独立 probe 只复用原编译/链接配置生成自己的可执行文件。

所有新增内容在 `benchmark/tuning_r1/` 和独立新结果目录；没有改 Controller C++、
现有测试、DWB 参数、原始 MyDWA YAML 或现有 Benchmark 执行/指标逻辑。
本轮不增加无用的候选注入功能；runner 的 `--params-file` 仅指向记录/预期文件，
不能把它当作 launcher 已换参的证据。此处用原始默认启动路径，配置文件哈希和
全部实际 MyDWA 参数比对共同证明加载的是归档原版。

## 6. 实际命令与复现

在 WSL Ubuntu-22.04 的 Bash 中运行：

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws/src/my_dwa_controller/benchmark

# 原始参数与冻结副本必须一致；不同则停止并另行设计配置注入，不能覆盖历史 YAML。
cmp ../config/nav2_params.yaml tuning_r1/configs/mydwa_static_v1_original.yaml
sha256sum -c evidence/unchanged.sha256

# 本轮实际运行命令，共三次：
python3 run_suite.py --controllers mydwa --scenes S04 --repeats 3 \
  --run-id tuning_r1_diagnostic_baseline_20261019

# 复跑时省略 run-id 自动产生新目录。已有目录拒绝覆盖。
# 以下离线分析不修改试验原始文件、summary 或已有 metrics：
python3 tuning_r1/analyze.py results/v1/static_v1_20261019 \
  --output tuning_r1/evidence/v1
python3 tuning_r1/analyze.py results/v1/tuning_r1_diagnostic_baseline_20261019 \
  --output tuning_r1/evidence/diagnostic
python3 tuning_r1/freeze.py

# 使用已有已构建测试的编译/链接参数，加载原插件作人工状态机制复现：
bash tuning_r1/run_probe.sh

# 全包回归：因已知 uncrustify 返回非零，查看 check.log 中的具体测试结果。
bash tuning_r1/check.sh
cat tuning_r1/freeze.log
python3 -m json.tool tuning_r1/evidence/diagnostic/analysis.json
```

归档副本发生内容冲突时 `freeze.py` 拒绝覆盖；未来新算法/参数应建立新版本目录。
后续必须先在单独任务中处理终点锁存与 GoalChecker/定位修正组合，再用相同到达
标准复测 S04、S06 及六场景。本轮没有证明更安全，没有可靠碰撞/最小净间距指标，
也没有把“历史可复现”误标为“适合直接进入动态障碍性能验收”。
