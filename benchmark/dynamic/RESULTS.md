# Nav2 动态障碍 Benchmark V0.1 首轮报告

日期：2026-10-20（Asia/Shanghai）。正式批次：`formal_v0_1_20261020`。

已实际完成 **12/12 次独立导航**，12 次均被 NavigateToPose 接收、两种真实插件核验通过。结果为 5 次成功、6 次物理碰撞终止、1 次 Nav2 导航失败；无正式测试系统异常、无正式超时。所有失败均保留，没有挑选结果或为成功率调参。每组只有 3 次，以下仅为本批次描述。

## A. 修改摘要与工程复用

原仓库位于 `${REPO_ROOT}`，HEAD 为 `1bad3b66aa64384d5955f1933dcfb5c1017dcd62`，分支 `feature/dynamic-obstacle`。工作区父目录不受 Git 管理，未发现 AGENTS.md。初始 tracked 工作树干净；根忽略规则原本整体忽略 benchmark/test/md，不能据此断言这些被忽略的既有文件没有历史用户修改。本次读取并保留了既有静态脚本、S01–S06 资产及结果，不做重构。

新增目录 `benchmark/dynamic`，复用现有 `scenes.generate()`、`run_trial.Collector/preflight/ActionWorker`、MyDWA/DWB 冻结 YAML、`_baseline_common.sh` 和 `_baseline_processes.py`。唯一既有 tracked 文件改动是批次结束后为动态模块增加 `.gitignore` 例外。没有修改 Controller 源码、头文件、Nav2 配置、机器人/雷达/Costmap、静态评测实现和旧报告，没有提交或推送。

|文件|用途|
|---|---|
|`scenarios.json`, `scenario.py`, `assets/D01|D02/*`|确定性关键帧、尺寸、起终点，以及配套物理世界/地图/几何|
|`motion.py`|只移动动态盒，不写机器人或控制指令|
|`observer.py`|独立保存真值、contact、雷达、Costmap、实际 cmd_vel|
|`run_trial.py`|真实 Nav2 action、插件与参数核验、仿真时间超时|
|`start_scene.sh`, `run_suite.py`, `run.sh`|Headless、逐轮重启、交替 Controller、失败保留、CSV/JSON/Markdown|
|`evaluation.py`|独立离线净间距、角速度、停止/恢复与终止评价|
|`../test_dynamic.py`|新增 7 个边界/几何/异常路径回归，纳入现有 unittest discover|
|`README.md`, `RESULTS.md`|定义、复现和本报告|
|`evidence/*`|构建、测试、运行、审计、失败原因摘录及生成脚本|
|`first_run/*`|首轮 CSV/JSON/审计/manifest 副本与完整原始记录归档|
|`../results/dynamic/*`|全部正式运行及 Smoke 的原始结果；依照原规则保持 Git 忽略|

D01 是 4 m 直行、3 m 宽通道，0.5 m 方盒在 x=1.6 从 y=-1.1 横穿至 y=+1.1，t=4..12.8、0.25 m/s。D02 为同起终点、0.9 m 通道与封闭侧袋，0.4×0.8 m 盒在 t=2..6.4 进入，停留至 t=20 后退出，t=23.4 主通道恢复、t=24.4 完全回到侧袋。运动 epoch 固定为任务 dispatch 的仿真时间，不以机器人位置触发。

## B. 环境与验证

- WSL2 Ubuntu 22.04.5、ROS2 Humble、Gazebo Classic 11.10.2、Nav2/DWB 1.1.20、gazebo_ros 3.9.0、TurtleBot3 Gazebo 2.3.8。版本和 SHA256 见 `evidence/environment_full.log`。
- `colcon build --packages-select my_dwa_controller --cmake-args -DBUILD_TESTING=ON` 成功。原 build cache 的 BUILD_TESTING=OFF 已纠正，未把空测试当作通过。
- 语法检查、41 个 benchmark Python 测试、6 个启动器隔离/清理测试通过；新增 Python flake8/pep257 通过。8 组 C++ GTest、cppcheck、CMake/XML 检查通过。
- 完整 CTest **14/15 通过**；现存 `src/obstacle_evaluator.cpp:43` 的 uncrustify 格式差异仍失败。其内容与 HEAD 一致，本次按冻结约束保留，不能声称全套检查全绿。colcon test-result 的 2 条 failure 表示同一格式问题在 CTest 汇总和 xunit 子记录重复出现。
- D01/MyDWA 修复后 Smoke：真实导航，物理碰撞；D02/DWB Smoke：SUCCEEDED、38.1 s、无接触。Smoke 和正式批次完全分开。
- 所有正式运行均证实障碍移动；原始真值与固定关键帧在接收 clock 关联下最大差异约 0.025 m，符合约 10 Hz 台阶和一 tick 对时误差。每轮 scan、Local Costmap 均有动态盒区域命中，原始 grids/ranges 全量保存并逐条重算一致。
- 全部 12 次验证 `my_dwa_controller::MyDWAController` / `dwb_core::DWBLocalPlanner`，每轮保存完整运行参数、世界、地图、机器人、BT、源码快照及二进制哈希。两场景各自公共运行签名一致，GT 订阅者仅评价节点，真值未注入控制器。
- 正式批次期间 23 个原 tracked 文件前后 SHA256 一致；所有 cleanup 返回 0。`.gitignore` 的交付例外在批次后添加，未改变正式运行配置。MyDWA 插件 SHA256 为 `976c629c8dcf112d80cad7cd994b7840b1f6bfac518e1a9b51bb3fc630e7d4c7`。
- Headless 各轮独立启动 Gazebo/Nav2/AMCL/Controller；没有通过重置复用上一轮进程。全部真实导航在无 RViz/gzclient 模式执行。

## C. 正式 Benchmark 结果

时间均为 dispatch 至观测终端 action 的仿真秒，包括碰撞取消确认；只有成功项进入成功时间均值。碰撞使用物理 contact，净间距是原机器人碰撞包络矩形到动态盒的**近似**距离，重叠为 0，不能当作精确形状并集或穿透深度。详细定义见 README。

|场景|Controller|真实接收|成功率|成功平均时间 s（n）|碰撞运行|最小近似净间距 m|
|---|---|---:|---:|---:|---:|---:|
|D01|mydwa|3|0/3|NA (0)|3|0.000|
|D01|dwb|3|0/3|NA (0)|3|0.000|
|D02|mydwa|3|3/3|38.400 (3)|0|0.131|
|D02|dwb|3|2/3|38.350 (2)|0|0.151|

|场景/Controller/次|真实 action|终止原因|运行 s|物理碰撞|近似间距 m|角速度总变差 rad/s|有效反转次数|
|---|---|---|---:|---|---:|---:|---:|
|D01/mydwa/1|CANCELED|collision|7.0|是|0.000|0.294|0|
|D01/dwb/1|CANCELED|collision|6.8|是|0.000|1.000|1|
|D01/dwb/2|CANCELED|collision|7.0|是|0.000|1.000|0|
|D01/mydwa/2|CANCELED|collision|7.0|是|0.000|0.477|0|
|D01/mydwa/3|CANCELED|collision|7.0|是|0.000|0.812|1|
|D01/dwb/3|CANCELED|collision|6.8|是|0.000|1.421|1|
|D02/mydwa/1|SUCCEEDED|success|38.6|否|0.136|10.036|3|
|D02/dwb/1|SUCCEEDED|success|38.2|否|0.191|10.579|5|
|D02/dwb/2|SUCCEEDED|success|38.5|否|0.155|11.324|3|
|D02/mydwa/2|SUCCEEDED|success|38.1|否|0.140|9.847|3|
|D02/mydwa/3|SUCCEEDED|success|38.5|否|0.131|10.414|5|
|D02/dwb/3|FAILED|navigation_failed|26.3|否|0.151|5.105|3|

碰撞任务由测试端取消，因此 action 为 CANCELED、benchmark 原因为 collision；没有冒充 Nav2 自主 FAILED。D01 无成功项，完成时间均值为 NA；6.8–7.0 s 是失败运行时长。

|D02 Controller/次|停止/低速 s|通道释放（相对 dispatch）s|释放后恢复 s|feedback 恢复计数|Spin/Backup/Wait goals|
|---|---:|---:|---:|---:|---|
|mydwa/1|15.3|23.4|3.3|15|2/1/2|
|dwb/1|15.1|23.4|3.7|15|2/1/2|
|dwb/2|15.2|23.4|4.1|15|2/1/2|
|mydwa/2|15.5|23.4|3.0|15|2/1/2|
|mydwa/3|15.5|23.4|3.3|15|2/1/2|
|dwb/3|15.8|23.4|NA|16|2/1/2|

恢复计数来自公开 feedback，包含上下文恢复，不能解读为 15–16 次 Spin；右列才是按 UUID 去重的行为 goal 尝试次数，不代表行为全部成功。停止累计包含启动/等待/终点低速。DWB 第 3 次未在失败前出现满足定义的持续恢复前进，保留 NA。

|运行|雷达命中帧/总帧|Costmap 命中帧/总帧|contact 消息数|
|---|---:|---:|---:|
|D01/mydwa/1|35/35|10/10|132|
|D01/dwb/1|34/34|11/11|49|
|D01/dwb/2|35/35|12/12|109|
|D01/mydwa/2|35/35|10/10|99|
|D01/mydwa/3|35/35|11/11|99|
|D01/dwb/3|34/34|12/12|58|
|D02/mydwa/1|169/193|58/63|0|
|D02/dwb/1|166/191|58/63|0|
|D02/dwb/2|165/192|59/63|0|
|D02/mydwa/2|167/190|56/61|0|
|D02/mydwa/3|167/193|58/63|0|
|D02/dwb/3|117/131|40/44|0|

contact 消息数包含接触期间的重复物理消息，不等于独立撞击次数。scan/Costmap 计数是动态盒附近关联证据，不是传感器检测精度。完整原始数据在 `benchmark/results/dynamic/formal_v0_1_20261020`；`validation.json` 确认 12/12 accepted、全部 metrics 重算相符、全部 sensor grids/ranges 重算相符、快照哈希有效、无 action-client 错误、清理成功。

## D. MyDWA 与 DWB 的初步比较

**实际测量：** D01 两者均 3 次碰撞、零成功，首次碰撞均在横穿交互阶段；两者均有雷达和 Costmap 证据，不能归类为障碍没有进入感知。D02 MyDWA 3/3、DWB 2/3；成功时间均值 38.40 s（n=3）与 38.35 s（n=2），相差 0.05 s 小于单 tick 对时精度，没有可解释的速度优势。两者均触发 Recovery；成功项释放后恢复范围为 MyDWA 3.0–3.3 s、DWB 3.7–4.1 s（仅两个成功项）。

**实验观察：** D02 各轮出现 2 个 Spin、1 个 Backup、2 个 Wait goal 尝试。有效角速度方向反转为 3–5 次，停止约 15.1–15.8 s，均无物理接触。不能仅凭指令总变差认定这些动作“不必要”。MyDWA 的 control loop missed 警告为 D01 各 51–54 次、D02 各 134–139 次；DWB 本批次对应日志为 0。该差异已经观察到，不能据此直接归因于算法精度或碰撞。

**失败原因观察：** D02/DWB 第 3 次真实 ActionStatus=6 (ABORTED)，26.3 s 终止，16 次 feedback recovery；物理通道已在相对 t=23.4 释放，但终止前仍有 GridBased 无有效路径、Backup 预测 Collision Ahead、最终 Goal failed。最后机器人真值约 x=1.402、y=-0.083、yaw=-3.095 rad，仍位于 blocker 前方。该 Collision Ahead 是恢复行为的预测报错，contact=0，不能当作真实碰撞。日志摘录见 `evidence/d02_dwb_003_failure_excerpt.log`。

**可能原因：** 暂时不可通行使全局规划与 BT 恢复反复切换；退出后的观测清除、重规划、恢复预算及姿态共同影响能否继续。横穿场景需要进一步检查感知更新、预测轨迹和制动时序的关系。

**尚未验证：** 没有逐周期因果实验证明 D01 碰撞具体由哪一模块造成，也没有证明最后一轮 DWB 失败由 Controller、全局 Costmap、AMCL 或恢复时序中的单一因素造成。D02 是端到端 Nav2 任务，不能把所有成功/失败归于局部控制器。n=3 不支持显著性推断或全面优劣结论。

## E. 发现的问题及处理边界

|类别|证据与处理|
|---|---|
|Controller / 执行性能|D01 两种插件均发生物理接触；MyDWA 存在控制周期 miss 日志。只记录原始性能，未改算法、采样、预测、评分或参数。|
|Nav2 规划/恢复链路|D02 各轮 Recovery；DWB 第 3 次释放后仍规划失败而 ABORTED。没有改 BT、planner、progress checker 或任何 YAML。|
|Gazebo 场景生成器|最初 Smoke 多个 contact 插件节点名相同，gzserver -11 崩溃，导航未接收；补唯一名称回归后修复，原失败目录与 failed_world.sdf 保留。|
|Benchmark 异常路径|独立审查发现清理异常可能继续下一轮、action-client 异常可能误记导航失败。正式 12 次均未触发；批次后通过故障注入回归修复并真实 Smoke 验证正常清理。原正式快照保留，按修复后评价重算数值完全一致，无须替换正式运行。|
|审计脚本|2 帧雷达逐射线计数初次不一致：读者用 float64 的 0.12，而 ROS range_min 是 float32 的 0.1199999973。按原机器人 SDF/消息表示还原读取边界后全部一致，未改传感器阈值、真实记录或碰撞规则。新记录显式附带 range_min/max，旧原始记录不回填修改。|
|既有检查/交付|原 C++ uncrustify 一项失败按冻结保留；原 benchmark 根忽略规则使新增文件不可见，批次后补动态模块例外，旧静态结果仍不纳入变更。|

Smoke 全部保留：`smoke_d01_001`（基础设施预检失败、未导航）、`smoke_d01_002`（MyDWA 碰撞）、`smoke_d02_001`（DWB 成功）、`post_review_smoke_d01`（DWB 碰撞、清理确认）。它们不进入正式 12 次统计。没有重跑或删除任何正式失败。

已知实验限制：规定运动的不可推动盒、约 10 Hz 位置台阶、50 Hz 无时间戳 model_states、100 Hz 有限接触检测、机器人保守矩形近似、固定时间触发导致交互时序差异、AMCL/激光噪声和调度随机性。没有宣称精确 Ground Truth 净间距，也没有把碰撞取消的最终时间当成首次接触时刻。完整定义、参数值和最小复现命令见 README。

## F. 下一阶段建议（本次未实施）

1. 对 D01 做按时间对齐的 scan → Costmap → 预测轨迹 → cmd_vel → contact 因果回放，区分更新滞后、制动能力与动态运动预测问题。
2. 针对 D02 分离研究 Nav2 全局规划/BT 恢复时序与局部 Controller 行为，先解释释放后恢复失败，不从本轮少量样本直接调参。
3. 保持本轮不变作为基线，后续研究增加明确种子与更多重复，并用更精确的碰撞形状和源时间戳提升评价分辨率。

完整复现：在 WSL 的仓库目录执行 `bash benchmark/dynamic/run.sh --repeats 3 --run-id new_unique_id`。运行命令、结果解释和离线审计均见 README / evidence，所有原始正式与 Smoke 记录归档到 `first_run/raw_records.tar.gz`。
