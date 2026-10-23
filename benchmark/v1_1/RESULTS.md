# MyDWA 终点状态同步修复与 V1.1 回归

日期：2026-10-19。协议及可复现命令见 [README.md](README.md)。原始 V1 和 R1 结果均保留，本轮没有候选参数、没有 DWB 新试验。

## 根因与修复

R1 已确定：同终点重规划保留 MyDWA 的 `Rotating`，而 Nav2 同时重置 stateful GoalChecker 的 XY 检查。定位变化后距离由约 0.231 m 增至 0.297 m，Controller 仍禁止平移、yaw 已达标时输出零速度，GoalChecker 却等待重新进入 0.25 m XY 范围。普通评分权重不能解除该锁存。

原 `setPlan()` 用终点位置、yaw 和 frame 判断 `same_goal`，只对不同终点重置；且在路径复制成功前就可能改状态。现实现删除终点身份判断，每条合法路径先校验全部位姿和 frame，复制成功后通过 noexcept move 提交，再在同一互斥锁内无条件设为 `Normal`。下一周期调用原 `prepareGoalControl()` 重新判断。无新路径时仍保持锁存，非法路径或复制失败不动旧路径和状态。

唯一生产修改是 `src/controller.cpp` 的 include 与 `setPlan()`；不改正常跟踪、预测、碰撞判断、评分、终点控制内部条件、C++ 头文件或任何参数。线程安全依赖原 `state_mutex_`；异常安全通过先验证/复制和不抛异常的提交保证。路径 TF 仍由原控制周期检查。与官方 Humble DWB 新路径重置 critic、RotateToGoalCritic 清除 `in_window_` / `rotating_` 的生命周期一致。

## 确定性证据

- 先在旧实现运行 28 项相关测试，23 通过、5 失败；失败涉及同终点 XY 漂出、真实 GoalChecker reset、q/-q 同终点和非法路径接收契约。原插件、源码、测试均已存档。
- 修复后 Controller 测试 **52/52**；8 个 C++ 测试目标合计 **206/206**；Benchmark Python **34/34**。正常控制指令特征签名测试保持通过。
- 新增 6 个测试，加上调整的历史预期，覆盖用户列出的全部状态情形。实际加载 `nav2_controller::SimpleGoalChecker`，模拟 Nav2 的 setPlan→checker.reset 顺序。以 20 Hz 闭环积分，在每周期重规划、每秒重规划两种频率下均完成旋转，同时检查线速度为零和角加速度约束。
- 构建、cppcheck、修改文件 uncrustify、Python lint 等通过。完整 colcon 检查仍有 **1 个已知历史格式失败**：`src/obstacle_evaluator.cpp`，未修改它。
- 独立只读代码审查无需要修改的发现；没有做内存分配失败注入或并发压力测试，不把代码保证描述成这两类试验已经执行。

具体证据：`evidence/red_tests.xml`、`evidence/tests.log`、`evidence/test_results/`、`evidence/controller.diff`、`evidence/test_controller.diff`、`evidence/review.md`。

## S04 历史对照与本轮结果

耗时为首个全局规划 Goal 原生仿真时间至观察到停止的跨度，约 0.1 s 接收量化；均值 ± 样本标准差仅计算有效成功试验。

| 版本 | S04 成功 | 成功耗时 s | 每次 feedback 恢复次数 | 进度失败总次数 |
|---|---:|---:|---|---:|
| MyDWA V1 原始 | 2/3 | 28.75 ± 8.84 | 9、1、0 | 9 |
| MyDWA R1 原参数诊断 | 3/3 | 36.77 ± 25.75 | 0、5、0 | 4 |
| **MyDWA V1.1 修复** | **5/5** | **22.50 ± 0.36** | **0、0、0、0、0** | **0** |
| DWB V1 历史参考 | 3/3 | 19.03 ± 0.12 | 0、0、0 | 0 |

V1 的失败试验保留：墙钟超时、Action CANCELED、观察仿真跨度 119.9 s，终点误差 0.525 m / 0.237 rad；119.9 s 没有混入成功耗时均值。R1 的 66.5 s 虽然 Action 成功，但有 5 次恢复、4 次进度失败，不能用“3/3 成功”掩盖它。

V1.1 的五次使用不同 suite ID `v1_1_S04_001`～`005`，内部各为 `trial_001`：

| suite 后缀 | 结果 | 耗时 s | 长度 m | 横向 RMSE m | 实际 ω 标准差 rad/s | ω 反向次数 | 终点 m / rad | 恢复 / 进度失败 |
|---|---|---:|---:|---:|---:|---:|---|---|
| 001 | SUCCEEDED | 22.4 | 4.590 | 0.101 | 0.445 | 6 | 0.195 / 0.251 | 0 / 0 |
| 002 | SUCCEEDED | 22.8 | 4.505 | 0.094 | 0.385 | 6 | 0.229 / 0.249 | 0 / 0 |
| 003 | SUCCEEDED | 22.4 | 4.570 | 0.102 | 0.448 | 6 | 0.233 / 0.249 | 0 / 0 |
| 004 | SUCCEEDED | 22.9 | 4.568 | 0.092 | 0.398 | 4 | 0.202 / 0.242 | 0 / 0 |
| 005 | SUCCEEDED | 22.0 | 4.498 | 0.105 | 0.422 | 4 | 0.237 / 0.262 | 0 / 0 |

五次均有约 22～23 条全局路径发布，观察到 Normal 后进入终点控制并完成，没有记录到 XY 外 `Rotating` 零输出，也没有进度停滞或恢复循环。001 的节流日志完整看到 Normal→Slowing→Rotating；其余短暂 Slowing 可能在约 1 Hz 的日志之间，不能据此声称跳过了减速。

本轮未人为注入 AMCL 漂移，五次都未复现历史 XY 漂出组合。解除锁存的因果证据来自确定性 C++ 测试与生命周期修复，真实试验支持无退化结论；不能把 5 次成功当作所有偶发故障均已排除的证明。

## 剩余回归

以下均为有效成功试验的均值；耗时列另给样本标准差。ω 为里程计测量实际角速度，反向计数使用 V1 的 0.05 rad/s 死区及 0.15 s 最短符号持续时间。

| 场景 | 版本 | 成功 | 耗时 s | 长度 m | 横向 RMSE m | ω 标准差 rad/s | 反向次数 | 终点 m / rad |
|---|---|---:|---:|---:|---:|---:|---:|---|
| S01 | V1 | 3/3 | 15.10 ± 0.00 | 3.750 | 0.023 | 0.041 | 1.00 | 0.250 / 0.056 |
| S01 | V1.1 | 2/2 | 15.05 ± 0.07 | 3.748 | 0.020 | 0.032 | 1.00 | 0.245 / 0.074 |
| S03 | V1 | 3/3 | 43.57 ± 0.06 | 11.071 | 0.052 | 0.179 | 4.00 | 0.237 / 0.155 |
| S03 | V1.1 | 2/2 | 43.55 ± 0.07 | 11.043 | 0.053 | 0.177 | 5.50 | 0.247 / 0.083 |
| S05 | V1 | 3/3 | 31.07 ± 0.06 | 7.810 | 0.026 | 0.047 | 7.00 | 0.239 / 0.073 |
| S05 | V1.1 | 2/2 | 31.00 ± 0.14 | 7.787 | 0.024 | 0.044 | 4.00 | 0.237 / 0.054 |
| S06 | V1 | 3/3 | 20.33 ± 0.23 | 3.843 | 0.022 | 0.277 | 2.67 | 0.232 / 0.252 |
| S06 | V1.1 | 3/3 | 20.47 ± 0.12 | 3.818 | 0.025 | 0.282 | 2.00 | 0.220 / 0.241 |

S06 三次恢复/Spin/进度失败均为 0，节流日志三次均记录到 Normal→Rotating。分别有 21、20、20 条全局路径发布，说明终点旋转是在正常重新规划持续存在时由 Controller 完成，而不是恢复行为 Spin。

S03 新的反向计数为 7、4 次，相比 V1 每次 4 次有所增加；角速度标准差、RMSE 与完成时间接近，未见持续振荡或进度失败。保留此变化作为后续观察项，不声称平滑性变好，也不凭两次试验认定显著退化。

上述剩余 9 次恢复及进度失败均为 0。S05 两次均正常通过窄通道，未见路径跟踪、耗时及角速度标准差的明显退化。

## 原始数据与局限

每次保存 trajectory.csv、summary.json、metrics.json、plans.jsonl、BT/Action/反馈/定位 JSONL、launch/runner 日志及配置快照。`evidence/comparison.csv` 包含全部新旧试验，不只保留最佳或成功记录；`evidence/analysis.json` 保留逐次控制阶段、恢复、异常、源数据计数及聚合值。

S04 的 001、002、004 各有一条节流诊断记录显示 Normal 阶段 400 个候选全因 inscribed cost 被拒绝，距离终点约 3.36～3.39 m；随后导航继续，零 progress failure、零恢复。控制循环超期警告仍存在。本轮修复没有消除这些历史可行性与计算耗时问题，也没有将它们误归为终点锁存。

部分试验启动附近有短暂 TF/odom/指令缺采或 service 重试，均保留 flags/anomalies 与空值，指标按原生时间窗口排除不合格样本。没有静默补零。终点误差是末次约 10 Hz 的 map TF 采样，不是 GoalChecker 判定瞬间的内部输入；例如 S04-005 yaw 为 0.262 rad，略大于配置 0.25 rad，未采集同一瞬间 GoalChecker 输入，因此不能声称所有外部采样误差严格在容差内，也不能据此改写真实 SUCCEEDED 结果。

S06 终点控制由 MyDWA `Rotating` 日志和零 Spin Action 交叉验证。缺少可靠 contact/碰撞及 footprint 障碍净间距指标，仍不能声称更安全。DWB 数字只作同配置历史参考，并非同期的新性能对照；小样本不支持统计显著优劣判断。S02 本轮未跑，V1 的结果不能标记成 V1.1 新回归。

## 冻结决策和工程交付

**接受最小生命周期修复，在本轮指定的五场景回归范围内冻结 V1.1 静态基线。** 本轮新增真实 Gazebo 试验 **14 次，14 次有效成功、恢复总数 0、进度失败总数 0**。确定性测试证明同终点新路径能释放失配的锁存；真实回归未复现 S04 恢复循环，S06 仍由 Controller 完成旋转，其他场景未见明显导航能力退化。可以进入受控的动态障碍开发，并继续用此静态基线做回归；这不构成安全认证或偶发故障永不再现的保证。

原 V1 参数、DWB 参数和所有旧数据不覆盖。核验了 39 次历史试验的 summary 哈希及新旧所有原始指标输入/配置快照。S04 的共同运行参数与资源签名与 R1 相同：`1f6c3101bc3e5eeebf9f6222b303b37bbd85ba7d1a69f63302c923447a5ae8ff`。软件包版本未变化。UI 启动脚本及除 Controller 生命周期外的 C++ 实现哈希保持原值；没有重新执行交互 GUI 验收。

| 标识 | SHA-256 |
|---|---|
| 原始 V1 MyDWA YAML / V1.1 同值 YAML | `0213f8ceb754196697c53c965789e6cc485f267a51d416576076df36920ee689` |
| 官方 DWB YAML | `21f6a890657da341a854efce403c895304a3ebc1a8f91f466348e720f2641380` |
| 修复后 controller.cpp | `bff50490db1b92b5c6a7d29a38f1cba4d62b81576f95f54e6db4c9cd5ae596ba` |
| 修复后实际加载插件 | `976c629c8dcf112d80cad7cd994b7840b1f6bfac518e1a9b51bb3fc630e7d4c7` |
| 修复前 R1 插件归档 | `78018c60ec1c24c5efd5931cb9218fa149bf6bff4fc4589907ff9f804b1eddb4` |

测试前后核验固定插件及源码哈希，并通过 Nav2 容器 `/proc/<pid>/maps` 核验实际加载的库；进程快照有 gzserver、没有 gzclient/RViz。全部试验 cleanup return code 为 0，最终域 77 的仿真/Nav2 相关进程审计为空。部分退出曾需清理器发送 SIGTERM/SIGKILL，因此不声称所有子进程均自行优雅退出。

文件修改/新增：

- 修改 `src/controller.cpp`：仅上述 setPlan 生命周期及输入/提交安全逻辑。
- 修改 `test/test_controller.cpp`：新增 6 项回归及调整旧契约预期。
- 新增 `benchmark/v1_1/`：本报告、README、计划、原/新代码与插件证据、测试日志、有限回归执行命令、只读分析和冻结清单。
- 新增 `benchmark/results/v1/v1_1_S04_001`～`005`、`v1_1_S06`、`v1_1_tracking`：14 次原始试验。没有修改现有 runner、启动脚本、地图、World、CMake、YAML 或核心评分实现。

实际执行的主要命令（WSL，已 source Humble 和 overlay）：

```bash
cd /home/user/ros2_ws
bash src/my_dwa_controller/benchmark/v1_1/red_tests.sh  # 旧实现上先验证红灯
colcon build --packages-select my_dwa_controller --symlink-install
ROS_DOMAIN_ID=97 colcon test --packages-select my_dwa_controller --event-handlers console_direct+
colcon test-result --test-result-base build/my_dwa_controller/test_results --verbose
bash src/my_dwa_controller/benchmark/v1_1/evidence/run_s04.sh
bash src/my_dwa_controller/benchmark/v1_1/evidence/run_remaining.sh
cd src/my_dwa_controller/benchmark
python3 v1_1/evidence/analyze.py > v1_1/evidence/analysis.log
python3 v1_1/evidence/capture_runtime.py final_process_audit.json
python3 v1_1/evidence/freeze.py
```

其中红灯脚本是历史执行记录，当前插件已修复，不应期望重跑它仍然失败。run-id 必须唯一；新试验重跑方法见 README，不覆盖验收目录。冻结配置 `configs/mydwa_static_v1_1.yaml` 与原参数完全相同，**必须结合新插件使用**，只更换 YAML 不会获得本修复。尚未解决项是控制循环耗时、短暂无可行轨迹、可靠安全几何指标、S02 的 V1.1 补测和更大样本可靠性估计；这些不作为本轮额外调优或重构的理由。
