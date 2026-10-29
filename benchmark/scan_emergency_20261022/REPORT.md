已完成本次定向修复，并仅新增两次 Gazebo 闭环：S05 一次、D01 2s 一次，均到达目标；D01 无接触、已解除原先无法恢复的风险。未增加 D01 4s。本结果只支持当前单个横穿方柱条件，不能宣称动态避障已经普遍解决。

时间均为导航派发后的仿真秒，原始 ROS 时间可通过每轮 summary 的 navigation_start_sim_s 换算。源码与日志以 WSL `${REPO_ROOT}` 为准，本目录是独立交付副本。

**根因证据与修正对应关系**

| 层次 | 复现证据 | 本次修改 |
|---|---|---|
| 感知表面变化 | 旧第三轮 elapsed 5.12–5.72：xmin 约 1.334–1.336m 不动，xmax 从 1.706 缩到 1.370m，却拟合出 vx 约 −0.30m/s，confidence 仍约 0.71 | 保留原拟合输出供审计；同一窗口内可见支撑尺寸变化过大则 motion=unknown，拟合残差小不能独立认定真实运动 |
| 真横向运动 | 同期 Y 两侧支撑随方柱向 +Y 移动；场景是 0.5×0.5m 方柱、x=1.6m、+Y 0.25m/s | 只接纳路径前方附近、连续且有效的法向运动；沿墙运动/未知表面不新建动态风险 |
| 风险生命周期 | 旧 moving_seen 对象过期后 unresolved_ 永久置位；12s 后 679 条风险记录均 unresolved=1 | 删除普通 Cluster 对永久风险的所有权；Monitor 保留一个固定的原冲突/接近区域，与 Cluster ID 分离 |
| 观测覆盖 | 旧 bag 的 528 点扫掠区域审计在 10.12s 首次全部有限射线可见且局部地图清空，12s 后 393 帧通过；整个前方 Outer corridor 却因滚动地图末端持续 unknown | 局部风险区独立取证，全部接近/穿越带用新鲜有限 Scan 射线证明，穿越带全部 Costmap 格已知且低于 INSCRIBED；不以远端地图缺失永久阻塞 |
| 额外恢复门槛 | 中间离线版本有 151 帧 area_clear=1 仍不能解除；15.2s 时 v=−0.000303m/s，而几何 yaw/路径方向仍对齐，velocity.v>=0 使 path_aligned=false | Scan 模式几何对齐不混入速度符号；停车仍检查制动资格，并只容忍停止阈值内微小反向噪声。原区域清空计数及恢复不依赖当前重规划/朝向 |
| 几何覆盖审查 | 旋转采样格点会遗漏局部 Costmap 单元；最初区域也未包住接受的纵向短时位移 | 单独枚举全部相交地图格和出界格；冻结区包含两秒已接受的纵向扫掠，横向接近带使用实际支撑投影而非对角半径 |

原路径区域在 2.32s、障碍进入之前就可能是空的。因此没有使用“当前路径暂时无占据”或固定超时来清障。`DIAGNOSIS.md` 在实现之前保存；原始证据在 `prior_surface_support.csv`、`prior_sweep_visibility.csv`、`alignment_gate_audit.csv`，对照旧 `final_manifest.json`、tracks/risk/actor CSV 和两张旧图完成。

**实际改动**

- `ScanObstacleTracker` 沿用精确扫描时间 TF、聚类和轻量关联，增加 Unknown/Stationary/Moving、支撑尺寸变化和有限测距射线。未成熟、表面切换、关联/观测失效保持未知；没有新增跟踪框架。
- `PathConflictMonitor` 接收可信横向趋势，首次冲突固定原路径切线下的接近/穿越矩形。普通对象消失不解除风险，连续新观测可以在原 ID 丢失后证明区域清空。地图格检查和射线检查分开。
- `RiskGovernor` 继续是唯一清障计数与速度状态机：三个不同 Scan 序号连续清空后恢复，恢复到 Normal 后反馈释放区域。未知/遮挡不清除风险，停车不成立也不会无条件恢复限速。
- Controller 只增加释放反馈和诊断字段；本次没有修改 DWA 采样、碰撞检查、评分或既有软偏好。开关默认仍关闭，原 YAML 未改，Scan 关闭时原停车/恢复分支保持。

具体实现契约见 `docs/scan_emergency_v02.md`（包含在源码包）；本次增量为 `incremental.patch`，开始时的全部未提交源文件保存在 `sources_before.zip`。

**运动估计与时间对齐**

| 新 D01 2s 的实际样本 | 结果 |
|---|---|
| 连续有效段 | Scan 源时间 2.901–4.901s，11 帧，最大间隔 0.200s，方向全部为 +Y |
| 连续段二维速度 | 中位 vx=0.0015、vy=0.2449m/s，真值 (0,0.25)m/s；二维 RMSE=0.02369m/s |
| 全部被接受的横向运动 | 17 帧，vy 范围 0.2172–0.2631m/s，二维 RMSE=0.02343m/s，90% 速度误差约 0.0384m/s |
| 未筛除的原始拟合 | 同一运动区间 28 帧，二维 RMSE=0.13273m/s；包含表面切换误差，不能都当作可信预测 |
| 有效范围 | 接受样本横跨 2.901–7.501s，但中间有 1.4s 缺口，缺口输出未知；没有声称全程连续可靠 |
| 位置与时效 | 接受样本可见簇位置相对真值中心的中位误差约 0.037m；观测 age 为 0.099–0.199s；0.8s 窗口拟合对起动有延迟 |
| TF / 队列 | base_scan，约 5Hz，time_increment=0；运行 TF 失败计数 0，Scan 队列累计丢弃 10 帧，仍需保留负载下丢帧风险 |

17 个接受样本全部参与 Monitor，11 个产生 conflict=1。第一次动态冲突时 Costmap Inner core=0，日志已有 predicted=1、inner=1，证明运动信息实际改变冲突证据。不能仅依据方向正确认定整个物体轨迹可预测；支撑稳定性是针对当前方柱的短时启发式。

**闭环行为**

| 条件 | 实测结果 |
|---|---|
| S05，round1 | SUCCEEDED，导航用时 31.9s；312 条风险诊断全部 Normal；接受的运动簇 0、动态路径冲突 0、异常风险触发 0 |
| D01 2s，round2 | SUCCEEDED，用时 23.9s；无接触记录、无等待超时 |
| 3.0s，进入保护 | Scan stamp=2.901s，predicted=1 且 core=0/0；available=0.233m、stop=0.204m，cap 从 0.260 降至 0.245m/s |
| 3.1–3.6s，停车资格变化 | Braking/Fallback 数次切换。3.1s 时 available=0.185905m < stopping=0.185916m；未放宽比较。最终 fallback 保留约 0.155m/s 上限及原软偏好/DWA 约束 |
| 6.1s，线速度停车 | 首次 abs(v)<0.01m/s，机器人 (x,y)≈(1.112,0.010)m；6.2–11.3s 的最大 abs(v)约 0.0102m/s |
| 等待性质 | 没有进入 Governor Wait。DWA fallback 内仍转向，该区间最大 abs(omega)约 1.035rad/s；因此不能称为经过认证的稳定全静止等待 |
| 11.0s，首次局部清空 | 原冻结区 center≈(1.6054,−0.00014)、半长 0.3170m、横向 [−1.18495,0.28414]；area_known=1、rays_clear=1、map_clear=1、predicted=0。此时完整 corridor 仍 known=0 |
| 11.4s，恢复 | 三个不同 Scan 连续清空后 Recovering，cap 开始提高；没有依赖原障碍 ID 持续存在或固定等待时长 |
| 11.7s，释放 | Normal、release=1、cap=0.260；11.8s 记录到 v=0.187m/s；随后持续行驶至目标 |

新 D01 原始证据：`runs/round2_D01_t2/launch.log` 行 386 首次制动；394 首次停车资格失败；595 首次局部清空；615 恢复；630 释放。`actor_aligned.csv`、`risk_all.csv`、`ground_truth_aligned.csv`、原 bag 和 collector JSONL 全部保留。`motion_and_control.png/pdf` 将未知伪速度、限速、实际速度和风险状态分别绘出。

**离线回归与工程检查**

原始第三轮 bag 导出 408 帧输入，直接调用实际 C++ Tracker/Monitor/Governor。最终回放 2.6s 制动、11.8s 首次局部清空、12.2s 恢复、13.2s 释放；不再因旧 ID 消失永久锁存。回放保持旧机器人运动且每 Scan 决策一次，时序不能替代上述新闭环；输入/时间对齐限制见 `REPRODUCE.md`。

新增 11 个 C++ 回归用例覆盖可见支撑切换、静态/纵向排除、未知与丢 ID 保持、射线遮挡、过期、地图未知、旋转漏格、纵向扫掠、微小反向里程计及独立区域清空。新增 4 个原 bag 离线回归检查真实趋势、伪速度未知、清障释放和后段无新横穿。对应失败日志先保存，再实施修复。

| 检查 | 实际结果 / 日志 |
|---|---|
| 独立构建 | 通过；`build_prelive.log`，编译无警告 |
| 功能 CTest | 12/12，通过；`ctest_functional.log` |
| C++ 单元用例 | 271/271，通过（原 260，加本次 11）；`verification.json` 的逐目标 XML 统计 |
| 原始 bag 原生回放 / 离线断言 | 执行成功，4/4 通过；`replay_final.log`、`offline_final.log` |
| 完整 CTest | 15/18；pep257、flake8、uncrustify 失败，返回码 8；`ctest_full_final.log`。未清理旧格式债，不能写成全部通过 |
| 定向 C++ 格式 | 11 个变更/新增 C++ 文件通过；`format_final.log` |
| 定向 Python | 本次 6 个工具 flake8 和 pep257 通过；`python_all_final.log`、`python_doc_final.log` |
| 代码审查 | 两项几何问题修复后，只读复审未发现新的 P1/P2；`review.txt` |

开发阶段曾有诊断日志格式字符串缺少参数导致 Controller 测试崩溃，已补齐；后续 64 个 Controller 用例全部通过。离线初次恢复断言失败定位为区域尺寸及 path_aligned 清障门槛，未用同时调整多个阈值反复试跑。Gazebo 仅本次两次启动，均有效、运行时插件映射正确，没有无效预启动或追加 4s。

**版本、保护及交付**

Git HEAD 为 `1bad3b66aa64384d5955f1933dcfb5c1017dcd62`，工作树保留未提交状态。两轮 `manifest.json` 均有参数副本 SHA256、源码 SHA256、插件副本和 `/proc` 运行时映射；两轮 source_sha256 均与最终源码一致。

插件 SHA256：`1bb5e6eeb912c9c5edb174c3d4602f69e1755563577bbd049371555ffabcb967`。

构建前缀 `${MYDWA_WS}/build_scan_emergency`，安装前缀 `${MYDWA_WS}/install_scan_emergency/my_dwa_controller`，日志前缀 `log_scan_emergency`。旧 install_scan_trend 插件 hash 仍为 `af3ea574…183605`（完整值在 manifest）。开始时 7055 个文件中仅 11 个预期源码/构建/测试文件修改；7008 个旧 Benchmark/配置文件 hash 无变化，无删除或覆盖原结果。

`sources_before.zip` / `sources_final.zip` / `incremental.patch` 提供源版本关系；两轮插件和 sources.zip 分别归档，`final_manifest.json` 可逐项验证。完整日志、bag、参数、结果与源码增量都在本目录，原正式场景及旧归档不在修改范围。

**尚未解决与建议**

当前结果解决了这次可复现的伪速度接受和风险无法解除问题，并证明短时趋势实际进入冲突与速度控制。仍需优先处理停车认证对路径首点重投影和边界数值的敏感性：本次仍靠 fallback/DWA 完成线速度停车，伴有转向，不能据此保证所有横穿条件能在设定停车余量前静止。

下一步先用已存 bag 离线核对固定区域投影距离、机器人行进距离和离散制动模型的一致性，再决定是否改停车几何；不先放宽停车阈值。支撑变化门控会丢失部分真实运动，单个固定区域也不承担多障碍、曲线路径远期预测或复杂遮挡重入。区域始终不可见时仍应保持保护，需要上层恢复策略获取新观测，不能以超时假定安全。本次每场景仅一次，尚无重复性统计与跨形状泛化证据。
