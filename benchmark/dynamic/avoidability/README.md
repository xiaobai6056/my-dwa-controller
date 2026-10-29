# D01 avoidability and decision diagnosis

独立研究模块；不修改既有控制逻辑、参数、世界或正式数据。结论和证据边界见 [REPORT_AVOIDABILITY.md](REPORT_AVOIDABILITY.md)。

## 从干净 WSL 终端复现

```bash
cd ${REPO_ROOT}
source /opt/ros/humble/setup.bash
source ${MYDWA_WS}/install/setup.bash
source ${MYDWA_WS}/install_compile_comparison_20261020/release/share/my_dwa_controller/local_setup.bash
D=benchmark/dynamic/avoidability

# 保持既有Release库及历史原始数据；不需要重建Controller。
sha256sum ${MYDWA_WS}/install_compile_comparison_20261020/release/lib/libmy_dwa_controller.so
# expected cceb8c0630836b22916da5e839176497acddabc38cb65028fa3283e5daca85ee

python3 "$D/analyze_offline.py"
bash "$D/build_probe.sh"       # 只构建外部观察器，-O2不影响Controller的-O3
bash "$D/calibrate_probe.sh"   # 原返回/异常转发及读回校验
bash "$D/verify.sh"

# 仅2个横穿时机、3个独立运动验证+2个Nav2诊断；使用新的唯一标签。
bash "$D/run_minimal.sh" reproduce_001
```

需要原有 ROS Humble、Gazebo Classic、Nav2/TurtleBot3、numpy/matplotlib 与当前工作区安装。原始输入为 `benchmark/dynamic/timing_sweep/outputs/results.json` 及其所有 runs（2–5s共24条），不联网、不引入新大型依赖。默认独占使用ROS_DOMAIN_ID91（Nav2）及92（独立运动），端口11345+domain，启动前原有baseline清理工具可能结束这些domain的旧仿真，请勿同时运行其它benchmark。

已执行的原始运行名是 `validation_t2_early_fixed`、`validation_t4_early`、`validation_t4_late`、`diagnostic_t2_linked`、`diagnostic_t4_linked`。另外两个基础设施失败目录全部保留。默认分析器按上述交付数据生成报告；要分析新一批数据，可给 `analyze_validation.py` 与 `analyze_decisions.py` 指定新标签：

```bash
python3 "$D/analyze_validation.py" --tag reproduce_001
python3 "$D/analyze_decisions.py" --tag reproduce_001
# 新标签分析写入outputs_reproduce_001，不覆盖本次outputs或原始runs。

```

单次入口（必须使用新run-id；已存在则拒绝）：

```bash
bash "$D/validate.sh" --start 2 --brake 4.0 --run-id unique_t2_early
bash "$D/validate.sh" --start 4 --brake 4.6 --run-id unique_t4_early
bash "$D/validate.sh" --start 4 --brake 6.0 --run-id unique_t4_late
bash "$D/diagnostic.sh" --start-time 4 --run-id unique_diagnostic_t4
```

独立验证没有Nav2 Action，不加载MyDWA/DWB，不属于导航成功率；只用真实底盘cmd_vel验证策略可执行性。场景world、模型SDF及障碍Motion与原D01一致，横穿启动使用原timing_sweep适配器，只替换时间。Nav2诊断使用原框架、冻结YAML和Release，记录实际插件maps/哈希/参数；实际输入来自正常激光、里程计、TF、Costmap与路径。ELF探针读取实际锁内网格，不读取GT；GT只用于评分/离线反事实。

## 脚本和数据

- `kinematics.py`：从实际SDF提取完整水平包络；SAT间隙下界；保持v/w或制动后等待，障碍持续运动直到穿完；可选轮加速度协调约束。
- `analyze_offline.py`：重用24次旧GT，生成每0.1s策略风险、逐轮制动过渡格及风险曲线；0.03/0.12m是条件余量和假设敏感性。
- `validate_motion.py` / `validate.sh`：Gazebo-only运动入口，唯一cmd_vel发布者检查、源时间戳odom、GT/物理接触/命令/实际障碍启动验证、超时及进程清理。
- `decision_probe.cpp` / `build_probe.sh` / fixture：外部观察器及转发校验；候选全量可行性/评分、最优完整轨迹和实际Costmap，每周期原始JSONL。
- `run_diagnostic.py` / `diagnostic.sh`：新监督器适配，复用原launch和trial_entry，不改旧文件。
- `analyze_decisions.py`：逐周期计数、全部加权评分、最优选择与最优轨迹地图代价一致性抽查/全检；支持CSV/JSON及原始Costmap图。候选全部轨迹未保存，未独立重算每条候选的地图统计。
- `analyze_validation.py`：持续停稳、物理接触、受控通过与预测对照。
- `runs/`：每次独立进程原始数据，包括失败；`outputs/`：CSV/JSON/PNG；`evidence/`：冻结哈希、fixture、构建、测试和失败日志。
- `audit.py`：5007个既有文件及Git diff/状态审计；`make_report.py`：生成本次交付报告。

## 定义和限制

真实碰撞=Gazebo接触；“安全”=指定假设、数值采样与保守包络下的动态障碍无重叠，不能当精确物理证明。SAT正值为欧氏净间距下界，负值只指包络重叠。原GT缺源时间戳，约0.1s接收时钟格和障碍约±0.2s调度验证范围限制精度；窗口不是精确最后物理制动时刻，0.12m不是经标定的严格误差界。完整地面动力学、未来控制闭环、静态墙和全操纵可达集未求解。

观察器同步写JSON会增加控制周期耗时；compute和serialize字段没有覆盖全部flush开销。完整额外开销未知；这些诊断不能用于性能对比或推断运行间差异原因。原benchmark和Release性能报告保持独立。报告中的4.744s等只用于定位周期，不意味着毫秒级物理阈值。

新实验复现不依赖第三方代码；无Controller/Scorer优化、无参数修改、无commit/push。重新构建插件或改变旧输入必须新建研究目录，不能冒充本次同条件证据。