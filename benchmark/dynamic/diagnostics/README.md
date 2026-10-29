# Dynamic Benchmark V0.2：GUI 与诊断

目的：在冻结 D01/D02、Controller/Nav2 参数的前提下观察导航，离线解释 D01 物理接触时序，并测量两种插件的计算开销。正式 V0.1 数据不改写，本模块不运行新的12轮批量基准，不改进算法。

## 一条命令打开 Gazebo + RViz

从 WSL Ubuntu 干净终端执行（当前 WSLg 已验证）：

```bash
cd ${REPO_ROOT}
bash benchmark/dynamic/diagnostics/run_once.sh --scenario D01 --controller mydwa --gui
```

脚本自动 source ROS2 Humble 与工作区。无需先启动旧 UI；每轮使用独立仿真进程、原有 preflight 锁、domain91/master11436及清理函数。不要同时运行另一套仿真。Gazebo自动俯视，RViz订阅已实测存在的 `/tf`、`/robot_description`、`/scan`、`/map`、`/local_costmap/costmap`、`/plan`，并显示机器人位置。没有凭空新增预测轨迹话题。

切换模式/场景/插件：

```bash
bash benchmark/dynamic/diagnostics/run_once.sh --scenario D01 --controller dwb --gui --hold-seconds 180
bash benchmark/dynamic/diagnostics/run_once.sh --scenario D02 --controller mydwa --gui
bash benchmark/dynamic/diagnostics/run_once.sh --scenario D02 --controller dwb --headless
```

GUI默认结束后留60秒查看；可设 `--hold-seconds 300`，Ctrl+C会清理。结果在 `benchmark/dynamic/diagnostics/runs/<UTC运行编号>/`，也可 `--run-id my_new_id`；已有目录会拒绝覆盖。终态/物理接触/日志保留在 metrics.json、trial/summary.json、launch.log和原始jsonl。**进程exit0只表示完整记录了该次测量，collision/CANCELED仍是导航失败**；基础设施失败/未完成返回非零。活动导航中中断返回130；结果保存后的查看等待中断可正常清理退出。

D01和D02完全复用 `../scenarios.json`、`../assets`、`../motion.py`、`../run_trial.py`。GUI与Headless只切换显示与相机；动态盒依旧是物理碰撞与激光可见模型，经 `/scan`→原Nav2 Costmap→Controller正常输入链路。Ground Truth和接触只进入独立评价。

实测：D01/MyDWA GUI两次（第二次复核自动视角），D01/DWB GUI一次，均accepted目标后发生物理接触并CANCELED；窗口、激光、地图、Local Costmap、路径与盒/机器人真实场景已检查。截图在 `outputs/gui/`。D02选项沿用同一逻辑，本轮未重跑D02 GUI/Recovery。

WSLg环境需要DISPLAY和可用图形设备。当前无需强制软件渲染；环境证据见evidence/provenance.json。首次GUI视角被高墙遮挡，现camera_view.py仅发送Gazebo GUI相机消息；失败会记录camera.log，不改变场景。GUI startup logs与Headless日志独立保存。

## 重现六轮正式 D01 的离线图

系统已有Python3、matplotlib；不用启动ROS/Gazebo：

```bash
cd ${REPO_ROOT}
python3 benchmark/dynamic/diagnostics/analyze.py
python3 benchmark/dynamic/diagnostics/make_report.py
```

生成 `outputs/v0_1_analysis/{mydwa,dwb}/trial_00{1,2,3}` 的 events.json 与 speed/trajectory/clearance/timeline.png，各图标注来源/场景/Controller/运行编号；汇总 `outputs/events.csv`。`--source`/`--output`可改离线源与派生输出目录，默认只重建派生图表，不写原始数据。make_report需要当前已交付的两轮perf诊断汇总（见下）；它生成 REPORT_V0_2.md、outputs/performance.csv/json/png。图可在Windows直接打开。

事件规则与时钟限制详见报告第3节。关键边界：T0为发送时接收clock；T3/T4为采集开始后首条保留的盒关联扫描/100值致命栅格，真实感知起始可能更早，不能计算感知延迟；T5/T6使用显著持续变化阈值与接触前保护窗口，未检出不等于没有短暂变化；T7为contact源stamp而非Action取消时刻。净间距采用原有二维保守包络，是近似值。

## 同口径性能复现（少量诊断）

已完成的数据无需重新导航，只读取固定mmap槽：

```bash
python3 benchmark/dynamic/diagnostics/read_perf.py benchmark/dynamic/diagnostics/runs/perf_mydwa_001
python3 benchmark/dynamic/diagnostics/read_perf.py benchmark/dynamic/diagnostics/runs/perf_dwb_001
python3 benchmark/dynamic/diagnostics/make_report.py
```

需要另行测量时，仅编译外部观察器；不重建/优化原插件：

```bash
cd ${REPO_ROOT}
bash benchmark/dynamic/diagnostics/build_probe.sh
source /opt/ros/humble/setup.bash
source ${MYDWA_WS}/install/setup.bash
python3 benchmark/dynamic/diagnostics/evidence/test_probe.py
bash benchmark/dynamic/diagnostics/run_once.sh --scenario D01 --controller mydwa --headless --run-id perf_mydwa_new --preload "$PWD/benchmark/dynamic/diagnostics/perf_build/libnavperf.so"
python3 benchmark/dynamic/diagnostics/read_perf.py benchmark/dynamic/diagnostics/runs/perf_mydwa_new
bash benchmark/dynamic/diagnostics/run_once.sh --scenario D01 --controller dwb --headless --run-id perf_dwb_new --preload "$PWD/benchmark/dynamic/diagnostics/perf_build/libnavperf.so"
python3 benchmark/dynamic/diagnostics/read_perf.py benchmark/dynamic/diagnostics/runs/perf_dwb_new
```

探针是Linux x86_64、当前GCC/ROS ABI专用，当前插件符号与结构已验证；更换编译器/插件版本后须重新校验，不能盲目沿用。目标是单个串行FollowPath Controller：初始化尚未保证并发安全，不支持并发插件调用。转发校验确认4002原调用与2个异常保留，5轮校准新增开销约1.26µs/调用（复验1.29µs），不是普遍误差上界。固定16384槽，溢出计数、未提交槽及异常均保留；没有有效样本时解析器直接失败。

记录wall steady_clock、threadCPU、入口频率、原生输出接收频率与警告数。MyDWA既有分段复制到内存，DWB仅外层同口径；速度采样独立分段、DWB内部分段未实现，不当作0。新运行保存插件/探针哈希。已测探针源码和二进制在evidence/measured_probe，后补统一来源证据在evidence/provenance.json；原运行manifest不覆盖。

当前MyDWA核心库没有-O优化编译标志；本次保持这一条件。系统DWB包与自研构建条件不同，因此不把测量当作同等优化条件下算法效率排名。GUI运行不混入Headless性能分布；当前源码/二进制与V0.1不同，不混入正式基准。

## 验证及原入口兼容性

```bash
cd ${REPO_ROOT}
source /opt/ros/humble/setup.bash
source ${MYDWA_WS}/install/setup.bash
python3 -m unittest discover -s benchmark -p 'test_*.py' -v
ctest --test-dir ${MYDWA_WS}/build/my_dwa_controller -R '^test_' --output-on-failure
python3 -m compileall -q benchmark/dynamic/diagnostics
bash -n benchmark/dynamic/diagnostics/{run_once,launch_scene,build_probe}.sh
python3 benchmark/dynamic/diagnostics/evidence/audit.py
```

旧入口完全不改，已实际兼容验证如下单次运行；不要复用已存在的run-id：

```bash
bash benchmark/dynamic/run.sh --scenes D01 --controllers dwb --repeats 1 --run-id v02_headless_compat_NEW
```

本次兼容记录在 `benchmark/results/dynamic/v02_headless_compat_001/`。evidence/audit.py核对初始冻结源文件/原始记录哈希、真实插件加载、全runtime参数和最早contact时间。该审计默认针对本次固定5轮诊断；不用于未来不同场景的参数批量比较。

交付原始诊断归档 `outputs/diagnostic_runs.tar.gz` 含 runs/ 和 headless_compat/，其中都是补充诊断，V0.1原始正式记录不重复拷贝也不修改。runs/与perf_build/默认Git忽略，归档、源码、报告、图表及evidence可纳入版本管理；本次未自动git add/commit/push。evidence中初始失败的测试/构建日志也保留，最终通过记录另存。

## 文件职责与已知限制

- run_once.* / launch_scene.sh / dynamic.rviz / camera_view.py：单次启动、GUI、查看等待、超时监督、清理和结果保存。
- analyze.py：原始六轮D01独立关联、事件检测及24张图；make_report.py：表格、CSV/JSON、性能图与报告。
- perf_probe.cpp / build_probe.sh / read_perf.py / cmd_probe.py：外部计时、只订阅原生速度指令及离线耗时分析。
- test_diagnostics.py：规则、转发外围编排失败状态和观测关联测试；evidence/：初始快照、校准、审计、版本与测试日志。

每轮样本少且只一次性能对照，未控制所有CPU频率/缓存/系统负载。旧记录缺乏每周期候选轨迹/评分和内部Costmap读取时间，不能确定评分为何选前进，也不能证明瓶颈造成碰撞。没有进一步调查D02恢复链路；没有做算法优化。详细事实、推断和不可确定项见REPORT_V0_2.md。