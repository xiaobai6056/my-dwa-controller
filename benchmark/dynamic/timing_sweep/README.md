# D01 横穿启动时序扫描

目的：只改变动态盒横穿开始时间，观察相遇时序与碰撞/控制行为的关系，不修改算法或调参。原D01、V0.1/V0.2及Release对照记录均保留。

## 配置与实际执行

使用原D01地图、物理模型、机器人起点[0,0,0]、目标[4,0,0]、盒尺寸0.5×0.5×0.7m、x=1.6m、y=-1.1→1.1m、速度0.25m/s、90秒任务超时。原4s关键帧原样，其他时机只平移运动帧时间，运动段仍8.8秒。rules.scene(4)测试为与原D01完整字典相等。没有随机引入新的场景因素。

本轮全部重新测试：2/3/4/5/6秒 × MyDWA/DWB ×3，共30次。旧正式MyDWA是未优化构建，之前Release实验不是本轮正式扫描，故不混用旧4秒记录。MyDWA统一使用独立Release安装 `/home/user/ros2_ws/install_compile_comparison_20261020/release`，实际-O3/-DNDEBUG；DWB使用既有官方系统包，两者分别使用原冻结YAML。

每轮使用Headless、domain91、Gazebo master11436，完整新进程/AMCL/任务/运动状态，结束清理并间隔2秒。重复1按2,3,4,5,6秒；重复2逆序；重复3按4,2,6,3,5秒，交替Controller先后顺序。完整顺序、UTC/负载/affinity在sequence_first.json。

## 从干净WSL终端复现

```bash
cd /home/user/ros2_ws/src/my_dwa_controller
bash benchmark/dynamic/timing_sweep/run.sh --tag new_scan
```

不要同时启动另一套仿真。tag和每轮目录不可复用，脚本拒绝覆盖；新记录在timing_sweep/runs。没有修改默认安装。单次指定时机：

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
python3 benchmark/dynamic/timing_sweep/run_one.py --scenario D01 --controller mydwa --headless --start-time 3 --run-id single_t3_new
```

Controller可换dwb，时间取2/3/4/5/6。默认不加载性能探针（两者相同观察条件），仅复用V0.2独立cmd_probe记录原生cmd_vel_nav/cmd_vel。真实插件路径通过/proc/PID/maps校验并保存SHA256，不能只依赖类名。Release实际库SHA cceb8c0630836b22916da5e839176497acddabc38cb65028fa3283e5daca85ee；DWB库SHA见evidence/selected_binaries.json。

trial_entry.py调用原dynamic/run_trial.py main，只替换进程内load(D01)返回的有效场景，不复制整个测试框架；Motion、Observer、静态Collector、Action、评价器直接复用原实现。原runner固定写原安装MyDWA哈希，因此每轮保留summary_from_original_runner.json，并在新summary中明确original_underlay_mydwa_sha256与实际selected_plugin_path/sha256，避免用原安装哈希冒充Release；不改控制输入。

有效配置分别保存为run/effective_scene.json和trial/effective_scene.json。trial/configs仍保留原冻结脚本/场景文件快照；它们是原实现证据，实际本轮启动时间以effective_scene为准。所有raw日志/指令/GT/传感器/接触记录都保留。

## 启动时刻严格验证

每轮不仅检查配置和运动请求，还检查物理Ground Truth：

- 首条盒y相对初始变化>0.005m的观测时间，必须在配置启动时间[-0.05,+0.35]秒内。
- 启动前不应提前移动；x保持1.6m。
- 全段实际y必须在按接收时钟t±0.2s预测的位置包络内（再加0.01m几何容差）。这是对接收clock约0.1s量化和异步GT的保守验证，不宣称精确物理毫秒时间。
- 所有motion请求必须与同一有效轨迹插值一致，记录服务发送/确认/error及GT样本数。

结果在motion_validation.json；错误时机和缺少实际位移的反例回归测试必须失败。若任务在移动前结束，不能声称实际启动验证通过，标为未验证；不会当作有效避障样本。

## 统计和交互行为规则

离线执行（不启动Gazebo）：

```bash
python3 benchmark/dynamic/timing_sweep/analyze.py --tag new_scan
python3 benchmark/dynamic/timing_sweep/audit.py --tag new_scan
```

本次tag=first派生CSV/JSON/图在outputs，报告REPORT_TIMING_SWEEP.md；其他tag使用outputs_<tag>及REPORT_<tag>.md，避免覆盖本次派生结果。--tag应与运行时一致，默认first。--partial仅供中途检查，最终默认要求30份完整记录；离线分析只重建派生输出。

成功必须来自真实Nav2 SUCCEEDED且原评价器判为success；物理碰撞来自contact源stamp。完成时间只对成功求平均，失败/超时独列。最小净间距沿用原保守二维包络（机器人x[-.1965,.077]、y[-.153,.153]，盒0.5m正方形）而非中心距离，不能冒充精确三维距离。碰撞位置来自接触源时间附近GT接收样本，精度约0.1s。

显著减速/转向：导航后1.5秒起，原生cmd_vel_nav连续样本跨度≥0.2秒达到v≤0.21m/s或|w|≥0.1rad/s；空档>0.35s会中断持续判定。排除首次接触前最后0.15秒及机器人离开交叉区后的终点减速。先检查原生指令窗口覆盖（两端0.2秒、至少3个不同clock样本、最大间隙0.35秒），缺失或缺段时行为分类未知，不把None当作无反应。记录命令和实际速度变化，时刻均相对导航dispatch的接收/clock，不证明避障意图。

交叉区域时间证据：机器人真实旋转包络与固定盒x区间[1.35,1.85]重叠的时间窗；盒包络与名义路线y[-.153,.153]重叠的时间窗。不同占用窗相隔>0.2秒、最小gap>0.05m，且无持续减速/等待/明显横向偏移，才标记“无交叉区交互通过—机器人先通过/盒先通过”；这是狭义的无交叉区同时占用，不表示未感知障碍物或没有近距离经过，也不构成主动避障证据。如果出现等待造成时间分离，优先标“观测到等待后通过”；转向及明显横移/减速后通过按观测描述，不能仅凭成功叫主动避障。几何接近但机制不能证实标未知；碰撞中转向只能叫尝试/变化，不能叫成功避让。具体阈值、状态码与逐轮证据见报告/JSON。

## 验证和保护

```bash
bash benchmark/dynamic/timing_sweep/verify.sh
```

包含全部相关Python、独立Release构建现有8个C++ GTest目标和benchmark功能目标、语法检查；不重新编译业务代码。audit核对3575个既有文件/原安装/Release/DWB哈希、用户Git差异、真实加载、全部运行参数、有效轨迹、原始来源一致性。归档包含全部新原始记录及配置/脚本/证据。没有git add/commit/push，没有筛选重跑。

样本每格只有3次；WSL/AMCL/物理与调度仍有波动，固定sim-time启动会因Controller进度不同而产生不同相遇状态（正是本实验需记录的因素）。不能据此做显著性或全面性能排名；“无交互通过”不能计作已证明成功避障。未记录每周期候选轨迹/评分，不能据最终速度推断内部原因。
本次完整原始记录与库/编译证据归档为 outputs/raw_timing_sweep.tar.gz，SHA和成员数量在 outputs/archive_manifest.json。31张图含碰撞率关系图及每轮4面板交互图。
