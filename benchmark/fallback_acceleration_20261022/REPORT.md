# 危险回退期间避免不必要加速：小规模验证

**结论：候选选择层面的想法可行，尚未证明避撞改善。** 保留了一个默认随Path Conflict关闭的实验实现。没有增加检测规则、调整评分权重/公式、修改场景或参数。修改版2s、两次4s全部仍碰撞，不能把速度更保守解释为动态安全。

## 做了什么

直接硬限速可能删掉唯一可行的加速避让轨迹；增加速度罚分又需要权重，仍不能确保不加速。因此选择软偏好：

1. `RiskGovernor`在DwaFallback（含待清障确认阶段）给出 `preferred_max_linear_speed=min(max(当前v,0),原max_v)`。硬上限仍为原上限；不允许未经认证进入WAIT。
2. Controller保留全部原始DWA速度/转向候选。如粗采样没有恰好覆盖软上限，补充该速度的一行原角速度样本，最多增加约一行工作量。
3. 所有候选先经过原终点约束和碰撞检查。有不超过软上限的可行候选时，优先在这一组按**原评分与原角速度平分规则**选优；这一组全部不可行才允许原加速候选。全部不可行仍抛原异常，不能靠直接输出零速绕过检查。
4. 连续有效清障确认后解除偏好；终点控制、外部限速、控制加速度、默认关闭行为不变。无新参数。

改动文件：`include/my_dwa_controller/risk_governor.hpp`、`src/risk_governor.cpp`增加软偏好输出；`controller.hpp/.cpp`编排补样与选择优先级；`controller_diagnostics.hpp/.cpp`追加可行优先候选数量、软上限及是否选择加速退路；原有三个GTest文件补充验证。Monitor、Sampler、Predictor、ObstacleEvaluator、Scorer和原配置未改。`docs/path_conflict_v01.md`同步描述。

注意：这里的“不必要”严格指**已有当前地图判据下，存在不加速的可行采样候选**，并不表示真正动态场景中加速一定没有避撞价值。

## Gazebo结果：全部保留，无挑选成功样本

复用现有`run_smoke.py`、timing_sweep和静态场景入口，仅用本目录`run_comparison.py`选择独立安装前缀/输出位置。原场景、原YAML不变，仅在每次生成的参数副本启用原Path Conflict开关。全部8次均valid=true、库映射验证通过、清理返回0。

|条件|当前对照版|软偏好版|解释|
|---|---|---|---|
|D01 2s|碰撞，5.90s|碰撞，6.51s|修改版4.50s进入Fallback；10条成功选择日志均未选择加速，保留慢移和转向；另有无可行候选异常。|
|D01 4s|碰撞，7.01s|首次碰撞7.11s；针对触发不稳定仅重复一次，仍碰撞7.11s|首次修改版碰撞前没有Fallback，重复版6.90s才进入；并未稳定复现旧日志中0.12m/s限速后回退的低速阶段。|
|D01 6s|未新增对照|成功，导航15.0s|未触发风险控制。|
|S04|未新增同版本对照|成功，导航21.0s|5.70～15.10s处于Fallback；44条选择记录的线速度0.254225～0.26m/s。不能称静态零触发。|
|S05|未新增同版本对照|成功，导航31.4s|未触发风险控制。|

碰撞时间相对各次导航dispatch，取接触消息自身stamp。独立运行的扫描/定位/规划状态不同，**不是同步反事实**，不能将2s接触晚0.61s归因于本策略，也不能仅凭各一次成功宣称静态性能没有退化。静态采集器未保存contact真值，S04/S05的“成功”指原Benchmark导航成功。

## 决策确实发生了变化

修改版2s的 [launch.log:343](//wsl.localhost/Ubuntu-22.04/home/user/ros2_ws/src/my_dwa_controller/benchmark/fallback_acceleration_20261022/runs/preference_D01_t2/launch.log:343)：绝对仿真8.1s，即导航后5.1s，输入v=0.016732；84条不加速候选可行。最终选v=0.016732、总分0.531486，而保留的更快前进候选v=0.022379、总分0.524127更低。两者mean/max障碍代价都为234。说明新的优先级确实压过了原加权分数对更快候选的偏好，而没有将该候选删掉。行355还有同类证据：输入/选择0.011496，更快前进候选0.014368的分数更低。

全部修改版日志共55条成功Fallback选择记录（2s 10、4s重复1、S04 44），均满足选择速度≤软上限；均未选择加速退路。它们按不同地图状态记录，**不是每个20Hz周期的全量证据**。Gazebo未出现“低速组全部失效、加速组仍可行”的成功退路案例；这一分支仅在真实插件单测中验证。

![独立运行速度对照](//wsl.localhost/Ubuntu-22.04/home/user/ros2_ws/src/my_dwa_controller/benchmark/fallback_acceleration_20261022/velocity_comparison.png)

命令使用接收clock，odom使用源stamp，不能用两条曲线的间距精确估计执行延迟。虚线为首次Fallback，红线为接触。右下那次未触发Fallback，不能拿来证明选择策略失效或有效。

![已记录的Fallback选择](//wsl.localhost/Ubuntu-22.04/home/user/ros2_ws/src/my_dwa_controller/benchmark/fallback_acceleration_20261022/selection_evidence.png)

## 代码与测试证据

新增行为测试先在旧实现失败：有可行低速候选仍输出更快速度，见`test_red.log`。最终功能测试覆盖：粗网格补边界、零线速度时保留角速度窗口、低速候选全阻塞时仍能选原加速轨迹、清障前保持偏好与清障后释放、外部上限、诊断字段。加速退路用较密轨迹及更细采样复核，避免靠跳过薄障碍构造“可行”。原有全部不可行时异常检查、终点链路和225组关闭功能签名继续复用。

Release构建成功；247个C++用例、全部11个功能CTest目标通过。完整CTest为14/17，原有pep257、flake8、uncrustify三项失败，不能称整包全绿。本轮新增Python脚本的flake8/pep257通过，修改C++文件剩余格式差异均在原有代码行。输出见`build_final_tests.log`、`tests_final_functional.log`、`tests_full.log`。独立只读代码复核未发现需要修改的缺陷；其指出的零速和诊断字段覆盖已补充，全部不可行异常已有原测试覆盖。

Gazebo对照插件SHA256为 `54edf237e23192d137fb703821c88379dbd662d453571959bd15dd13697e50e5`；修改插件为 `54fbea7ee2db06a484c7238cfcacca40e80990c2cafd3d5bd04b3089e52dcc78`。两者源码分别保存在`sources_before.zip`和`sources_gazebo.zip`，每次manifest记录实际源哈希、库映射、场景、参数和命令。闭环后只对Controller新增lambda做排版换行，业务逻辑完全相同，但最终库哈希不同，记录于`final_manifest.json`；最终排版版本通过功能测试，未再重跑Gazebo。

## 局限与下一步判断

- 这个修改抑制的是**命令相对当前测得速度的加速**，不是严格单调速度天花板。odom/执行误差可使下一周期软上限轻微回升；本轮没有再加锁存阈值或滞回规则。
- 当前Costmap静态轨迹可行，不保证动态障碍未来不会撞上慢行或停止机器人。本轮2s正是低速转向后仍碰撞；这证明“抑制再加速”不足以解决D01。
- 4s触发阶段变化很大；本轮无法给出原0.12→0.26反弹在相同地图输入下的Gazebo反事实。单测证明选择规则，不证明真实避撞。
- S04的Fallback约9.4s既包含当前冲突，也可能包含待确认清障，不能把整段解释成动态障碍或持续无法停车。此轮没有增加静态排除条件。
- 日志输出、地图复制及全循环压力未做系统性能评估；增加的采样最多是一行，不能据此推断任意地图均满足50ms周期。

建议仅作为默认关闭的可行性原型保留。下一步若评估避撞收益，应优先在相同地图/路径/速度输入下保存候选并比较选择，再选择一次能稳定进入“低速且停车不可认证”阶段的闭环复核；不应继续堆叠检测条件或根据障碍已知启动时序触发策略。本轮不宣称已解决动态碰撞。

## 复现、数据与回滚

所有新增文件位于本目录；旧实验不覆盖。`results.json/csv`包括8次全部结果，`*_risk.csv`带原始日志行号，`*_selections.csv`为选择证据。三份`*_command.txt`保存实际运行命令。`before.json`记录5899个初始文件哈希，`final_manifest.json`记录保护核验、最终构建版本与源码差异，`incremental.patch`仅覆盖本轮改动。

```bash
source /opt/ros/humble/setup.bash
cd /home/user/ros2_ws
colcon --log-base log_fallback_acceleration build --build-base build_fallback_acceleration --install-base install_fallback_acceleration --packages-select my_dwa_controller --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install_fallback_acceleration/setup.bash
cd build_fallback_acceleration/my_dwa_controller
ctest -R test_ --output-on-failure
cd /home/user/ros2_ws/src/my_dwa_controller/benchmark/fallback_acceleration_20261022
python3 -B analyze.py
```

运行命令中的tag输出目录已存在；复现Gazebo应换新tag，不覆盖记录。旧前缀 `/home/user/ros2_ws/install_path_conflict_v01/my_dwa_controller` 未覆盖；新版本仅装入独立`install_fallback_acceleration`。不加载新前缀可回到原版本；或禁用原`path_conflict.enabled`并重新configure。撤销源码可先`git apply --reverse --check incremental.patch`，确认后反向应用，保留其他未提交改动。
