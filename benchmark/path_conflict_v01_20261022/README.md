# Path Conflict V0.1 实施与验证记录（2026-10-22）

功能及参数说明：`../../docs/path_conflict_v01.md`。当前默认关闭。

## 实测结果（每条件一次；两版本分别列出）

|版本|条件|结果|风险状态|
|---|---|---|---|
|首轮 b5076d6471af|D01 2s|碰撞|DwaFallback；未限速/减速|
|首轮 b5076d6471af|D01 4s|碰撞|未触发风险控制|
|首轮 b5076d6471af|D01 6s|SUCCEEDED|未触发风险控制|
|首轮 b5076d6471af|S04|SUCCEEDED，20.9s|未限速/等待；静态无contact真值|
|首轮 b5076d6471af|S05|SUCCEEDED，31.6s|未限速/等待；静态无contact真值|
|Inner前缀修正版 9aecefa4178f|D01 2s|碰撞，接触6.080s|Braking 4.600s，DwaFallback 4.800s|
|Inner前缀修正版 9aecefa4178f|D01 4s|碰撞，接触6.910s|Limited 5.500s，DwaFallback 6.700s|

所有时间相对各次导航dispatch。接触采用原contact消息源stamp，导航dispatch约0.1s量化；
不把跨次时间差当精确因果增益。每次 valid=true，运行时实际库映射已验证，清理返回0。
原场景、原参数不改；只在独立参数副本增加 enabled=true。

首轮发现STOP检查错误地要求完整Outer已知，在3m rolling map中会拒绝实际已知的
Inner停车前缀。按几何条件修正，并新增未知区间/边界回归；随后只重跑2个失败条件。
全部旧结果不覆盖。首轮源码包经逐文件SHA-256匹配manifest后保存；两版本没有混合统计。

修正版2s中路径坐标的冲突前缘从约0.580m变为0.105m，已不满足停车距离条件；
这里只能确认证据变化，尚未归因到感知、路径重投影或物体表面可见性。
4s在Outer预警时目标cap=0.12、实际合并cap=0.135，体现了动态窗口可达下界；
后续仍接触。**这批数据不证明避撞改善。** 两次均未进入WAIT。
修正前静态结果可作为无明显误触发线索，不能宣称最终二进制已完成静态同版本验证。

新模块在这些日志记录中的计算耗时p95约1.35–1.61ms；仅覆盖不同地图/状态转换日志，
不含日志输出与地图复制，不是全循环压力测试或20Hz硬实时保证。
闭环后又修正了折返路径的最早相交安全边界，仅做单元/功能回归，未再跑Gazebo。
因此上表及耗时不代表最终源码的同版本闭环验证；差异仅在monitor.cpp，见verification.json。

![修正版风险与速度响应](//wsl.localhost/Ubuntu-22.04/home/user/ros2_ws/src/my_dwa_controller/benchmark/path_conflict_v01_20261022/risk_response.png)

## 构建、测试与复现

在WSL Ubuntu-22.04执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/user/ros2_ws
colcon --log-base log_path_conflict_v01 build \
  --build-base build_path_conflict_v01 --install-base install_path_conflict_v01 \
  --packages-select my_dwa_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install_path_conflict_v01/setup.bash
cd build_path_conflict_v01/my_dwa_controller
ctest --output-on-failure
```

最终完整CTest：14/17目标通过；10个C++ GTest目标共239项通过，现有Python Benchmark
目标通过。保留原225组命令/异常签名校验；新增静态墙、扩展、rolling/unknown、边界、
重复图、停车条件、角速度可达、恢复失去证据、不同目标重置和插件闭环等测试。
整包pep257/flake8/uncrustify仍失败，详 `tests_final_all.log`；没有将整包标为全绿。
本轮新增模块/单测文件单独uncrustify及两个新Python脚本单独flake8/pep257通过。

实际运行命令（输出目录已存在，复现请换新tag）：

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
source /home/user/ros2_ws/install_path_conflict_v01/my_dwa_controller/share/my_dwa_controller/local_setup.bash
cd /home/user/ros2_ws/src/my_dwa_controller
python3 -B benchmark/path_conflict_v01_20261022/run_smoke.py \
  --cases D01_t2 D01_t4 D01_t6 S04 S05 --tag smoke --domain 96
python3 -B benchmark/path_conflict_v01_20261022/run_smoke.py \
  --cases D01_t2 D01_t4 --tag prefix_fix --domain 96
python3 -B benchmark/path_conflict_v01_20261022/analyze_smoke.py
```

首条5条件命令对应首轮二进制，不能用当前修正版假装重现首轮精确结果。
`sources_smoke_pre_prefix.zip`保存首轮业务源码/构建配置（无旧版测试，复现时BUILD_TESTING=OFF）；
`sources_prefix_fix.zip`保存两次前缀修正版业务源码（BUILD_TESTING=OFF）；
`sources_v01.zip`保存最终几何修正源码、现有及新增测试和配置。每次manifest另保存完整源文件哈希、
Git HEAD、插件哈希、实际命令和场景参数；trial保存原采集器的参数快照、CSV、plan、
Costmap/scan压缩记录、日志及动态contact/真值。命令入口复用原启动/采集/清理代码；
动态场景继续通过原真值隔离预检，没有让Controller订阅真值。

## 交付索引

- `results.json` / `results.csv`：七次全部结果及首个风险时间、耗时统计。
- `runs/*/derived_risk_events.csv`：派生事件表；原始数据仍在各trial目录。
- `run_smoke.py` / `analyze_smoke.py`：薄适配入口与离线归纳，不新增Benchmark框架。
- `build_*.log`、`tests_*red.log`、`tests_final_all.log`：构建、红绿回归和完整检查日志。
- `before.json`、`verification.json`：原始文件保护核验；原YAML、Benchmark场景和
  既有实验数据未修改。原有Controller未提交诊断改动已保留；未commit/push。

目前限制：严格的全Outer清晰恢复可能在rolling地图中长期不满足；地图不变也不累计恢复；
Nav2原10s进度检查器可能结束等待；停车参数未经实体标定；不能保证动态物体未来不进入停车位置。
关闭开关并重新configure可回到原DWA策略；独立安装前缀可直接不加载。
