# MyDWA 未优化 / Release 构建对照

只验证构建优化对既有Controller性能的影响；没有修改业务C++、CMakeLists.txt、Nav2 YAML、D01资产/运动脚本，不混入V0.1成功率。

## 隔离、构建与回退

原安装 `${MYDWA_WS}/install/my_dwa_controller` 及原build保留。用户未提交的controller.cpp/controller.hpp修改原样保留；evidence/before.json、git_before.diff、original_CMakeCache.txt、original_flags.make和原.so保存开始状态。

两个新build/install在工作区源目录之外：

- `${MYDWA_WS}/build_compile_comparison_20261020/{unoptimized,release}`
- `${MYDWA_WS}/install_compile_comparison_20261020/{unoptimized,release}`

都是同一当前源码、编译器、BUILD_TESTING=ON；仅empty build type与Release不同。实际business compile_commands去掉 `-O3 -DNDEBUG` 后逐项完全一致（evidence/build_identity.json）。没有修改全局默认构建类型，也没有重建任何系统ROS/Nav2包。这里的unoptimized不是Debug，没有添加-g。

从干净终端首次构建：

```bash
bash ${REPO_ROOT}/benchmark/dynamic/diagnostics/release_comparison/build.sh
```

已有独立目录时脚本拒绝覆盖，不必重建本次已完成的构建。可只传 `release` 或 `unoptimized` 来构建尚不存在的单个条件。直接CMake安装没有colcon根local_setup，实际使用 `<prefix>/share/my_dwa_controller/local_setup.bash`。早期路径错误与测试红灯日志保留，最终记录见 *_functional_tests_final.log。

无需替换原.so即可回退：退出诊断子进程后，原启动入口仍source原install；干净WSL终端source `/opt/ros/humble/setup.bash` 和 `${MYDWA_WS}/install/setup.bash`，默认仍是原安装。Release只在本模块选定的子进程环境内生效，不改用户shell配置。所有实例已顺序结束并清理。

## 六轮实际对照与重现

固定D01、mydwa、Headless、domain91/Gazebo master11436、原参数与超时，原V0.2外部ELF探针原样复用，完整仿真重启。运行顺序 U1,R1,R2,U2,U3,R3；每轮结束统一3秒空档。编译/测试结束后才开始测量，期间没有其他构建或仿真实例。

```bash
cd ${REPO_ROOT}
source /opt/ros/humble/setup.bash
source ${MYDWA_WS}/install/setup.bash
python3 benchmark/dynamic/diagnostics/release_comparison/run_all.py --tag new_comparison
```

tag不可复用，所有输出在本模块runs内。单次命令：

```bash
python3 benchmark/dynamic/diagnostics/release_comparison/run_one.py --variant release --scenario D01 --controller mydwa --headless --run-id release_new --preload ${REPO_ROOT}/benchmark/dynamic/diagnostics/perf_build/libnavperf.so
```

将release换unoptimized即可切换。新launch_scene.sh只在原冻结V0.2启动器source原overlay之后追加所选包local_setup；其余场景、运动、导航发送和评价复用原代码。NAVPERF_MYDWA_LIB指定同一所选插件，/proc/<PID>/maps严格检查只映射该路径；每轮manifest保存实际路径/hash与映射证据，不能仅凭参数或日志中的类名判断Release已加载。

原run_trial.py的正常非成功导航退出码是2，不能将碰撞误作测试系统未执行。本次初版复制包装器继承了V0.2的严格exit0假设，六轮原始manifest/退出码保留。离线后处理按valid、finished、Action accepted、无action_error、实际映射、同参数、mmap样本完整性统一核验并追加派生metrics；不重写raw manifest/summary，也不删除或重跑挑选表现好的轮次。修正后的独立包装器有相应回归测试；原V0.2脚本与数据均未改。

## 离线结果与功能验证

```bash
cd ${REPO_ROOT}
source /opt/ros/humble/setup.bash
source ${MYDWA_WS}/install/setup.bash
python3 benchmark/dynamic/diagnostics/release_comparison/analyze_comparison.py
python3 benchmark/dynamic/diagnostics/release_comparison/audit.py
python3 -m unittest discover -s benchmark -p 'test_*.py' -v
```

重复执行分析只重建派生JSON/CSV/图表/报告，不覆盖原始summary、manifest、bin、指令、传感器日志。首次六轮原始数据在runs，派生对比在outputs；全部原始记录及构建二进制归档另附。比较表有每轮与合并调用分布，同时列正常返回/异常调用，score阶段、threadCPU、实际调用/输出Hz、missed警告和接触结果。百分位为线性插值，超预算条件严格>50ms；合并调用并不等价3个独立实验重复的统计推断，报告另列每轮均值范围与配对加速倍数。

两套安装分别验证C++与benchmark测试：

```bash
source /opt/ros/humble/setup.bash
source ${MYDWA_WS}/install_compile_comparison_20261020/release/share/my_dwa_controller/local_setup.bash
ctest --test-dir ${MYDWA_WS}/build_compile_comparison_20261020/release -R '^test_' --output-on-failure
```

另一条件将release换unoptimized。只运行现有功能目标，不声称全部lint通过。核心算法与参数哈希、V0.1/V0.2旧记录哈希由audit核对，未git add/commit/push。

## 解释边界

Release同时包含-O3和-DNDEBUG，实验评估整个Release构建条件，不分离两者贡献；没有-fast-math、LTO、-march=native或Scorer源代码优化。候选数/路径长度与接触状态可能随闭环进程微小变化，报告记录工作量与正常返回子集。当前WSL/宿主负载、CPU频率和热状态未做严格实验室控制（WSL未提供cpufreq读数），交替顺序及每轮环境记录只能降低/揭示漂移，不能完全排除。每种3轮只做描述比较。碰撞是附带观察，性能达标不意味着动态避障正确。