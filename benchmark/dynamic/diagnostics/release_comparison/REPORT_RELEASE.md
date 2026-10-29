# MyDWA Release 编译优化性能对照

日期：2026-10-20；六轮实际D01 Headless诊断，未混入V0.1正式基准。

## 结论

Release已成功构建并在三轮运行中实际加载。相同当前源码的空build type与Release相比，实际业务编译命令仅多 `-O3 -DNDEBUG`。整体调用均值从 **77.814 ms降至15.352 ms，合并均值加速5.07×**。每轮波动和配对加速见下表；该变化不能用三轮正常运行波动解释，但本实验不分离-O3与-DNDEBUG的各自贡献。

Release三轮实际调用频率为 **20.000, 20.000, 20.001 Hz**，missed警告 **[0, 0, 0]**，函数超过50ms比例 **0.0%**。这支持在本次三轮短时D01中维持目标20Hz，不构成长时间/所有场景实时性保证。

D01物理接触：未优化 **3/3**，Release **3/3**。这是附带观察，不是构建优化验收条件，也不作为新的正式成功率推断。性能改善不等于动态避障问题已解决。

## 1. 冻结与隔离

工作分支feature/dynamic-obstacle。开始时Git已有 .gitignore、controller.hpp、controller.cpp修改及未跟踪benchmark；原样保留。初始Git差异、源码/配置/旧记录SHA、构建缓存、flags和原插件备份在evidence。没有git add/commit/push。没有修改CMakeLists、业务C++、YAML、D01尺寸/路径/运动触发和评价阈值。

两个独立CMake build/install prefix分别为 `${MYDWA_WS}/{build,install}_compile_comparison_20261020/{unoptimized,release}`，原build/install未覆盖。两者都从同一工作区源码、GCC、BUILD_TESTING=ON构建，所有package内部静态库随相同build type编译，没有重编译ROS/Nav2/DWB系统包。unoptimized是空build type（无-O），不是带-g的Debug。

原安装插件SHA256：`c38de8839ea2f22bfdd88f484316897dd4dc9b97f31b61052687ed1b90031c35`。

- 新未优化插件：`59a57620cf721bd9569937718d9e9074d81b4094f9dccfb2f946bdcc8b49d22b`。
- Release插件：`cceb8c0630836b22916da5e839176497acddabc38cb65028fa3283e5daca85ee`。
- 同一V0.2探针：`91af24625bf2fc773c6c5b9af71329233a5bebcbea586922de581af880c1ebd5`。

每轮从 `/proc/<PID>/maps`保存真实映射，严格拒绝错误路径、双副本或deleted映射；manifest保存运行时SHA且与已归档对应.so一致。NAVPERF_MYDWA_LIB也指向同一选定路径，防止探针偷偷调用原安装。六轮全部通过加载核对，不能只凭插件类名声称Release已加载。

Release编译flags：`-O3 -DNDEBUG -fPIC -Wall -Wextra -Wpedantic`；未优化flags：`-fPIC -Wall -Wextra -Wpedantic`。capture_build核对所有业务编译命令，剔除-O3/-DNDEBUG后逐项一致；完整verbose build日志、compile_commands、缓存和二进制在evidence。没有-fast-math、LTO、-march=native或任何Scorer源级优化。

## 2. 公平性与实际运行

预先固定顺序U1,R1,R2,U2,U3,R3，配对顺序交替；全为D01/mydwa/headless，同一WSL宿主、ROS_DOMAIN_ID91、Gazebo master11436、V0.2原探针、参数文件与原导航/运动/评价程序。每次完整重启仿真、独立初始定位和目标，结束后统一3秒空档。编译及功能测试完成后才测量，无并发构建或其他仿真实例。

六份runtime_parameters与V0.2对应基线全字典一致；原始参数、场景和V0.1/V0.2文件由audit逐SHA复核。sequence_first.json保留真实顺序、UTC、CPU affinity、/proc/stat和loadavg。WSL未提供cpufreq读数，不能保证CPU频率/热状态和Windows后台负载严格相同；这是同一机器连续交替实测，不是假称锁定全部硬件因素。

外部探针保持原版本，每次只mmap记录原函数wall steady_clock及thread CPU，原调用转发一次，复制原有MyDWA分段；无新增控制输入。沿用V0.2约1.26µs/调用合成校准，未假设它是Release实际开销的普遍上界。探针单Controller串行使用，不宣称并发初始化安全。函数计时不等于整个Nav2周期；20Hz对应50ms，missed是原ControllerServer WallRate截止点判断。

**基础设施问题保留**：初版包装器继承V0.2的runner exit0假设；实际原run_trial.py对“完整记录的非成功导航”返回2。因此六轮的包装器/批量退出码非零，但不是导航未执行。没有改写这些原始manifest/sequence/summary，而是统一校验valid、finished、Action accepted、无action_error、参数一致、实际插件映射正确、mmap完整后追加measurement_validation.json及派生metrics。六轮均合格，全数纳入；没有因碰撞丢弃或挑选重跑。独立包装器返回码规则已修复并红绿回归，原V0.2文件未动。

## 3. computeVelocityCommands 整体分布

| 构建 | 调用数 | 均值 ms | 中位 ms | P95 ms | P99 ms | 最大 ms | >50 ms |
| --- | --- | --- | --- | --- | --- | --- | --- |
| unoptimized | 247 | 77.814 | 78.254 | 82.608 | 85.209 | 87.624 | 99.19% |
| release | 415 | 15.352 | 15.408 | 16.061 | 16.806 | 19.095 | 0.00% |

单位ms，百分位为全部调用样本线性插值，超预算严格>50ms。合并调用分布按样本数加权，不冒充3个独立实验的置信区间；下面保留每轮，另列正常返回子集，避免异常快速返回扭曲比较。

| 顺序/运行 | n | 均值 ms | 中位 ms | P95 ms | P99 ms | 最大 ms | >50ms | 调用Hz | missed | score均值ms | 碰撞 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| first_01_unoptimized_001 | 82 | 76.158 | 77.808 | 79.991 | 80.771 | 80.820 | 97.6% | 11.772 | 54 | 74.124 | True |
| first_02_release_001 | 139 | 15.238 | 15.414 | 16.024 | 16.480 | 16.989 | 0.0% | 20.000 | 0 | 14.632 | True |
| first_03_release_002 | 135 | 15.545 | 15.557 | 16.102 | 16.677 | 19.095 | 0.0% | 20.000 | 0 | 14.939 | True |
| first_04_unoptimized_002 | 82 | 78.014 | 78.291 | 80.308 | 81.419 | 83.283 | 100.0% | 11.734 | 55 | 75.965 | True |
| first_05_unoptimized_003 | 83 | 79.253 | 78.682 | 84.857 | 85.741 | 87.624 | 100.0% | 11.595 | 56 | 77.174 | True |
| first_06_release_003 | 141 | 15.280 | 15.259 | 16.174 | 16.909 | 17.114 | 0.0% | 20.001 | 0 | 14.686 | True |

## 4. score、CPU与正常返回子集

| 构建/score | n | 均值 ms | 中位 ms | P95 ms | P99 ms | 最大 ms | >50ms |
| --- | --- | --- | --- | --- | --- | --- | --- |
| unoptimized | 247 | 75.760 | 76.246 | 80.374 | 82.844 | 85.206 | 99.2% |
| release | 415 | 14.750 | 14.810 | 15.424 | 16.165 | 18.416 | 0.0% |

| 构建/正常返回 | n | 均值ms | P95ms | score均值ms | 全调用CPU均值ms | 每轮全调用均值范围ms |
| --- | --- | --- | --- | --- | --- | --- |
| unoptimized | 245 | 78.433 | 82.624 | 76.378 | 77.802 | 76.158–79.253 |
| release | 414 | 15.388 | 16.064 | 14.786 | 15.350 | 15.238–15.545 |

完整threadCPU中位/P95/P99/最大、各原有分段及候选数分布在outputs/comparison.json。异常调用没有删除；正常返回子集另列。速度采样无独立可靠计时，剩余项不写成0。已保存每周期processed/valid/scored计数；闭环接触前后的无可行轨迹调用、候选有效数及路径可随运行不同，因此同时列正常返回对照。未做离线固定输入微基准，不能把所有差异都拆解成单个函数的纯指令执行差异。

| 配对 | 整体均值加速 | 正常返回均值加速 | score均值加速 |
| --- | --- | --- | --- |
| 1 | 4.998× | 5.085× | 5.066× |
| 2 | 5.019× | 5.019× | 5.085× |
| 3 | 5.187× | 5.187× | 5.255× |

合并整体均值加速 **5.069×**。每轮均值范围与配对数值说明构建效应的量级；仅3轮，不做显著性检验或跨场景普遍排名。

## 5. 控制频率与输出频率

| 运行 | 调用Hz/steady | cmd_vel_nav Hz/steady | cmd_vel Hz/steady | 入口周期P95 ms | 异常调用 | 终态 |
| --- | --- | --- | --- | --- | --- | --- |
| first_01_unoptimized_001 | 11.772 | 11.958 | 20.000 | 100.079 | 2 | CANCELED |
| first_02_release_001 | 20.000 | 20.044 | 20.000 | 50.088 | 1 | CANCELED |
| first_03_release_002 | 20.000 | 20.045 | 20.000 | 50.079 | 0 | CANCELED |
| first_04_unoptimized_002 | 11.734 | 11.717 | 20.000 | 100.086 | 0 | CANCELED |
| first_05_unoptimized_003 | 11.595 | 11.608 | 20.000 | 100.118 | 0 | CANCELED |
| first_06_release_003 | 20.001 | 20.006 | 20.000 | 50.078 | 0 | CANCELED |

调用Hz为原函数入口首末steady跨度，输出Hz为独立订阅器原生消息接收steady跨度，限定导航窗口。原始JSON另含sim clock频率；/cmd_vel是平滑后的20Hz底盘输出，不能据此误认原插件也20Hz。入口周期P95描述调度间隔，不能直接当函数耗时。所有六轮accepted目标，真实物理接触后测试器取消Action，终态CANCELED；失败全部保留。

![六轮耗时分布及顺序](outputs/comparison.png)

## 6. 功能、原始记录与回退

两套构建分别运行现有9个功能CTest目标（8个C++ GTest目标及benchmark Python），并执行相关全部52项Python测试，最终日志见evidence/*functional_tests_final.log与python_tests_final.log；仅功能测试，不声称全部lint通过。Release构建不修改源算法，-DNDEBUG是标准Release条件的一部分。

直接CMake安装setup位于share/my_dwa_controller/local_setup.bash，早期脚本错误地查找根local_setup并中断；只修复测试启动路径，不改CMake/源码，原错误日志保留。最终测试在正确selected prefix下重跑通过。原始数据在runs（bin、指令、传感器、contact、GT、summary、maps与日志），逐样本CSV及耗时汇总也保留；outputs/给出逐轮CSV/JSON和图。归档包含二进制/编译参数/原始数据。

默认安装完全不变，退出诊断子进程并在干净终端source原ROS和原workspace setup即可回到原版本；现有默认启动器仍加载原.so。只读audit核验当前源码、旧记录、原安装未变。具体复现命令见README。

## 7. 是否继续优化Scorer

Release已大幅降低原先主要计算开销；判断是否继续优化应看当前score绝对耗时与50ms预算余量，而非只看相对占比。本次Release的score平均14.75ms，仍占整体约96.1%，但只占50ms周期预算的约29.5%；它仍是主要热点，已不构成本次20Hz超时瓶颈，源级Scorer优化不再是恢复20Hz的首要任务。仍可为更大路径/采样规模做后续叶函数采样与长时压力验证，但此次没有改代码来“验证猜想”。D01碰撞机制需要独立逐周期轨迹/评分/实际Costmap证据，不能靠编译提速直接下结论。
