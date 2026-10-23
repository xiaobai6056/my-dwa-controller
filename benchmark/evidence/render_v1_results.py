"""Render this recorded V1 acceptance report from the complete aggregate."""
import json
from pathlib import Path

bench = Path(__file__).resolve().parents[1]
suite = bench / 'results/v1/static_v1_20261019'
aggregate = json.loads((suite / 'aggregate.json').read_text())
validation = json.loads((bench / 'evidence/v1_validation.json').read_text())
groups = sorted(aggregate['groups'], key=lambda g: (g['scene'], g['controller']))


def value(group, key):
    stat = group['all_observed'][key]
    if stat['mean'] is None:
        return 'null (n=0)'
    sd = f'{stat["sample_sd"]:.3f}' if stat['sample_sd'] is not None else 'NA'
    return f'{stat["mean"]:.3f} ± {sd}'


lines = ['# 静态导航 Benchmark V1：2026-10-19 真实验收', '',
         '36 次独立重启的 Gazebo Classic + Nav2 Headless 端到端 NavigateToPose 试验。'
         '每场景、每 Controller 3 次；35 次 SUCCEEDED，MyDWA/S04/1 墙钟超时并确认取消。'
         '失败之后继续执行全部任务，没有删除失败样本。', '',
         '配置、坐标和指标定义见 [V1_README.md](V1_README.md)，完整统计为 '
         '[aggregate.json](results/v1/static_v1_20261019/aggregate.json)。'
         '以下全部使用 all_observed（含失败的局部记录），均值 ± 样本标准差；'
         '表内每项 n=3。另存 successful_valid 但不以它隐藏失败。', '',
         '## 导航、轨迹和终点', '',
         '时间为首次规划 Goal 原生时间至观测结束的仿真秒，包含失败取消收尾，'
         '并非精确 Action 起止时间。原始时钟跨度、末次反馈时长独立保留。'
         '路径误差参考各次首次有效全局路径；端到端路径可能因定位/重规划而不同。', '',
         '|场景|Controller|成功|仿真 s|轨迹 m|横向 RMSE m|最大横向 m|终点 m|朝向 rad|',
         '|---|---|---:|---|---|---|---|---|---|']
for group in groups:
    keys = ['timing.first_planner_to_observed_stop_sim_s', 'motion.trajectory_length_m',
            'motion.lateral_rmse_m', 'motion.lateral_max_m', 'final_error.position_m',
            'final_error.yaw_rad']
    lines.append(f'|{group["scene"]}|{group["controller"]}|{group["action_successes"]}/3|'
                 + '|'.join(value(group, key) for key in keys) + '|')
lines += ['', '## 实测运动变化', '',
          '有符号车体 vx / wz 使用 odom 原生仿真时间，缺失段不跨越。'
          '角速度切换采用 0.05 rad/s 死区与 0.15 s 符号持续要求。'
          '指令速度与实测速度的完整统计均在每次 metrics.json 内独立保存。', '',
          '|场景|Controller|vx 标准差 m/s|wz 标准差 rad/s|平均绝对线加速度 m/s²|'
          '平均绝对角加速度 rad/s²|角速度方向切换|恢复 Goal 数|',
          '|---|---|---|---|---|---|---|---|']
for group in groups:
    keys = ['velocity.actual.linear.std', 'velocity.actual.angular.std',
            'velocity.actual.linear.mean_abs_acceleration',
            'velocity.actual.angular.mean_abs_acceleration',
            'velocity.actual.angular_direction_switches',
            'navigation_system.behavior_goals_executed']
    lines.append(f'|{group["scene"]}|{group["controller"]}|'
                 + '|'.join(value(group, key) for key in keys) + '|')
lines += ['', '## 验收与限制', '',
          f'- {validation["total_csv_rows"]:,} 行 CSV；36/36 起点、实际插件、时钟顺序、'
          '原始输入/配置哈希、清理返回码检查通过。6/6 场景两种 Controller 共用参数及资源签名一致。',
          '- S01/S06 实际首路径为长直线，S02 经过 L 形拐角，S03 经过强制高位走廊后返回，'
          'S04 绕开直连线上圆柱，S05 穿过 0.8 m 通道；36 次首路径均通过几何目的检查。',
          '- S06 六次均 SUCCEEDED，45–49 个近终点低线速旋转样本，恢复动作审计就绪，'
          '没有 Spin 或其他恢复 Goal；MyDWA 日志同时有 goal_phase=rotating。',
          f'- 最大终点 map/world 偏差 {validation["end_map_world_disagreement_max_m"]:.3f} m，'
          '无超过 0.5 m 的定位警示。这是异步端点诊断，不代表全程定位精度或障碍净间距。',
          '- MyDWA/S04/trial_001：墙钟 120 s 触发取消，Action CANCELED，最终位置误差 '
          '0.525 m、朝向误差 0.237 rad，反馈恢复次数最大 9。日志见 '
          'results/v1/static_v1_20261019/logs/mydwa_S04_1_launch.log。'
          '观察到近终点阶段零速度与恢复循环；需结合局部目标、GoalChecker、定位和 BT '
          '进一步定位，当前证据不把整次故障简单归为控制器算法缺陷。',
          '- 最小障碍净间距、碰撞次数为 null：没有可靠完整碰撞 footprint/contact 依据，'
          '不以激光量程、膨胀参数或中心点间距替代。固定参考 followPath（B）尚未执行。',
          '- 仅 3 次重复，数值差异为描述性结果，不宣称统计显著优势。'
          '终点误差接近配置的 0.25 m GoalChecker 容差，属于保留原配置的行为。',
          '- 构建通过；34 项 Benchmark Python、8 组共 200 项 C++ 功能测试、6 项清理测试通过；'
          'flake8/pep257/cppcheck 等通过。全包仍有历史 obstacle_evaluator.cpp uncrustify 失败。'
          'UI 入口、8 个 C++ 源文件和两份 Controller YAML 的原哈希均一致；本轮未手工重测 RViz。',
          '- 批量结束后对旧清理器新增子进程竞态作辅助修复：专属组空时再扫一次确认。'
          '确定性回归及真实临时进程树测试通过，额外 S01 smoke 单独保存，绝不混入上述 36 次。',
          '- 额外 smoke 全部保留：cleanup_smoke_20261019 的初始位姿请求已到达 AMCL，'
          '但 CLI 回复丢失，15 s 超时导致 launcher 关闭仿真，Action FAILED；日志中后续'
          '心跳错误发生在关闭阶段。start_scene.sh 已改为记录回复警告并保留仿真，'
          '仍由 runner 的 TF/map/world 起点预检决定能否发 Goal；不重发重置请求。'
          '修复后的 cleanup_smoke_20261019_002 为 SUCCEEDED，清理返回 0。'
          '该次服务正常回复，尚未人为注入丢回复来重测警告分支。'
          '该额外故障归于启动服务应答处理，不混作 Controller 性能样本。',
          '', '## 文件范围', '',
          '- 新增 benchmark/scenarios.yaml、scenes.py、start_scene.sh 和 '
          'assets/S02,S03,S05/ 的 SDF/PGM/YAML/geometry.json，统一定义场景与物理资源。',
          '- 新增 benchmark/metrics.py、run_suite.py、test_metrics.py、test_scenes.py、'
          'test_suite.py、V1_PLAN.md、V1_README.md、V1_RESULTS.md，以及 evidence/ 验证脚本/日志。',
          '- 扩展 benchmark/run_trial.py、test_run_trial.py；更新 benchmark/README.md。'
          'CMakeLists.txt 注册 Python 测试，package.xml 声明已有 PyYAML 运行依赖。',
          '- 辅助修复工作区 scripts/_baseline_processes.py；扩展 '
          'scripts/tests/test_baseline_cleanup.py。原有 UI/headless 入口脚本未改。',
          '- 原始和派生输出：benchmark/results/v1/static_v1_20261019/；'
          'pilot 和两次额外 smoke 各自独立保留。',
          '', '## 实际命令与下一步', '',
          '启动、单次执行、批量选择、重算和结果查看命令完整列于 V1_README.md。'
          '原始正式命令：', '', '```bash',
          'source /opt/ros/humble/setup.bash',
          'source /home/user/ros2_ws/install/setup.bash',
          'cd /home/user/ros2_ws/src/my_dwa_controller/benchmark',
          'python3 run_suite.py --controllers mydwa dwb --scenes S01 S02 S03 S04 S05 S06 '
          '--repeats 3 --run-id static_v1_20261019',
          'python3 run_suite.py --aggregate results/v1/static_v1_20261019',
          'cat results/v1/static_v1_20261019/report.md', '```', '',
          '已有 run-id 拒绝覆盖；复现时省略它产生新 ID。最小扩展：继续在 scenarios.yaml '
          '增加经过地图几何核验的场景；重用重启调度器；先增加一个保存共同参考路径并以 '
          'followPath 执行的对照；安全指标须先建立真实 footprint/contact 数据依据。'
          '现有原始数据可直接增加离线指标，无须重跑已有仿真。', '']
(bench / 'V1_RESULTS.md').write_text('\n'.join(lines))
print(bench / 'V1_RESULTS.md')
