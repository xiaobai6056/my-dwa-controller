# TrajectoryScorer 实施计划

> 在当前会话内依次执行，使用 executing-plans 的逐项验证流程。

**目标：** 实现设计文档中限定的四项线性评分与可长期保留的独立测试。

**架构：** 评分器只保存已验证的配置，复用 Trajectory、Pose2D 和 distanceToPath；通过标准异常报告输入错误和计算溢出。

**技术：** C++17、ament_cmake、GTest、ROS 2 Humble 已有 geometry_msgs 类型。

## 步骤

- [x] 新建 test/test_trajectory_scorer.cpp，使用构造 API `TrajectoryScorer(1.0)`，验证轨迹 {(1,1),(3,1)}、速度 0.5、参考线段 {(0,0),(4,0)}、均值 126 的结果为 obstacle=0.5、speed=0.5、goal=sqrt(2)、path=2、total=0.7+0.2*sqrt(2)。补充设计文档中的几何、配置、边界和异常测试。
- [x] 用 g++ 编译该测试，确认因 trajectory_scorer.hpp 尚不存在而失败，记录输出。
- [x] 新建 include/my_dwa_controller/trajectory_scorer.hpp 和 src/trajectory_scorer.cpp，公开默认权重、分项结果和构造/评分签名；实现参数校验、均值复用、前进速度范围校验、路径平均距离、终点距离和结果有限性检查。
- [x] 修改 CMakeLists.txt，按 path_geometry 的 PIC/安装/导出规范注册 trajectory_scorer，PUBLIC 链接 path_geometry；在 BUILD_TESTING 中注册 test_trajectory_scorer 并链接库。
- [x] 在 WSL 执行 `source /opt/ros/humble/setup.bash; cd ~/ros2_ws; colcon build --packages-select my_dwa_controller --cmake-args -DBUILD_TESTING=ON`，要求构建退出码为 0。
- [x] 执行 `colcon test --packages-select my_dwa_controller --event-handlers console_direct+` 和 `colcon test-result --test-result-base build/my_dwa_controller --verbose`，检查新增测试和原有算法测试，单独报告既有格式检查问题。
- [x] 审阅文件范围、异常契约、导出目标及中文注释；记录实际测试结果，提供后续 Controller 调用示例。

目标包未纳入 Git 仓库；直接修改用户指定位置，不创建提交或工作树。
