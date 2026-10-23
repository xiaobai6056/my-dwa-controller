# PathHandler 实施计划

**目标：** 无状态地提供与当前机器人位姿处于同一局部地图坐标系的参考路径。

**架构：** 单次 TF 快照及逆变换、既有路径几何算法、现有 Pose2D，C++17/ROS 2 Humble/GTest。

- [x] 在 test/test_path_geometry.cpp 将跨角度边界预期改为最短差插值，补充逆向与截取朝向用例；运行 test_path_geometry 验证旧实现失败。
- [x] 在 src/path_geometry.cpp 仅修改 theta 插值，更新 path_geometry.hpp 契约，保持位置及其他算法行为。
- [x] 新建 test/test_path_handler.cpp，用共享 Buffer 和 setTransform 测试设计文档中的坐标、时间及非法输入约定。编译确认缺失 path_handler.hpp 时失败。
- [x] 新建 include/my_dwa_controller/path_handler.hpp 和 src/path_handler.cpp：验证帧、输入消息和长度；跨帧按机器人时间查询一次 TF，转换机器人、调用 projectOntoPath/extractForwardPath、用逆变换输出 Pose2D。
- [x] 更新 CMakeLists.txt 的 find_package、PIC 静态库、安装/导出及 ament_add_gtest；package.xml 补充直接依赖。Controller 不改。
- [x] `source /opt/ros/humble/setup.bash; cd ~/ros2_ws; colcon build --packages-select my_dwa_controller --cmake-args -DBUILD_TESTING=ON` 验证构建。
- [x] `ctest --test-dir build/my_dwa_controller -L gtest --output-on-failure` 验证全部算法测试；`colcon test --packages-select my_dwa_controller` 和 test-result 检查包内静态检查，单独报告既有 Controller 格式问题。
- [x] 根据代码审查技能进行只读复核，并记录实际测试结果和后续 Controller 示例。

直接在用户指定包中执行；该包未纳入 Git，不创建工作树或提交。
