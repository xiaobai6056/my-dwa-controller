# Controller 第一阶段整合实施计划

**目标：** 在原 Controller 内串联现有算法并通过可执行基础测试，不启动 Gazebo。

- [x] 新建 test/test_controller.cpp，人工 Costmap2DROS 子类只装入 LayeredCostmap，不启动节点生命周期或传感器；GTest 调用 configure/setPlan/activate/compute，验证最低代价、过滤和失败语义。
- [x] 在 CMake 注册测试并链接现有 Controller，先运行测试确认原实现固定零速度及不报错的缺陷。
- [x] 在 controller.hpp/.cpp 添加配置和模块成员、状态锁；configure 用现有参数及必要新增参数初始化，cleanup 释放，生命周期及限速同步。
- [x] 实现局部位姿/参考路径、采样、预测、持图锁过滤评分、最低分选择、TwistStamped 输出，不修改算法模块。
- [x] 更新 YAML FollowPath 的实际参数及 CMake 链接库和必要直接依赖。
- [x] `source /opt/ros/humble/setup.bash; cd ~/ros2_ws; colcon build --packages-select my_dwa_controller` 编译。
- [x] `ctest --test-dir build/my_dwa_controller -L gtest --output-on-failure` 回归所有算法和 Controller 测试；启用系统 cppcheck 2.7 后运行全包检查。
- [x] 按代码审查技能做只读复核，修正有依据的问题；更新文档中的运行命令、参数表和未验证事项。

在用户指定的非 Git 包中直接修改，不创建工作树或提交。
