cmake_minimum_required(VERSION 3.8)
project(my_dwa_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
ament_export_dependencies(Eigen3)


# 轨迹预测独立库；启用 PIC，便于后续链接到 Controller 共享插件。
add_library(trajectory_predictor STATIC src/trajectory_predictor.cpp)
set_target_properties(trajectory_predictor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(trajectory_predictor PUBLIC cxx_std_14)
target_include_directories(trajectory_predictor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(trajectory_predictor geometry_msgs)

install(TARGETS trajectory_predictor
  EXPORT export_trajectory_predictor
  ARCHIVE DESTINATION lib
)
ament_export_targets(export_trajectory_predictor HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs)

add_library(motion_predictor STATIC src/motion_predictor.cpp)
set_target_properties(motion_predictor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(motion_predictor PUBLIC cxx_std_17)
target_include_directories(motion_predictor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(motion_predictor geometry_msgs)
install(TARGETS motion_predictor EXPORT export_motion_predictor ARCHIVE DESTINATION lib)
ament_export_targets(export_motion_predictor HAS_LIBRARY_TARGET)

# 无状态速度采样独立库，可单独使用，也可供后续共享插件调用。
add_library(velocity_sampler STATIC src/velocity_sampler.cpp)
set_target_properties(velocity_sampler PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(velocity_sampler PUBLIC cxx_std_14)
target_include_directories(velocity_sampler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(velocity_sampler geometry_msgs)

install(TARGETS velocity_sampler
  EXPORT export_velocity_sampler
  ARCHIVE DESTINATION lib
)
ament_export_targets(export_velocity_sampler HAS_LIBRARY_TARGET)

# 障碍物评估独立库；启用 PIC，便于后续链接到共享插件。
add_library(obstacle_evaluator STATIC src/obstacle_evaluator.cpp)
set_target_properties(obstacle_evaluator PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(obstacle_evaluator PUBLIC cxx_std_17)
target_include_directories(obstacle_evaluator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(obstacle_evaluator geometry_msgs nav2_costmap_2d)

install(TARGETS obstacle_evaluator
  EXPORT export_obstacle_evaluator
  ARCHIVE DESTINATION lib
)
ament_export_targets(export_obstacle_evaluator HAS_LIBRARY_TARGET)
ament_export_dependencies(nav2_costmap_2d)

# 路径几何与公共位姿转换；转换实现使用已有 TF 四元数工具。
add_library(path_geometry STATIC src/path_geometry.cpp)
set_target_properties(path_geometry PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(path_geometry PUBLIC cxx_std_17)
target_include_directories(path_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(path_geometry
  PUBLIC geometry_msgs::geometry_msgs__rosidl_generator_cpp
  PRIVATE tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs
)

install(TARGETS path_geometry
  EXPORT export_path_geometry
  ARCHIVE DESTINATION lib
)
ament_export_targets(export_path_geometry HAS_LIBRARY_TARGET)

# 轨迹评分独立库；复用路径几何工具，不新增地图或节点依赖。
add_library(trajectory_scorer STATIC src/trajectory_scorer.cpp)
set_target_properties(trajectory_scorer PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(trajectory_scorer PUBLIC cxx_std_17)
target_include_directories(trajectory_scorer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(trajectory_scorer PUBLIC path_geometry)

install(TARGETS trajectory_scorer
  EXPORT export_trajectory_scorer
  ARCHIVE DESTINATION lib
)
ament_export_targets(export_trajectory_scorer HAS_LIBRARY_TARGET)

# 路径消息与 TF 适配层；全局路径由 Controller 持有，几何计算复用独立库。
add_library(path_handler STATIC src/path_handler.cpp)
set_target_properties(path_handler PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(path_handler PUBLIC cxx_std_17)
target_include_directories(path_handler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(path_handler PUBLIC
  path_geometry
  nav_msgs::nav_msgs__rosidl_generator_cpp
  tf2_ros::tf2_ros
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  rclcpp::rclcpp
)

install(TARGETS path_handler
  EXPORT export_path_handler
  ARCHIVE DESTINATION lib
)
ament_export_targets(export_path_handler HAS_LIBRARY_TARGET)
ament_export_dependencies(nav_msgs tf2 tf2_ros tf2_geometry_msgs rclcpp)

# Scan perception is independent of path scoring and velocity selection.
add_library(scan_obstacle_tracker STATIC src/scan_obstacle_tracker.cpp src/surface_motion.cpp src/partial_surface/velocity.cpp)
target_include_directories(scan_obstacle_tracker PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/partial_surface)
target_link_libraries(scan_obstacle_tracker Eigen3::Eigen)
set_target_properties(scan_obstacle_tracker PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(scan_obstacle_tracker PUBLIC cxx_std_17)
target_include_directories(scan_obstacle_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(scan_obstacle_tracker sensor_msgs rclcpp tf2_ros tf2 tf2_geometry_msgs)
install(TARGETS scan_obstacle_tracker
  EXPORT export_scan_obstacle_tracker ARCHIVE DESTINATION lib)
ament_export_targets(export_scan_obstacle_tracker HAS_LIBRARY_TARGET)
ament_export_dependencies(sensor_msgs)

# Source-time quasi-static evidence and causal raw-odom stop observation.
add_library(quasi_static_observer STATIC src/quasi_static_observer.cpp)
set_target_properties(quasi_static_observer PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(quasi_static_observer PUBLIC cxx_std_17)
target_include_directories(quasi_static_observer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS quasi_static_observer
  EXPORT export_quasi_static_observer ARCHIVE DESTINATION lib)
ament_export_targets(export_quasi_static_observer HAS_LIBRARY_TARGET)

# Stateless path conflict query, reused for trusted-motion handover/re-entry checks.
add_library(path_conflict_evaluator STATIC src/path_conflict_evaluator.cpp)
set_target_properties(path_conflict_evaluator PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(path_conflict_evaluator PUBLIC cxx_std_17)
target_include_directories(path_conflict_evaluator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(path_conflict_evaluator PUBLIC path_geometry scan_obstacle_tracker)
install(TARGETS path_conflict_evaluator
  EXPORT export_path_conflict_evaluator ARCHIVE DESTINATION lib)
ament_export_targets(export_path_conflict_evaluator HAS_LIBRARY_TARGET)

# Continuous candidate geometry, used by Controller only when Scan tracking is enabled.
add_library(dynamic_trajectory_evaluator STATIC src/dynamic_trajectory_evaluator.cpp src/dynamic_risk_evaluator.cpp src/dynamic_filter.cpp)
set_target_properties(dynamic_trajectory_evaluator PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(dynamic_trajectory_evaluator PUBLIC cxx_std_17)
target_include_directories(dynamic_trajectory_evaluator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dynamic_trajectory_evaluator PUBLIC
  scan_obstacle_tracker geometry_msgs::geometry_msgs__rosidl_generator_cpp)
install(TARGETS dynamic_trajectory_evaluator
  EXPORT export_dynamic_trajectory_evaluator ARCHIVE DESTINATION lib)
ament_export_targets(export_dynamic_trajectory_evaluator HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  # Benchmark data/code is archived separately and need not be present.
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark")
    ament_add_test(test_benchmark
      COMMAND ${Python3_EXECUTABLE} -m unittest discover
        -s ${CMAKE_CURRENT_SOURCE_DIR}/benchmark -p "test_*.py" -v
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 20
    )
  endif()
  ament_add_gtest(test_trajectory_predictor test/test_trajectory_predictor.cpp)
  target_link_libraries(test_trajectory_predictor trajectory_predictor)

  ament_add_gtest(test_velocity_sampler test/test_velocity_sampler.cpp)
  target_link_libraries(test_velocity_sampler velocity_sampler)

  ament_add_gtest(test_obstacle_evaluator test/test_obstacle_evaluator.cpp)
  target_link_libraries(test_obstacle_evaluator obstacle_evaluator)

  ament_add_gtest(test_path_geometry test/test_path_geometry.cpp)
  target_link_libraries(test_path_geometry path_geometry)

  ament_add_gtest(test_trajectory_scorer test/test_trajectory_scorer.cpp)
  target_link_libraries(test_trajectory_scorer trajectory_scorer)

  ament_add_gtest(test_path_handler test/test_path_handler.cpp)
  target_link_libraries(test_path_handler path_handler)

  ament_add_gtest(test_scan_obstacle_tracker test/test_scan_obstacle_tracker.cpp)
  target_link_libraries(test_scan_obstacle_tracker scan_obstacle_tracker)

  ament_add_gtest(test_quasi_static_observer test/test_quasi_static_observer.cpp)
  target_link_libraries(test_quasi_static_observer quasi_static_observer)

  ament_add_gtest(test_path_conflict_evaluator test/test_path_conflict_evaluator.cpp)
  target_link_libraries(test_path_conflict_evaluator path_conflict_evaluator)

  ament_add_gtest(test_dynamic_trajectory_evaluator test/test_dynamic_trajectory_evaluator.cpp)
  target_link_libraries(test_dynamic_trajectory_evaluator dynamic_trajectory_evaluator trajectory_predictor)

  # 使用 cppcheck 的 GTest 宏模型注册全包静态检查。
  # 仅替换自动注册，检查文件范围不变，避免重复执行同一项检查。
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(LIBRARIES googletest INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)

  # Trial snapshots contain copies of upstream Python; lint only maintained code.
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257(--exclude benchmark/results benchmark/evidence)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_pep257)

  find_package(ament_lint_auto REQUIRED)
  # 以下设置跳过源文件版权检查。
  # 为所有源文件补充版权与许可证后，可注释掉该设置以启用检查。
  set(ament_cmake_copyright_FOUND TRUE)
  # 以下设置跳过 cpplint 检查，该工具仅适用于 Git 仓库。
  # 当本包置于 Git 仓库中，并且
  # 所有源文件均已补齐版权与许可证时，可注释掉该设置。
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(${PROJECT_NAME} SHARED
  src/controller.cpp
  src/crossing_yield.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)
target_link_libraries(${PROJECT_NAME}
  velocity_sampler trajectory_predictor motion_predictor obstacle_evaluator trajectory_scorer path_handler
  scan_obstacle_tracker dynamic_trajectory_evaluator
  quasi_static_observer path_conflict_evaluator
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  ament_add_gtest(test_controller test/test_controller.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
  )
  target_compile_features(test_controller PRIVATE cxx_std_17)
  target_compile_definitions(test_controller PRIVATE
    MY_DWA_TEST_PARAMS_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/nav2_params.yaml"
  )
  target_link_libraries(test_controller
    velocity_sampler trajectory_predictor
    obstacle_evaluator trajectory_scorer path_handler
  )
  ament_target_dependencies(test_controller nav2_core pluginlib rclcpp_lifecycle)
  add_dependencies(test_controller ${PROJECT_NAME})
  target_link_libraries(test_controller motion_predictor)

  ament_add_gtest(test_surface_motion test/test_surface_motion.cpp)
  target_link_libraries(test_surface_motion scan_obstacle_tracker)
  target_compile_definitions(test_surface_motion PRIVATE
    SURFACE_MOTION_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/surface_motion")

  ament_add_gtest(test_stoppable test/test_stoppable.cpp)
  target_link_libraries(test_stoppable motion_predictor obstacle_evaluator dynamic_trajectory_evaluator)

  ament_add_gtest(test_dynamic_risk test/test_dynamic_risk.cpp)
  target_link_libraries(test_dynamic_risk dynamic_trajectory_evaluator trajectory_predictor)

  ament_add_gtest(test_crossing_yield test/test_crossing_yield.cpp src/crossing_yield.cpp)
  target_link_libraries(test_crossing_yield scan_obstacle_tracker motion_predictor)

  ament_add_gtest(test_controller_scan test/test_controller_scan.cpp)
  target_link_libraries(test_controller_scan ${PROJECT_NAME} dynamic_trajectory_evaluator)
  ament_target_dependencies(test_controller_scan rclcpp rclcpp_lifecycle nav2_costmap_2d sensor_msgs)

endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  nav2_core
  my_dwa_controller_plugin.xml
)

ament_package()
