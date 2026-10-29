cmake_minimum_required(VERSION 3.16)
project(bypass_probe LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(my_dwa_controller REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(GTest REQUIRED)
find_library(CONTROLLER_LIBRARY NAMES my_dwa_controller
  PATHS "${my_dwa_controller_DIR}/../../../lib" REQUIRED NO_DEFAULT_PATH)
add_executable(probe probe.cpp)
target_compile_features(probe PUBLIC cxx_std_17)
target_link_libraries(probe ${CONTROLLER_LIBRARY}
  my_dwa_controller::scan_obstacle_tracker my_dwa_controller::motion_predictor
  my_dwa_controller::velocity_sampler my_dwa_controller::path_handler
  my_dwa_controller::dynamic_trajectory_evaluator nlohmann_json::nlohmann_json)
ament_target_dependencies(probe rosbag2_cpp tf2_msgs)
add_executable(geometry_test geometry_test.cpp)
target_compile_features(geometry_test PUBLIC cxx_std_17)
target_link_libraries(geometry_test my_dwa_controller::scan_obstacle_tracker GTest::gtest_main)
enable_testing()
add_test(NAME geometry COMMAND geometry_test)
