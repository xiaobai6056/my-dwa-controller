cmake_minimum_required(VERSION 3.16)
project(rolling_bypass LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(my_dwa_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(GTest REQUIRED)
add_library(rolling_model rolling_model.cpp)
target_compile_features(rolling_model PUBLIC cxx_std_17)
target_link_libraries(rolling_model my_dwa_controller::scan_obstacle_tracker)
ament_target_dependencies(rolling_model nav2_costmap_2d)
add_executable(model_test model_test.cpp)
target_link_libraries(model_test rolling_model GTest::gtest_main)
enable_testing()
add_test(NAME rolling_model COMMAND model_test)
add_executable(rolling rolling.cpp)
target_link_libraries(rolling rolling_model my_dwa_controller::motion_predictor
  my_dwa_controller::velocity_sampler my_dwa_controller::trajectory_scorer
  my_dwa_controller::obstacle_evaluator my_dwa_controller::dynamic_trajectory_evaluator
  nlohmann_json::nlohmann_json)
add_executable(audit_selected audit_selected.cpp)
target_link_libraries(audit_selected rolling_model my_dwa_controller::motion_predictor nlohmann_json::nlohmann_json)

add_executable(test_strategy test_strategy.cpp)
target_link_libraries(test_strategy my_dwa_controller::trajectory_scorer GTest::gtest_main)
add_test(NAME strategy_weights COMMAND test_strategy)

find_library(CONTROLLER_LIBRARY NAMES my_dwa_controller
  PATHS "${my_dwa_controller_DIR}/../../../lib" REQUIRED NO_DEFAULT_PATH)
add_library(reference_model reference.cpp)
target_link_libraries(reference_model rolling_model ${CONTROLLER_LIBRARY} my_dwa_controller::trajectory_scorer)
add_executable(test_reference test_reference.cpp)
target_link_libraries(test_reference reference_model GTest::gtest_main)
add_test(NAME reference_lifecycle COMMAND test_reference)

target_link_libraries(rolling reference_model)

add_executable(audit_reference audit_reference.cpp)
target_link_libraries(audit_reference reference_model my_dwa_controller::motion_predictor my_dwa_controller::velocity_sampler my_dwa_controller::trajectory_scorer my_dwa_controller::obstacle_evaluator my_dwa_controller::dynamic_trajectory_evaluator nlohmann_json::nlohmann_json)
