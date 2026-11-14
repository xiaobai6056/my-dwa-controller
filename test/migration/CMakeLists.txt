cmake_minimum_required(VERSION 3.16)
project(migration_replay LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(my_dwa_controller REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_library(CONTROLLER_LIBRARY NAMES my_dwa_controller
  PATHS "${my_dwa_controller_DIR}/../../../lib" REQUIRED NO_DEFAULT_PATH)
add_executable(replay_history replay_history.cpp)
target_compile_features(replay_history PUBLIC cxx_std_17)
target_link_libraries(replay_history ${CONTROLLER_LIBRARY}
  my_dwa_controller::scan_obstacle_tracker my_dwa_controller::motion_predictor
  nlohmann_json::nlohmann_json)
ament_target_dependencies(replay_history rosbag2_cpp tf2_msgs)
