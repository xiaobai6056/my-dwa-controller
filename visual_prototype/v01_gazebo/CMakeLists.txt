cmake_minimum_required(VERSION 3.16)
project(v01_pose LANGUAGES CXX)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GAZEBO REQUIRED IMPORTED_TARGET gazebo)
find_package(ament_cmake REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)

ament_index_has_resource(gazebo_ros_prefix packages gazebo_ros)
find_path(gazebo_ros_headers gazebo_ros/node.hpp
  PATHS "${gazebo_ros_prefix}/include" NO_DEFAULT_PATH REQUIRED)
find_library(gazebo_ros_node gazebo_ros_node
  PATHS "${gazebo_ros_prefix}/lib" NO_DEFAULT_PATH REQUIRED)

add_library(v01_pose SHARED pose_plugin.cpp)
target_compile_features(v01_pose PRIVATE cxx_std_17)
target_include_directories(v01_pose PRIVATE "${gazebo_ros_headers}")
target_link_libraries(v01_pose PkgConfig::GAZEBO "${gazebo_ros_node}")
ament_target_dependencies(v01_pose gazebo_msgs rclcpp)
