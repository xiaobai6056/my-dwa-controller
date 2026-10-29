#pragma once
// Replays saved candidate inputs through the actual original/new exported libraries.
// No ROS node, simulator, new candidates, velocity fitting, or scoring formula.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "my_dwa_controller/trajectory_predictor.hpp"
#include "my_dwa_controller/dynamic_trajectory_evaluator.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
using Json = nlohmann::json;
using namespace my_dwa_controller;
void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}
template<typename M> M decode(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & rec)
{
  rclcpp::SerializedMessage serialized(*rec->serialized_data);M msg;
  rclcpp::Serialization<M>().deserialize_message(&serialized, &msg);return msg;
}
struct Bag {std::shared_ptr<tf2_ros::Buffer> tf;
  std::map<int64_t, sensor_msgs::msg::LaserScan> scans;};
Bag loadBag(const std::string & path)
{
  Bag data;data.tf = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(
      RCL_ROS_TIME), tf2::durationFromSec(1000.));data.tf->setUsingDedicatedThread(true);
  rosbag2_cpp::Reader reader;reader.open(path);
  while (reader.has_next()) {
    auto rec = reader.read_next();if (rec->topic_name == "/tf" || rec->topic_name == "/tf_static") {
      for (auto & t:decode<tf2_msgs::msg::TFMessage>(rec).transforms) {
        data.tf->setTransform(t, "saved_bag", rec->topic_name == "/tf_static");
      }
    } else if (rec->topic_name == "/scan") {
      auto scan = decode<sensor_msgs::msg::LaserScan>(rec);data.scans[rclcpp::Time(
          scan.header.stamp).nanoseconds()] = std::move(scan);
    }
  }
  return data;
}
