#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.hpp"

namespace my_dwa_controller
{

struct ObstacleAabb
{
  double min_x = 0.0, max_x = 0.0;
  double min_y = 0.0, max_y = 0.0;
};

struct ObstacleObservation
{
  ObstacleAabb aabb;
  double vx = 0.0, vy = 0.0;
  bool velocity_valid = false;
  // Original valid beam points in scan order, transformed once into the
  // enclosing ScanSnapshot frame at its source stamp. No fitted/filled shape.
  std::vector<std::array<double, 2>> points;
  // Independent partial surface evidence in this snapshot's frame (m/s).
  // rank 0 means unusable; rank 1 constrains one direction only; rank 2 spans 2D.
  // The vector is the observable projection, not a complete velocity. Missing
  // components must not be treated as zero motion or set velocity_valid.
  int partial_motion_rank = 0;
  // Association index into previous_stamp_ns; -1 means no adjacent association.
  int previous_cluster_index = -1;
  double partial_motion_vx = 0, partial_motion_vy = 0;
};

struct ScanSnapshot
{
  // Scan source time; consumers decide freshness. Reading never advances it.
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;
  // No retained clusters is not evidence that any spatial region is clear.
  std::vector<ObstacleObservation> obstacles;
  // Source timestamp of the adjacent Tracker association baseline, or zero.
  int64_t previous_stamp_ns = 0;
  // Exact source-time laser pose in frame_id; original polar rays are retained
  // for region-clear checks. Empty clusters alone do not certify clear space.
  std::array<double, 3> laser_pose{};
  std::vector<float> ranges;
  double angle_min = 0, angle_increment = 0, range_min = 0, range_max = 0;
};

// Serializes a single scan stream; returned snapshots are immutable.
class ScanObstacleTracker
{
public:
  explicit ScanObstacleTracker(
    std::shared_ptr<tf2_ros::Buffer> tf, double cluster_distance = 0.12,
    std::size_t min_cluster_points = 3, double association_distance = 0.35);

  // Bounds cover the visible surface only. Velocity estimates describe past
  // surface translation when two directions constrain it. Unknown is not zero motion.
  // Only 0 < dt <= 0.5 s yields velocity; longer gaps start a new baseline.
  void update(const sensor_msgs::msg::LaserScan & scan, const std::string & target_frame);
  // One bounded FIFO for TF readiness. Pending scans do not invalidate snapshot().
  // Caller polls without blocking; processing needs exact-stamp TF and ROS time
  // at or after the original Scan stamp. No future tolerance or stamp adjustment.
  void enqueue(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
  void processPending(const std::string & target_frame, const rclcpp::Time & now);
  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.reset();
    pending_.clear();
    last_processing_time_.reset();
  }

  // A new subscription gets fresh observation state. An already selected old
  // callback can then neither populate it nor retain the Controller object.
  std::shared_ptr<ScanObstacleTracker> newSession() const;

  // Null means no usable scan (including processing failure), not confirmed clear.
  // A non-null snapshot with no obstacles is a successfully processed empty frame.
  std::shared_ptr<const ScanSnapshot> snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

private:
  void updateLocked(const sensor_msgs::msg::LaserScan & scan, const std::string & target_frame);
  std::deque<sensor_msgs::msg::LaserScan::ConstSharedPtr> pending_;
  std::optional<rclcpp::Time> last_processing_time_;
  rclcpp::Clock diagnostic_clock_{RCL_STEADY_TIME};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double cluster_distance_;
  std::size_t min_cluster_points_;
  double association_distance_;
  mutable std::mutex mutex_;
  std::shared_ptr<const ScanSnapshot> snapshot_;
};

}  // namespace my_dwa_controller
