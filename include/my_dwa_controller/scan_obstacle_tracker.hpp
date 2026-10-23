#ifndef MY_DWA_CONTROLLER__SCAN_OBSTACLE_TRACKER_HPP_
#define MY_DWA_CONTROLLER__SCAN_OBSTACLE_TRACKER_HPP_

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "my_dwa_controller/types.hpp"

namespace my_dwa_controller
{
struct ScanTrackerConfig
{
  std::string topic = "/scan";
  std::string fixed_frame = "odom";
  double cluster_gap = 0.12;
  double max_cluster_size = 0.9;
  double association_gate = 0.35;
  double fit_window = 0.8;
  double max_age = 0.35;
  double retention = 0.8;
};

enum class ScanMotion {Unknown, Stationary, Moving};
const char * scanMotionName(ScanMotion motion);

struct ScanRay
{
  Pose2D origin, end;
  double min_range = 0.0;
};

struct ScanObstacleTrack
{
  std::size_t id = 0;
  double x = 0.0, y = 0.0, vx = 0.0, vy = 0.0;
  double radius = 0.0, confidence = 0.0, residual = 0.0;
  double stamp = 0.0, age = 0.0, span = 0.0;
  std::size_t samples = 0;
  bool observed = false;
  ScanMotion motion = ScanMotion::Unknown;
  double support_change = 0.0;
  double half_x = 0.0, half_y = 0.0;
};

struct ScanTrackSnapshot
{
  std::vector<ScanObstacleTrack> tracks;
  std::vector<ScanRay> rays;
  std::string frame;
  double stamp = 0.0;
  std::size_t sequence = 0, tf_failures = 0, dropped = 0;
  bool fresh = false;
  // Diagnostic only: ordinary unknown clusters do not own dynamic risk lifetime.
  bool unresolved = false;
};

/// Subscription only queues scans. snapshot() processes exact-time TF when ready;
/// it never substitutes latest TF, or treats a failed transform as an empty scan.
class ScanObstacleTracker
{
public:
  explicit ScanObstacleTracker(std::shared_ptr<tf2_ros::Buffer> tf, ScanTrackerConfig config = {});
  void start(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void stop();
  void enqueue(const sensor_msgs::msg::LaserScan & scan);
  ScanTrackSnapshot snapshot(const std::string & target_frame, double now);
  void reset();

private:
  struct Inbox
  {
    std::mutex mutex;
    std::deque<sensor_msgs::msg::LaserScan> scans;
    bool accepting = true;
    std::size_t dropped = 0;
  };
  struct Sample {double t, x, y, width_x, width_y;};
  struct Track
  {
    ScanObstacleTrack value;
    std::deque<Sample> history;
  };
  struct Cluster {double x, y, radius, width_x, width_y;};
  bool process(const sensor_msgs::msg::LaserScan & scan);
  void associate(const std::vector<Cluster> & clusters, double stamp);
  void fit(Track & track);
  std::shared_ptr<tf2_ros::Buffer> tf_;
  ScanTrackerConfig config_;
  std::shared_ptr<Inbox> inbox_ = std::make_shared<Inbox>();
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::vector<Track> tracks_;
  double stamp_ = 0.0, last_query_ = 0.0;
  std::size_t sequence_ = 0, next_id_ = 1, tf_failures_ = 0;
  std::vector<ScanRay> rays_;
};
}  // namespace my_dwa_controller
#endif  // MY_DWA_CONTROLLER__SCAN_OBSTACLE_TRACKER_HPP_
