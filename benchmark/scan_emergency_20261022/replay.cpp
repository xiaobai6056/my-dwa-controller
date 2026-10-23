#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "my_dwa_controller/path_geometry.hpp"
#include "my_dwa_controller/risk_governor.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3) {return 2;}
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("offline_scan_emergency_replay");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  my_dwa_controller::ScanObstacleTracker tracker(tf);
  my_dwa_controller::PathConflictConfig config;
  config.robot_radius = .23414213530762959;
  my_dwa_controller::PathConflictMonitor monitor(config);
  my_dwa_controller::RiskGovernorConfig risk_config;
  risk_config.robot_radius = config.robot_radius;
  my_dwa_controller::RiskGovernor governor(risk_config);
  std::ifstream input(argv[1]);
  std::ofstream output(std::string(argv[2]) + "_risk.csv");
  std::ofstream tracks(std::string(argv[2]) + "_tracks.csv");
  output <<
    "elapsed,state,cap,actual_v,inner,predicted,area,known,rays_clear,map_clear,clear,release,aligned,map_state,scan_new,area_x,area_y,half_length,lateral_min\n";
  tracks << "elapsed,id,x,y,vx,vy,motion,confidence,support,observed,conflict\n";
  output << std::setprecision(10);
  tracks << std::setprecision(10);
  double epoch, now, stamp, sx, sy, yaw;
  input >> epoch;
  while (input >> now >> stamp >> sx >> sy >> yaw) {
    my_dwa_controller::Pose2D robot;
    my_dwa_controller::Velocity2D velocity;
    sensor_msgs::msg::LaserScan scan;
    std::size_t n;
    input >> robot.x >> robot.y >> robot.theta >> velocity.v >> velocity.omega >> n;
    input >> scan.angle_min >> scan.angle_increment >> scan.range_min >> scan.range_max;
    scan.header.frame_id = "recorded_laser";
    scan.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    for (std::size_t i = 0; i < n; ++i) {
      double r;
      input >> r;
      scan.ranges.push_back(r < 0 ? std::numeric_limits<float>::quiet_NaN() : r);
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = scan.header.frame_id;
    transform.header.stamp = scan.header.stamp;
    transform.transform.translation.x = sx;
    transform.transform.translation.y = sy;
    transform.transform.rotation.z = std::sin(yaw / 2);
    transform.transform.rotation.w = std::cos(yaw / 2);
    tf->setTransform(transform, "recorded_tf");
    input >> n;
    std::vector<my_dwa_controller::Pose2D> path(n);
    for (auto & p : path) {
      input >> p.x >> p.y >> p.theta;
    }
    unsigned int w, h;
    double resolution, ox, oy;
    input >> w >> h >> resolution >> ox >> oy;
    nav2_costmap_2d::Costmap2D map(w, h, resolution, ox, oy, 0);
    for (unsigned int i = 0; i < w * h; ++i) {
      unsigned int cost;
      input >> cost;
      map.getCharMap()[i] = cost;
    }
    tracker.enqueue(scan);
    const auto observed = tracker.snapshot("odom", now);
    const auto e = monitor.observe(path, robot, velocity, map, now, &observed);
    const auto d = governor.decide(e, velocity, .26);
    if (d.release_scan_risk) {monitor.releaseScanRisk();}
    output << now - epoch << ',' << my_dwa_controller::riskStateName(d.state) << ',' <<
      d.max_linear_speed << ',' << velocity.v << ',' << e.inner_conflict << ',' <<
      e.scan_inner_conflict << ',' << e.scan_area_active << ',' << e.scan_area_known << ',' <<
      e.scan_area_rays_clear << ',' << e.scan_area_map_clear << ',' << e.scan_area_clear << ',' <<
      d.release_scan_risk << ',' << e.path_aligned << ',' << static_cast<int>(e.map_state) << ',' <<
      e.scan_new_state << ',' << e.scan_area.center.x << ',' << e.scan_area.center.y << ',' <<
      e.scan_area.half_length << ',' << e.scan_area.lateral_min << '\n';
    for (const auto & relation : e.scan_relations) {
      const auto & t = relation.track;
      tracks << stamp - epoch << ',' << t.id << ',' << t.x << ',' << t.y << ',' << t.vx << ',' <<
        t.vy << ',' << my_dwa_controller::scanMotionName(t.motion) << ',' << t.confidence << ',' <<
        t.support_change << ',' << t.observed << ',' << relation.conflict << '\n';
    }
  }
  rclcpp::shutdown();
  return input.eof() ? 0 : 1;
}
