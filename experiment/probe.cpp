#include "common.hpp"
#include "probe_geometry.hpp"
#include "my_dwa_controller/path_handler.hpp"
#include "my_dwa_controller/velocity_sampler.hpp"
#include "my_dwa_controller/motion_predictor.hpp"
#include "my_dwa_controller/dynamic_filter.hpp"

int main(int argc, char ** argv)
{
  require(argc == 3, "probe INPUT OUTPUT_ROOT");
  Json input;std::ifstream(argv[1]) >> input;
  auto bag = loadBag(input.at("run").get<std::string>() + "/bag");
  ScanObstacleTracker tracker(bag.tf);
  std::map<int64_t, std::shared_ptr<const ScanSnapshot>> snapshots;
  for (const auto & entry : bag.scans) {
    tracker.update(entry.second,"odom");
    if (auto s = tracker.snapshot()) {snapshots[entry.first] = s;}
  }
  MotionConfig mc;mc.angular_acceleration = 3.2;
  MotionPredictor predictor(mc);PathHandler handler(bag.tf);DynamicFilter filter(.22,1.7,.05);
  SamplingConfig config;config.max_v = .26;config.max_w = 1;config.acc_v = 2.5;
  config.acc_w = 3.2;config.v_samples = config.w_samples = 20;
  Exclusion z;bool pending = false;
  std::ofstream rows(std::string(argv[2])+"/cycles.jsonl");
  std::ofstream candidates(std::string(argv[2])+"/candidates.jsonl");
  int pending_cycles = 0, valid_cycles = 0, rear_cycles = 0, rear_full_cycles = 0;
  int rejected = 0, rear_total = 0, rear_full_total = 0;
  for (const auto & row : input.at("rows")) {
    const auto & d = row.at("decision");const auto & y = row.at("yield");
    const int64_t stamp = d.at("scan_ns"), now = d.at("now_ns");
    if (row.at("plan").is_null() || !snapshots.count(stamp)) {continue;}
    const auto snapshot = snapshots.at(stamp);
    auto robot = point(d["initial"][0],d["initial"][1]);robot.theta = d["initial"][2];
    Velocity2D measured{d["measured"][0],d["measured"][1]};
    geometry_msgs::msg::PoseStamped rp;rp.header.frame_id = "odom";
    rp.header.stamp = rclcpp::Time(now);rp.pose.position.x = robot.x;
    rp.pose.position.y = robot.y;rp.pose.orientation.z = std::sin(robot.theta/2);
    rp.pose.orientation.w = std::cos(robot.theta/2);
    nav_msgs::msg::Path plan;plan.header.frame_id = row["plan"]["frame"];
    for (const auto & p : row["plan"]["poses"]) {
      geometry_msgs::msg::PoseStamped ps;ps.pose.position.x = p[0];ps.pose.position.y = p[1];
      ps.pose.orientation.z = p[2];ps.pose.orientation.w = p[3];plan.poses.push_back(ps);
    }
    std::vector<Pose2D> path;
    try {path = handler.getLocalPath(plan,rp,"odom",2.0);} catch (const std::exception & e) {
      rows << Json{{"cycle",d["cycle"]},{"error",e.what()}} << '\n';continue;
    }
    if (y.at("triggered").get<bool>()) {
      const auto & a = path.front();const auto & b = path.back();
      const double len = std::hypot(b.x-a.x,b.y-a.y), tx = (b.x-a.x)/len, ty = (b.y-a.y)/len;
      for (size_t i = 0; i < snapshot->obstacles.size(); ++i) {
        const auto & o = snapshot->obstacles[i];if (o.partial_motion_rank <= 0 || o.points.empty()) {continue;}
        double side = 0;for (const auto & p : o.points) {side += -ty*(p[0]-a.x)+tx*(p[1]-a.y);}
        if (-std::copysign(1.,side)*(-ty*o.partial_motion_vx+tx*o.partial_motion_vy) <= .10) {continue;}
        z.start(*snapshot,i,tx,ty);pending = true;break;
      }
    } else if (pending) {z.observe(*snapshot,now);}
    if (y.at("released").get<bool>()) {pending = false;}
    if (!pending) {continue;}
    ++pending_cycles;
    if (!z.valid) {
      rows << Json{{"cycle",d["cycle"]},{"now_ns",now},{"valid",false},{"scan_ns",stamp}} << '\n';continue;
    }
    ++valid_cycles;
    auto commands = sampleVelocities(measured,config);
    std::vector<MotionRollout> nominal, backup;
    for (const auto & cmd : commands) {
      nominal.push_back(predictor.predict(robot,measured,cmd,1.7));
      backup.push_back(predictor.predict(robot,measured,cmd,.05));
    }
    std::vector<const Trajectory *> queries;
    for (const auto & b : backup) {queries.push_back(&b.trajectory);}
    const auto prepared = filter.prepare(queries,snapshot.get(),"odom",rclcpp::Time(now,RCL_ROS_TIME));
    int rear = 0, full = 0, denied = 0, original_safe = 0, allowed = 0;
    for (size_t i = 0; i < commands.size(); ++i) {
      const auto & n = nominal[i].trajectory;const auto & b = backup[i].trajectory;
      const double reserve = std::max(std::abs(measured.v),std::abs(commands[i].v))*.05;
      const bool zone_clear = z.clear(n,reserve) && z.clear(b,reserve);
      const auto risk = filter.evaluate(b,prepared);
      const bool safe = backup[i].stop_time <= 1.7 && risk.available && risk.conflicts.empty() &&
        !filter.observedGeometryCollision(n,prepared) && !filter.observedGeometryCollision(b,prepared);
      if (safe) {++original_safe;}
      if (safe && zone_clear) {++allowed;}
      double max_s = -1e100, min_q = 1e100;
      for (const auto & p : n.poses) {max_s = std::max(max_s,z.along(p));min_q = std::min(min_q,z.across(p));}
      // A rear candidate must actually reach the expanded obstacle's longitudinal
      // slab while remaining behind it; stopping before it is only an approach.
      const bool reaches = zone_clear && max_s >= z.lo-reserve && commands[i].v > 0;
      const bool passes = reaches && max_s > z.hi+reserve;
      const bool front_attempt = !zone_clear && min_q > z.front+reserve;
      if (!zone_clear) {++denied;}
      if (safe && reaches) {++rear;}
      if (safe && passes) {++full;}
      candidates << Json{{"cycle",d["cycle"]},{"command",{commands[i].v,commands[i].omega}},
        {"zone_clear",zone_clear},{"original_nonstatic_safe",safe},{"rear_reaches",reaches},
        {"front_attempt",front_attempt},
        {"rear_passes",passes},{"max_along",max_s},{"min_across",min_q}} << '\n';
    }
    rows << Json{{"cycle",d["cycle"]},{"now_ns",now},{"scan_ns",stamp},{"valid",true},
      {"hold",y["hold"]},{"dynamic_input_available",prepared.input.input_available},
      {"dynamic_input_reason",prepared.input.reason},
      {"lo",z.lo},{"hi",z.hi},{"rear",z.rear},{"front",z.front},{"tangent",{z.tx,z.ty}},
      {"direction_sign",z.sign},{"sampled",commands.size()},{"zone_rejected",denied},
      {"original_nonstatic_safe",original_safe},{"zone_and_nonstatic_safe",allowed},
      {"rear_reaches",rear},{"rear_passes",full},{"recorded_static_feasible",d["static_feasible"]}} << '\n';
    rear_cycles += rear > 0;rear_full_cycles += full > 0;rejected += denied;
    rear_total += rear;rear_full_total += full;
  }
  Json summary{{"pending_cycles",pending_cycles},{"valid_direction_cycles",valid_cycles},
    {"rear_reaches_cycles",rear_cycles},{"rear_passes_cycles",rear_full_cycles},
    {"zone_rejected",rejected},{"rear_reaches",rear_total},{"rear_passes",rear_full_total},
    {"static_costmap_replayed",false},{"closed_loop_authorized_by_offline",false}};
  std::ofstream(std::string(argv[2])+"/summary.json") << summary.dump(2);std::cout << summary.dump(2) << '\n';
}
