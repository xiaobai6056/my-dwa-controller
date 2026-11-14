// Offline structural parity probe. Source snapshots use all recorded Scans and
// exact-stamp TF; decision inputs use the archived consumed-frame record.
#include "common.hpp"
#include "my_dwa_controller/crossing_yield.hpp"
#include "my_dwa_controller/motion_predictor.hpp"

Json decision(const YieldDecision & y)
{
  return {{"hold", y.hold}, {"triggered", y.triggered}, {"released", y.released},
    {"occupied", y.occupied}, {"clear", y.clear}, {"safe_stop", y.safe_stop},
    {"clear_scans", y.clear_scans}, {"rank", y.evidence_rank},
    {"entry", y.entry}, {"exit", y.exit}, {"margin", y.margin}};
}

Pose2D pose(const Json & p)
{
  Pose2D result;
  result.x = p[0];result.y = p[1];result.theta = p.size() > 2 ? p[2].get<double>() : 0;
  return result;
}

int main(int argc, char ** argv)
{
  require(argc == 4, "usage: replay_history FROZEN_FRAMES RUN_DIR OUTPUT");
  Json input;std::ifstream(argv[1]) >> input;
  auto bag = loadBag(std::string(argv[2]) + "/bag");
  ScanObstacleTracker tracker(bag.tf);
  std::map<int64_t, std::shared_ptr<const ScanSnapshot>> snapshots;
  int failed_tf = 0;
  for (const auto & entry:bag.scans) {
    tracker.update(entry.second, "odom");
    if (auto s = tracker.snapshot()) {snapshots[entry.first] = s;} else {++failed_tf;}
  }
  MotionConfig config;config.angular_acceleration = 3.2;
  MotionPredictor predictor(config);
  CrossingYield policy;
  Json rows = Json::array();
  int point_mismatches = 0, velocity_mismatches = 0;
  for (const auto & f:input.at("frames")) {
    const int64_t stamp = f.at("stamp_ns");
    require(snapshots.count(stamp), "missing source Scan/TF");
    const auto & s = *snapshots.at(stamp);
    const auto & d = f.at("decision");
    const Pose2D robot = pose(d.is_null() ? f.at("robot") : d.at("initial"));
    const auto & v = d.is_null() ? f.at("velocity") : d.at("measured");
    Velocity2D measured{v[0], v[1]};
    auto brake = predictor.predict(robot, measured, {0, 0}, 0).trajectory;
    std::vector<Pose2D> path;
    for (const auto & p:f.at("path")) {
      path.push_back(pose(p));
    }
    const int64_t now = d.is_null() ? stamp : d.at("now_ns").get<int64_t>();
    const bool admission = !d.is_null() && d.at("accepted").get<int>() > 0;
    Json obs = Json::array();
    point_mismatches += s.obstacles.size() != f.at("observations").size();
    for (size_t i = 0; i < s.obstacles.size(); ++i) {
      const auto & o = s.obstacles[i];
      obs.push_back(
        {{"points", o.points}, {"velocity", {o.vx, o.vy}},
          {"velocity_valid", o.velocity_valid}, {"partial_rank", o.partial_motion_rank},
          {"partial_velocity", {o.partial_motion_vx, o.partial_motion_vy}},
          {"previous_cluster_index", o.previous_cluster_index}});
      if (i < f.at("observations").size()) {
        const auto & old = f.at("observations")[i];
        point_mismatches += Json(o.points) != old.at("points");
        velocity_mismatches += o.velocity_valid != old.at("velocity_valid").get<bool>() ||
          Json::array({o.vx, o.vy}) != old.at("velocity");
      }
    }
    Json row = {{"stamp_ns", stamp}, {"now_ns", now}, {"observations", obs},
      {"decision", decision(policy.update(&s, path, robot, measured, brake, now, admission))}};
    // A repeated read must not create a new clear Scan. Exercise both current
    // admission choices without synthesizing a new measurement timestamp.
    row["duplicate_no_admission"] =
      decision(policy.update(&s, path, robot, measured, brake, now, false));
    row["duplicate_admission"] =
      decision(policy.update(&s, path, robot, measured, brake, now, admission));
    rows.push_back(row);
  }
  std::ofstream(argv[3]) << Json{{"id", input.at("id")}, {"rows", rows},
    {"bag_scans", bag.scans.size()}, {"failed_tf", failed_tf},
    {"archived_point_mismatches", point_mismatches},
    {"archived_full_velocity_mismatches", velocity_mismatches}}.dump();
  std::cout << input.at("id") << " frames=" << rows.size() << " scans=" << bag.scans.size()
            << " failed_tf=" << failed_tf << " points_mismatch=" << point_mismatches
            << " full_velocity_mismatch=" << velocity_mismatches << '\n';
}
