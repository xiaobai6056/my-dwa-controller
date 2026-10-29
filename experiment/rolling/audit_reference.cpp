#include "replay_helpers.hpp"
Json scoreTerms(const ScoreResult & s)
{return {{"goal",s.goal_cost},{"path",s.path_cost},{"speed",s.speed_cost},{"obstacle",s.obstacle_cost},{"heading",s.heading_cost},{"total",s.total_cost}};}
int main()
{
  const std::string root="/home/user/ros2_experiments/locked_reference_20261029";
  Json input;std::ifstream(root+"/input.json")>>input;
  std::vector<Box> walls;for (const auto & b:input["walls"]) {walls.push_back({b[0],b[1],b[2],b[3]});}
  std::vector<Pose2D> path;for (const auto & p:input["path"]) {path.push_back(point(p[0],p[1]));}
  MotionConfig mc;mc.angular_acceleration=3.2;MotionPredictor predictor(mc);TrajectoryScorer scorer(.26);DynamicFilter filter(.22,1.7,.05);
  Plant plant;plant.pose=point(input["initial"][0],input["initial"][1]);plant.pose.theta=input["initial"][2];plant.time=4;
  double last_scan=-1;std::unique_ptr<nav2_costmap_2d::Costmap2D> map;
  std::ifstream file(root+"/results/phase1/B_t4/start_4.00.jsonl");std::string line;Json result=Json::array();
  while (std::getline(file,line)) {
    const auto r=Json::parse(line);const double t=r["t"],source=r["scan_t"];
    if (last_scan!=source) {auto boxes=walls;boxes.push_back(actor(source));map=makeMap(plant.pose,boxes);last_scan=source;}
    if (std::hypot(plant.pose.x-r["pose"][0].get<double>(),plant.pose.y-r["pose"][1].get<double>())>1e-10) {return 3;}
    bool chosen=false;for (double key:{5.,6.,8.,8.4,8.6}) {chosen=chosen || std::abs(t-key)<1e-6;}
    if (chosen) {
      const auto target=point(r["reference"]["target"][0],r["reference"]["target"][1]);
      const auto z=exclusion(source);const auto local=extractForwardPath(path,projectOntoPath(plant.pose,path).value(),2.0);
      Json choices=Json::object();
      const auto examine=[&](const Json & c) {
        const Velocity2D cmd{c[0],c[1]};const auto n=predictor.predict(plant.pose,plant.velocity,cmd,1.7),b=predictor.predict(plant.pose,plant.velocity,cmd,.05);
        const auto qb=queuedBackup(plant,cmd);const double reserve=std::max(std::abs(plant.velocity.v),std::abs(cmd.v))*.05;
        const auto snapshot=observation(source);const auto prepared=filter.prepare({&b.trajectory},&snapshot,"scene",rclcpp::Time(sourceNs(t),RCL_ROS_TIME));
        const auto risk=filter.evaluate(b.trajectory,prepared);
        const bool admitted=b.stop_time<=1.7 && evaluateTrajectory(n.trajectory,*map).collision_free &&
          evaluateTrajectory(b.trajectory,*map).collision_free && evaluateTrajectory(qb,*map).collision_free &&
          z.clear(n.trajectory,reserve) && z.clear(b.trajectory,reserve) && z.clear(qb,reserve) &&
          clearance(n.trajectory,t,walls)>=.05 && clearance(b.trajectory,t,walls)>=.05 && clearance(qb,t,walls)>=.05 &&
          risk.available && risk.conflicts.empty();
        const auto ordinary=scorer.score(n.trajectory,local,trajectoryObstacleCost(n.trajectory,*map),true);
        const auto guided=referenceScore(ordinary,n.trajectory,plant.pose,target);
        return Json{{"command",c},{"admitted",admitted},{"ordinary_terms_same_action",scoreTerms(ordinary)},
          {"guided_terms",scoreTerms(guided)},{"nominal_end",{n.trajectory.poses.back().x,n.trajectory.poses.back().y,n.trajectory.poses.back().theta}}};
      };
      choices["selected"]=examine(r["selected"]);
      if (std::abs(choices["selected"]["guided_terms"]["total"].get<double>()-r["score"]["total"].get<double>())>1e-12) {return 4;}
      choices["best_negative_turn"]=r["best_negative_turn"].is_null()?Json(nullptr):examine(r["best_negative_turn"]["command"]);
      SamplingConfig config;config.max_v=.26;config.acc_v=2.5;config.acc_w=3.2;config.v_samples=config.w_samples=20;
      const auto samples=sampleVelocities(plant.velocity,config);Json aligned=nullptr;
      for (const auto & cmd:samples) {
        auto check=examine(Json::array({cmd.v,cmd.omega}));if (!check["admitted"].get<bool>()) {continue;}
        if (aligned.is_null() || check["guided_terms"]["heading"].get<double>()<aligned["guided_terms"]["heading"].get<double>()-1e-12 ||
          (std::abs(check["guided_terms"]["heading"].get<double>()-aligned["guided_terms"]["heading"].get<double>())<=1e-12 &&
           check["guided_terms"]["total"]<aligned["guided_terms"]["total"])) {aligned=check;}
      }
      choices["most_heading_aligned_admitted"]=aligned;
      result.push_back({{"t",t},{"desired_yaw",r["reference"]["desired_yaw"]},{"pose",r["pose"]},
        {"measured",r["measured"]},{"choices",choices}});
    }
    plant.issue(t,{r["selected"][0],r["selected"][1]});for (int j=1;j<=10;++j) {plant.advance(t+j*.005);}
  }
  std::ofstream(root+"/score_audit.json")<<result.dump(2);std::cout<<result.dump(2)<<'\n';
}
