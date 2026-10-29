#include "replay_helpers.hpp"
Json scoreTerms(const ScoreResult & s)
{return {{"goal",s.goal_cost},{"path",s.path_cost},{"speed",s.speed_cost},{"obstacle",s.obstacle_cost},{"heading",s.heading_cost},{"total",s.total_cost}};}
int main(int argc,char ** argv)
{
  if (argc!=4) {return 2;}
  Json input;std::ifstream(argv[1])>>input;
  const double heading_multiplier=input["reference_parameters"].value("heading_multiplier",1.);
  std::vector<Box> walls;for (const auto & b:input["walls"]) {walls.push_back({b[0],b[1],b[2],b[3]});}
  std::vector<Pose2D> path;for (const auto & p:input["path"]) {path.push_back(point(p[0],p[1]));}
  MotionConfig mc;mc.angular_acceleration=3.2;MotionPredictor predictor(mc);TrajectoryScorer scorer(.26);DynamicFilter filter(.22,1.7,.05);
  Plant plant;plant.pose=point(input["initial"][0],input["initial"][1]);plant.pose.theta=input["initial"][2];plant.time=4;
  double last_scan=-1;std::unique_ptr<nav2_costmap_2d::Costmap2D> map;
  std::ifstream file(argv[2]);std::string line;Json result=Json::array();
  while (std::getline(file,line)) {
    const auto r=Json::parse(line);const double t=r["t"],source=r["scan_t"];
    if (result.empty() && last_scan<0) {plant.time=t;}
    if (last_scan!=source) {auto boxes=walls;boxes.push_back(actor(source));map=makeMap(plant.pose,boxes);last_scan=source;}
    if (std::hypot(plant.pose.x-r["pose"][0].get<double>(),plant.pose.y-r["pose"][1].get<double>())>1e-10) {return 3;}
    bool chosen=false;for (double key:{5.,6.,7.,8.,8.4,8.6,8.8,9.,10.,12.,14.,15.95}) {chosen=chosen || std::abs(t-key)<1e-6;}
    if (chosen && r["guided_scoring"].get<bool>()) {
      const auto ref=r["reference"]["target"].is_null()?r["reference"]["proposal"]:r["reference"]["target"];
      const auto target=point(ref[0],ref[1]);
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
          risk.available && risk.conflicts.empty() && !filter.observedGeometryCollision(n.trajectory,prepared) && !filter.observedGeometryCollision(b.trajectory,prepared);
        const auto ordinary=scorer.score(n.trajectory,local,trajectoryObstacleCost(n.trajectory,*map),true);
        const auto guided=referenceScore(ordinary,n.trajectory,plant.pose,target,heading_multiplier);
        return Json{{"command",c},{"admitted",admitted},{"ordinary_terms_same_action",scoreTerms(ordinary)},
          {"guided_terms",scoreTerms(guided)},{"nominal_end",{n.trajectory.poses.back().x,n.trajectory.poses.back().y,n.trajectory.poses.back().theta}}};
      };
      choices["selected_proposal"]=examine(r["proposed_candidate"]);
      choices["actual_command"]=r["selected"];choices["held"]=r["command_held"];
      if (std::abs(choices["selected_proposal"]["guided_terms"]["total"].get<double>()-r["score"]["total"].get<double>())>1e-12) {return 4;}
      choices["best_negative_turn"]=r["best_negative_turn"].is_null()?Json(nullptr):examine(r["best_negative_turn"]["command"]);
      SamplingConfig config;config.max_v=.26;config.acc_v=2.5;config.acc_w=3.2;config.v_samples=config.w_samples=20;
      const auto samples=sampleVelocities(plant.velocity,config);Json aligned=nullptr,fastest=nullptr,ordinary_best=nullptr;
      for (const auto & cmd:samples) {
        auto check=examine(Json::array({cmd.v,cmd.omega}));if (!check["admitted"].get<bool>()) {continue;}
        if (fastest.is_null() || check["command"][0]>fastest["command"][0] ||
          (check["command"][0]==fastest["command"][0] && check["guided_terms"]["total"]<fastest["guided_terms"]["total"])) {fastest=check;}
        if (ordinary_best.is_null() || check["ordinary_terms_same_action"]["total"]<ordinary_best["ordinary_terms_same_action"]["total"]) {ordinary_best=check;}
        if (aligned.is_null() || check["guided_terms"]["heading"].get<double>()<aligned["guided_terms"]["heading"].get<double>()-1e-12 ||
          (std::abs(check["guided_terms"]["heading"].get<double>()-aligned["guided_terms"]["heading"].get<double>())<=1e-12 &&
           check["guided_terms"]["total"]<aligned["guided_terms"]["total"])) {aligned=check;}
      }
      choices["most_heading_aligned_admitted"]=aligned;
      choices["fastest_admitted"]=fastest;choices["best_ordinary_admitted"]=ordinary_best;
      result.push_back({{"t",t},{"desired_yaw",r["reference"]["desired_yaw"]},{"pose",r["pose"]},
        {"measured",r["measured"]},{"choices",choices}});
    }
    plant.issue(t,{r["selected"][0],r["selected"][1]});for (int j=1;j<=10;++j) {plant.advance(t+j*.005);}
  }
  std::ofstream(argv[3])<<result.dump(2);std::cout<<result.dump(2)<<'\n';
}
