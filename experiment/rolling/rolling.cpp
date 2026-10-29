#include <fstream>
#include <iostream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include "rolling_model.hpp"
#include "my_dwa_controller/motion_predictor.hpp"
#include "my_dwa_controller/velocity_sampler.hpp"
#include "my_dwa_controller/trajectory_scorer.hpp"
#include "my_dwa_controller/path_geometry.hpp"
#include "my_dwa_controller/obstacle_evaluator.hpp"
#include "my_dwa_controller/dynamic_filter.hpp"
using Json=nlohmann::json;

ScanSnapshot observation(double t)
{
  ScanSnapshot scan;scan.stamp=rclcpp::Time(sourceNs(t),RCL_ROS_TIME);
  scan.frame_id="scene";const auto b=actor(t);ObstacleObservation o;
  o.aabb={b.min_x,b.max_x,b.min_y,b.max_y};o.velocity_valid=true;
  o.vy=t>=2 && t<10.8?.25:0;o.partial_motion_rank=2;o.partial_motion_vy=o.vy;
  for (int i=0;i<=20;++i) {
    const double u=i/20.;
    o.points.push_back({b.min_x+.5*u,b.min_y});o.points.push_back({b.min_x+.5*u,b.max_y});
    o.points.push_back({b.min_x,b.min_y+.5*u});o.points.push_back({b.max_x,b.min_y+.5*u});
  }
  scan.obstacles={o};return scan;
}
double clearance(const Trajectory & trajectory,double t,const std::vector<Box> & walls)
{
  double result=1e100;
  for (size_t i=0;i<trajectory.poses.size();++i) {
    const auto & p=trajectory.poses[i];const auto & previous=trajectory.poses[i?i-1:0];
    for (const auto & b:walls) {result=std::min(result,segmentDistance(previous,p,b)-.22);}
    const auto b=actor(t+i*.05),a=actor(t+(i?i-1:0)*.05);
    // Relative segment against a static box is exact on each original linear keyframe span.
    auto q=p;q.x-=b.min_x-a.min_x;q.y-=b.min_y-a.min_y;
    result=std::min(result,segmentDistance(previous,q,a)-.22);
  }
  return result;
}
Trajectory queuedBackup(Plant plant,Velocity2D cmd)
{
  const double t=plant.time;plant.issue(t,cmd);plant.issue(t+.05,{});
  Trajectory out{cmd,{plant.pose}};
  for (int i=1;i<=34;++i) {plant.advance(t+i*.05);out.poses.push_back(plant.pose);}
  return out;
}
int main(int argc,char ** argv)
{
  if (argc<3) {return 2;}
  Json input;std::ifstream(argv[1])>>input;
  if (input["scene"]["keyframes"]!=Json::array({Json::array({0,1.6,-1.1}),
      Json::array({2.0,1.6,-1.1}),Json::array({10.8,1.6,1.1})}) ||
      input["scene"]["size"]!=Json::array({.5,.5,.7})) {
    throw std::runtime_error("This audited model only supports the archived D01 t2 timing and size");
  }
  std::vector<Box> walls;for (const auto & b:input["walls"]) {walls.push_back({b[0],b[1],b[2],b[3]});}
  std::vector<Pose2D> path;for (const auto & p:input["path"]) {path.push_back(point(p[0],p[1]));}
  MotionConfig mc;mc.angular_acceleration=3.2;MotionPredictor predictor(mc);
  SamplingConfig config;config.max_v=.26;config.acc_v=2.5;config.acc_w=3.2;
  config.v_samples=config.w_samples=20;
  TrajectoryScorer scorer(.26);DynamicFilter filter(.22,1.7,.05);
  Json summaries=Json::array();
  std::vector<double> starts=input["start_times"].get<std::vector<double>>();
  if (argc>3) {starts.clear();for (int i=3;i<argc;++i) {starts.push_back(std::stod(argv[i]));}}
  for (double start:starts) {
    Plant plant;plant.pose=point(input["initial"][0],input["initial"][1]);plant.pose.theta=input["initial"][2];plant.time=start;
    std::ostringstream name;name<<std::fixed<<std::setprecision(2)<<start;
    std::ofstream log(std::string(argv[2])+"/start_"+name.str()+".jsonl");
    double min_clearance=1e100,first_motion=-1,entry=-1,pass=-1,last_scan=-1;
    int stops=0;bool moving=false;std::string failure="timeout";
    ScanSnapshot scan;Exclusion z;std::unique_ptr<nav2_costmap_2d::Costmap2D> map;
    Json motion_starts=Json::array();Json totals=Json::object();
    for (int cycle=0;plant.time<14-1e-8;++cycle) {
      const double t=plant.time,source_time=std::floor((t+1e-8)*5)/5;
      if (source_time!=last_scan) {
        scan=observation(source_time);z=exclusion(source_time);
        auto boxes=walls;boxes.push_back(actor(source_time));map=makeMap(plant.pose,boxes);last_scan=source_time;
      }
      const auto projection=projectOntoPath(plant.pose,path).value();
      auto local=extractForwardPath(path,projection,2.0);
      const auto commands=sampleVelocities(plant.velocity,config);
      struct Candidate {MotionRollout nominal,backup;double cost;};
      std::vector<Candidate> candidates;Json rejected={{"horizon",0},{"static_backup",0},{"static_nominal",0},
        {"exact_geometry",0},{"exclusion",0},{"dynamic",0},{"queue_backup",0}};
      const auto reject=[&](const char * key){rejected[key]=rejected[key].get<int>()+1;};
      for (const auto & cmd:commands) {
        auto b=predictor.predict(plant.pose,plant.velocity,cmd,.05);
        if (b.stop_time>1.7) {reject("horizon");continue;}
        if (!evaluateTrajectory(b.trajectory,*map).collision_free) {reject("static_backup");continue;}
        auto n=predictor.predict(plant.pose,plant.velocity,cmd,1.7);
        if (!evaluateTrajectory(n.trajectory,*map).collision_free) {reject("static_nominal");continue;}
        if (clearance(n.trajectory,t,walls)<.05 || clearance(b.trajectory,t,walls)<.05) {reject("exact_geometry");continue;}
        const double reserve=std::max(std::abs(plant.velocity.v),std::abs(cmd.v))*.05;
        if (!z.clear(n.trajectory,reserve) || !z.clear(b.trajectory,reserve)) {reject("exclusion");continue;}
        const double cost=trajectoryObstacleCost(n.trajectory,*map);
        candidates.push_back({std::move(n),std::move(b),cost});
      }
      const auto fallback=predictor.predict(plant.pose,plant.velocity,{},0);
      std::vector<const Trajectory *> queries{&fallback.trajectory};
      for (const auto & c:candidates) {queries.push_back(&c.backup.trajectory);}
      const auto prepared=filter.prepare(queries,&scan,"scene",rclcpp::Time(sourceNs(t),RCL_ROS_TIME));
      const auto fr=filter.evaluate(fallback.trajectory,prepared);
      const auto queue_fallback=queuedBackup(plant,{});
      const bool fallback_ok=fallback.stop_time<=1.7 && evaluateTrajectory(fallback.trajectory,*map).collision_free &&
        fr.available && fr.conflicts.empty() && !filter.observedGeometryCollision(fallback.trajectory,prepared) &&
        z.clear(fallback.trajectory,std::abs(plant.velocity.v)*.05) && clearance(fallback.trajectory,t,walls)>=.05 &&
        z.clear(queue_fallback,std::abs(plant.velocity.v)*.05) && clearance(queue_fallback,t,walls)>=.05;
      struct Ranked {size_t index;ScoreResult score;};std::vector<Ranked> ranked;
      for (size_t i=0;i<candidates.size();++i) {
        const auto & c=candidates[i];const auto risk=filter.evaluate(c.backup.trajectory,prepared);
        if (!risk.available || !risk.conflicts.empty() || filter.observedGeometryCollision(c.nominal.trajectory,prepared) ||
          filter.observedGeometryCollision(c.backup.trajectory,prepared)) {reject("dynamic");continue;}
        ranked.push_back({i,scorer.score(c.nominal.trajectory,local,c.cost,true)});
      }
      std::stable_sort(ranked.begin(),ranked.end(),[&](const Ranked & a,const Ranked & b){
        if (a.score.total_cost!=b.score.total_cost) {return a.score.total_cost<b.score.total_cost;}
        return std::abs(candidates[a.index].nominal.trajectory.command.omega)<
          std::abs(candidates[b.index].nominal.trajectory.command.omega);
      });
      Json best_turn=nullptr,best_rotation=nullptr;int turns=0,rotations=0,rear_reaches=0;
      for (const auto & r:ranked) {
        const auto & tr=candidates[r.index].nominal.trajectory;const auto cmd=tr.command;
        if (cmd.omega<=-.1 && cmd.v>0) {
          ++turns;if (best_turn.is_null()) {best_turn={{"command",{cmd.v,cmd.omega}},{"score",r.score.total_cost}};}
        }
        if (cmd.v==0 && cmd.omega<=-.1) {
          ++rotations;if (best_rotation.is_null()) {best_rotation={{"command",{cmd.v,cmd.omega}},{"score",r.score.total_cost}};}
        }
        for (const auto & p:tr.poses) {if (p.x>=z.lo && p.x<=z.hi && p.y<z.rear) {++rear_reaches;break;}}
      }
      Velocity2D selected{};Json selected_score=nullptr;int selected_rank=-1;
      for (size_t i=0;i<ranked.size();++i) {
        const auto & c=candidates[ranked[i].index];const auto cmd=c.nominal.trajectory.command;
        const auto qb=queuedBackup(plant,cmd);const double reserve=std::max(std::abs(plant.velocity.v),std::abs(cmd.v))*.05;
        if (!z.clear(qb,reserve) || clearance(qb,t,walls)<.05 || !evaluateTrajectory(qb,*map).collision_free) {reject("queue_backup");continue;}
        selected=cmd;const auto & s=ranked[i].score;selected_rank=i;
        selected_score={{"total",s.total_cost},{"goal",s.goal_cost},{"path",s.path_cost},{"speed",s.speed_cost},
          {"obstacle",s.obstacle_cost},{"heading",s.heading_cost}};break;
      }
      Json state={{"t",t},{"scan_t",source_time},{"pose",{plant.pose.x,plant.pose.y,plant.pose.theta}},
        {"measured",{plant.velocity.v,plant.velocity.omega}},{"selected",{selected.v,selected.omega}},
        {"sampled",commands.size()},{"ranked",ranked.size()},{"selected_rank",selected_rank},
        {"score",selected_score},{"rejected",rejected},{"fallback_ok",fallback_ok},
        {"dynamic_available",prepared.input.input_available},{"dynamic_reason",prepared.input.reason},
        {"negative_turn_ranked",turns},{"best_negative_turn",best_turn},
        {"negative_rotation_ranked",rotations},{"best_negative_rotation",best_rotation},{"rear_reaches_ranked",rear_reaches},
        {"zone",{z.lo,z.hi,z.rear}},{"map_origin",{map->getOriginX(),map->getOriginY()}}};
      for (auto it=rejected.begin();it!=rejected.end();++it) {totals[it.key()]=totals.value(it.key(),0)+it.value().get<int>();}
      if (!fallback_ok) {failure="fallback_safety_admission_lost";state["abort"]=failure;log<<state<<'\n';break;}
      plant.issue(t,selected);
      Trajectory executed{selected,{plant.pose}};
      double executed_min=1e100;
      for (int j=1;j<=10;++j) {
        const auto previous=plant.pose;const double oldtime=plant.time;plant.advance(t+j*.005);
        for (const auto & b:walls) {executed_min=std::min(executed_min,segmentDistance(previous,plant.pose,b)-.22);}
        const auto a=actor(oldtime),b=actor(plant.time);auto q=plant.pose;q.y-=b.min_y-a.min_y;
        executed_min=std::min(executed_min,segmentDistance(previous,q,a)-.22);
        executed.poses.push_back(plant.pose);
      }
      min_clearance=std::min(min_clearance,executed_min);state["executed_clearance"]=executed_min;
      state["next_pose"]={plant.pose.x,plant.pose.y,plant.pose.theta};log<<state<<'\n';
      if (executed_min<=0 || !z.clear(executed,0)) {failure="executed_contact_or_exclusion_violation";break;}
      const bool now_moving=std::abs(plant.velocity.v)>=.01 || std::abs(plant.velocity.omega)>=.05;
      if (now_moving && !moving) {motion_starts.push_back(plant.time);if (first_motion<0) {first_motion=plant.time;}}
      if (!now_moving && moving) {++stops;}moving=now_moving;
      if (entry<0 && plant.pose.x>=z.lo && plant.pose.y<z.rear) {entry=plant.time;}
      if (entry>=0 && passed(plant.pose,z)) {pass=plant.time;failure="";break;}
    }
    Json summary={{"start",start},{"passed",pass>=0},{"first_motion",first_motion},{"rear_entry",entry},
      {"pass_time",pass},{"minimum_clearance",min_clearance},{"stops",stops},{"motion_starts",motion_starts},
      {"failure",failure},{"final_pose",{plant.pose.x,plant.pose.y,plant.pose.theta}},{"rejections",totals}};
    summaries.push_back(summary);std::cout<<summary.dump()<<std::endl;
  }
  std::ofstream(std::string(argv[2])+"/summary.json")<<summaries.dump(2);
}
