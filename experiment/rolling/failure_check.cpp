#include "replay_helpers.hpp"
int main(int argc,char **argv){
 if(argc!=4)return 2;Json input;std::ifstream(argv[1])>>input;
 std::vector<Box> walls;for(auto b:input["walls"])walls.push_back({b[0],b[1],b[2],b[3]});
 Plant plant;plant.pose=point(input["initial"][0],input["initial"][1]);plant.pose.theta=input["initial"][2];plant.time=4;
 MotionConfig mc;mc.angular_acceleration=3.2;MotionPredictor predictor(mc);
 std::unique_ptr<nav2_costmap_2d::Costmap2D> map;double last=-1;std::ifstream file(argv[2]);std::string line;
 while(std::getline(file,line)){
  auto r=Json::parse(line);double t=r["t"],source=r["scan_t"];
  if(source!=last){auto boxes=walls;boxes.push_back(actor(source));map=makeMap(plant.pose,boxes);last=source;}
  if(std::hypot(plant.pose.x-r["pose"][0].get<double>(),plant.pose.y-r["pose"][1].get<double>())>1e-10)return 3;
  if(r.contains("abort")){
   auto f=predictor.predict(plant.pose,plant.velocity,{},0);auto q=queuedBackup(plant,{});auto z=exclusion(source);double reserve=std::abs(plant.velocity.v)*.05;
   Json result={{"t",t},{"pose",r["pose"]},{"measured",r["measured"]},{"zone",r["zone"]},{"reserve",reserve},
    {"fallback_stop_time",f.stop_time},{"static_fallback",evaluateTrajectory(f.trajectory,*map).collision_free},
    {"fallback_exclusion",z.clear(f.trajectory,reserve)},{"queued_fallback_exclusion",z.clear(q,reserve)},
    {"fallback_truth_clearance",clearance(f.trajectory,t,walls)},{"queued_fallback_truth_clearance",clearance(q,t,walls)},
    {"fallback_poses",Json::array()},{"queue_poses",Json::array()}};
   for(auto p:f.trajectory.poses)result["fallback_poses"].push_back({p.x,p.y,p.theta});
   for(auto p:q.poses)result["queue_poses"].push_back({p.x,p.y,p.theta});
   std::ofstream(argv[3])<<result.dump(2);std::cout<<result.dump(2);return 0;
  }
  plant.issue(t,{r["selected"][0],r["selected"][1]});for(int j=1;j<=10;++j)plant.advance(t+j*.005);
 }
 return 4;
}
