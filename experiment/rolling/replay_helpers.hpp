#include <fstream>
#include <iostream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include "rolling_model.hpp"
#include "strategy.hpp"
#include "reference.hpp"
#include "my_dwa_controller/crossing_yield.hpp"
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
