#pragma once
#include <deque>
#include <memory>
#include "../probe_geometry.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
struct Box {double min_x,min_y,max_x,max_y;};
inline int64_t sourceNs(double t) {return std::llround((t+1)*1e9);}
Box actor(double t);
double distance(const Pose2D &,const Box &);
double segmentDistance(const Pose2D &,const Pose2D &,const Box &);
Exclusion exclusion(double t);
bool passed(const Pose2D &,const Exclusion &);
std::unique_ptr<nav2_costmap_2d::Costmap2D> makeMap(const Pose2D &,const std::vector<Box> &);
struct Plant
{
  Pose2D pose;Velocity2D velocity{};double time=0;
  std::deque<std::pair<double,Velocity2D>> queue;
  void issue(double t,Velocity2D v);
  void advance(double end);
};
