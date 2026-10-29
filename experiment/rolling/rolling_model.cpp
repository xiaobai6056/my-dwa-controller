#include "rolling_model.hpp"
Box actor(double t)
{
  const double y=-1.1+.25*std::clamp(t-2.,0.,8.8);
  return {1.35,y-.25,1.85,y+.25};
}
double distance(const Pose2D & p,const Box & b)
{
  const double dx=std::max({b.min_x-p.x,0.,p.x-b.max_x});
  const double dy=std::max({b.min_y-p.y,0.,p.y-b.max_y});
  if (dx==0 && dy==0) {return -std::min({p.x-b.min_x,b.max_x-p.x,p.y-b.min_y,b.max_y-p.y});}
  return std::hypot(dx,dy);
}
double segmentDistance(const Pose2D & a,const Pose2D & b,const Box & box)
{
  const double low[]={box.min_x,box.min_y},high[]={box.max_x,box.max_y};
  const double from[]={a.x,a.y},to[]={b.x,b.y};double enter=0,leave=1;
  for (int k=0;k<2;++k) {
    const double d=to[k]-from[k];
    if (d==0) {if (from[k]<low[k] || from[k]>high[k]) {leave=-1;break;}}
    else {
      double u=(low[k]-from[k])/d,v=(high[k]-from[k])/d;
      if (u>v) {std::swap(u,v);}enter=std::max(enter,u);leave=std::min(leave,v);
    }
  }
  if (enter<=leave) {return 0;}
  double result=std::min(std::max(0.,distance(a,box)),std::max(0.,distance(b,box)));
  const double dx=b.x-a.x,dy=b.y-a.y,norm=dx*dx+dy*dy;
  for (double x : {box.min_x,box.max_x}) {for (double y : {box.min_y,box.max_y}) {
    const double u=norm>0?std::clamp(((x-a.x)*dx+(y-a.y)*dy)/norm,0.,1.):0;
    result=std::min(result,std::hypot(a.x+u*dx-x,a.y+u*dy-y));
  }}
  return result;
}
Exclusion exclusion(double t)
{
  const auto b=actor(t);ObstacleObservation o;
  o.points={{{b.min_x,b.min_y}},{{b.max_x,b.min_y}},{{b.max_x,b.max_y}},{{b.min_x,b.max_y}}};
  o.partial_motion_rank=2;o.partial_motion_vy=.25;
  ScanSnapshot s;s.stamp.sec=1;s.obstacles={o};Exclusion z;z.start(s,0,1,0);return z;
}
bool passed(const Pose2D & p,const Exclusion & z)
{return z.along(p)>z.hi && z.across(p)<z.rear;}
std::unique_ptr<nav2_costmap_2d::Costmap2D> makeMap(const Pose2D & p,const std::vector<Box> & boxes)
{
  auto map=std::make_unique<nav2_costmap_2d::Costmap2D>(60,60,.05,
    std::floor((p.x-1.5)/.05)*.05,std::floor((p.y-1.5)/.05)*.05,0);
  std::vector<std::pair<int,int>> occupied;
  for (int y=0;y<60;++y) {for (int x=0;x<60;++x) {
    double wx,wy;map->mapToWorld(x,y,wx,wy);
    for (const auto & b:boxes) {
      if (wx+.025>=b.min_x && wx-.025<=b.max_x && wy+.025>=b.min_y && wy-.025<=b.max_y) {
        occupied.emplace_back(x,y);break;
      }
    }
  }}
  // Same configured Nav2 inflation law; complete geometry replaces ray marking.
  for (const auto & cell:occupied) {
    for (int y=std::max(0,cell.second-11);y<=std::min(59,cell.second+11);++y) {
      for (int x=std::max(0,cell.first-11);x<=std::min(59,cell.first+11);++x) {
        const double d=.05*std::hypot(x-cell.first,y-cell.second);
        if (d>.55) {continue;}
        const unsigned char cost=d==0?254:d<=.22?253:
          static_cast<unsigned char>(252*std::exp(-3*(d-.22)));
        map->setCost(x,y,std::max(cost,map->getCost(x,y)));
      }
    }
  }
  return map;
}
void Plant::issue(double t,Velocity2D v) {queue.emplace_back(t,v);}
void Plant::advance(double end)
{
  while (time<end-1e-10) {
    const double dt=std::min(.005,end-time),applied=time+dt/2-.12;
    while (queue.size()>1 && queue[1].first<=applied+1e-10) {queue.pop_front();}
    Velocity2D target{};
    if (!queue.empty() && queue.front().first<=applied+1e-10) {target=queue.front().second;}
    const auto approach=[](double a,double b,double d){return a+std::clamp(b-a,-d,d);};
    const Velocity2D next{approach(velocity.v,target.v,dt),approach(velocity.omega,target.omega,3.2*dt)};
    const double v=(velocity.v+next.v)/2,w=(velocity.omega+next.omega)/2;
    pose.x+=v*std::cos(pose.theta+w*dt/2)*dt;pose.y+=v*std::sin(pose.theta+w*dt/2)*dt;
    pose.theta+=w*dt;velocity=next;time+=dt;
  }
}
