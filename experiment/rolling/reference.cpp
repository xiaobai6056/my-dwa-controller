#include "reference.hpp"
bool bypassActionAdmitted(int selected_rank,const Velocity2D & command)
{return selected_rank>=0 && (command.v!=0 || command.omega!=0);}
void ReferenceGuide::observe(const ScanSnapshot * s,const Exclusion & z,const Pose2D & robot,
  const std::vector<Box> & walls,int64_t now)
{
  fresh=aborted=released=false;
  const int64_t stamp=s?int64_t(s->stamp.sec)*1000000000+s->stamp.nanosec:0;
  if (!s || !z.valid || stamp<=0 || stamp>now || now-stamp>350000000 || stamp<last_stamp ||
      (side!=0 && side!=int(z.sign))) {abort();return;}
  valid=true;if (stamp==last_stamp) {return;}
  if (last_stamp>0 && stamp-last_stamp>350000000) {
    if (phase==ReferencePhase::Locked) {abort();last_stamp=stamp;return;}
    votes=0;
  }
  fresh=true;last_stamp=stamp;if (side==0) {side=int(z.sign);}
  if (phase!=ReferencePhase::Waiting) {return;}
  proposal.reset();
  const double s_target=z.hi+.10,q_target=z.rear-.10;
  const auto world=[&](double a,double b){return point(a*z.tx-b*z.sign*z.ty,a*z.ty+b*z.sign*z.tx);};
  const auto target_point=world(s_target,q_target),corner=world(z.along(robot),q_target);
  Trajectory certificate{{0,0},{robot,corner,target_point}};
  if (!std::isfinite(target_point.x) || !std::isfinite(target_point.y) || !z.clear(certificate,0)) {votes=0;return;}
  // A connectivity certificate, not a path supplied to the sampler or scorer.
  for (const auto & b:walls) {
    if (segmentDistance(robot,corner,b)<.37-1e-10 || segmentDistance(corner,target_point,b)<.37-1e-10) {votes=0;return;}
  }
  proposal=target_point;
}
bool ReferenceGuide::approve(bool stopped,bool admitted,bool useful,bool fallback_safe)
{
  if (!valid || !fallback_safe || (phase==ReferencePhase::Locked && !admitted)) {abort();return false;}
  if (phase==ReferencePhase::Locked) {fresh=false;return true;}
  if (phase==ReferencePhase::Done) {fresh=false;return false;}
  if (fresh) {
    fresh=false;
    if (!stopped || !admitted || !useful || !proposal) {votes=0;return false;}
    ++votes;
    if (votes>=4) {target=proposal;phase=ReferencePhase::Locked;lock_stamp=last_stamp;return true;}
  }
  return false;
}
bool ReferenceGuide::complete(const Pose2D & p,const Exclusion & z,bool admitted,bool recovers_path)
{
  if (phase!=ReferencePhase::Locked || !valid || !z.valid || !admitted || !recovers_path ||
      z.along(p)<=z.hi || z.across(p)>=z.rear) {return false;}
  phase=ReferencePhase::Done;target.reset();proposal.reset();votes=0;released=true;return true;
}
void ReferenceGuide::abort()
{phase=ReferencePhase::Waiting;target.reset();proposal.reset();votes=0;valid=false;fresh=false;aborted=true;}
void ReferenceGuide::riskReleasedWhileWaiting()
{if (phase==ReferencePhase::Waiting) {phase=ReferencePhase::Done;proposal.reset();votes=0;}}
double ReferenceGuide::desiredYaw(const Pose2D & p) const
{
  const auto ref=phase==ReferencePhase::Locked?target:proposal;
  return ref?std::atan2(ref->y-p.y,ref->x-p.x):p.theta;
}
ScoreResult referenceScore(const ScoreResult & original,const Trajectory & t,const Pose2D & p,const Pose2D & target)
{
  auto s=original;const double desired=std::atan2(target.y-p.y,target.x-p.x),pi=std::acos(-1.);
  s.heading_cost=std::abs(std::remainder(desired-t.poses.back().theta,2*pi))/pi;
  const ScoreWeights w;
  s.total_cost=static_cast<double>(static_cast<long double>(w.obstacle)*s.obstacle_cost+
      static_cast<long double>(w.speed)*s.speed_cost+static_cast<long double>(w.goal)*s.goal_cost+
      static_cast<long double>(w.path)*s.path_cost+static_cast<long double>(w.heading)*s.heading_cost);
  return s;
}
void fillCurrentRays(ScanSnapshot & s,const Pose2D & p,const std::vector<Box> & boxes)
{
  s.laser_pose={p.x,p.y,p.theta};s.angle_min=0;s.angle_increment=2*std::acos(-1.)/360;
  s.range_min=.12;s.range_max=3.5;s.ranges.assign(360,3.5f);
  for (size_t i=0;i<s.ranges.size();++i) {
    const double angle=p.theta+i*s.angle_increment,dir[]={std::cos(angle),std::sin(angle)},origin[]={p.x,p.y};
    for (const auto & b:boxes) {
      const double lo[]={b.min_x,b.min_y},hi[]={b.max_x,b.max_y};double enter=0,leave=3.5;
      for (int k=0;k<2;++k) {
        if (std::abs(dir[k])<1e-15) {if (origin[k]<lo[k] || origin[k]>hi[k]) {leave=-1;break;}}
        else {
          double a=(lo[k]-origin[k])/dir[k],c=(hi[k]-origin[k])/dir[k];if (a>c) {std::swap(a,c);}
          enter=std::max(enter,a);leave=std::min(leave,c);
        }
      }
      if (enter<=leave) {s.ranges[i]=std::min(s.ranges[i],static_cast<float>(enter));}
    }
  }
}
