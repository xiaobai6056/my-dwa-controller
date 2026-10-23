// Observation-only ABI interposition: original methods called once; no return/argument edits.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "my_dwa_controller/controller_diagnostics.hpp"
namespace m=my_dwa_controller;
using Pose=geometry_msgs::msg::PoseStamped;using Twist=geometry_msgs::msg::Twist;using Command=geometry_msgs::msg::TwistStamped;
using Compute=Command(*)(void*,const Pose&,const Twist&,void*);
constexpr const char* MY="_ZN17my_dwa_controller15MyDWAController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE";
constexpr const char* EV="_ZN17my_dwa_controller18evaluateTrajectoryERKNS_10TrajectoryERKN15nav2_costmap_2d9Costmap2DEPNS_17ObstacleRejectionE";
constexpr const char* SC="_ZN17my_dwa_controller16CycleDiagnostics12recordScoredERKNS_10Velocity2DERKNS_11ScoreResultERKNS_18ObstacleEvaluationEd";
constexpr const char* BE="_ZN17my_dwa_controller16CycleDiagnostics10recordBestEb";
constexpr const char* RP="_ZN17my_dwa_controller21ControllerDiagnostics6reportERKN6rclcpp6LoggerERKNS_16CycleDiagnosticsEPKcb";
void* original(const char* sym){const char* p=std::getenv("NAVPERF_MYDWA_LIB");void*h=dlopen(p,RTLD_LAZY|RTLD_NOLOAD);void*f=h?dlsym(h,sym):nullptr;if(!f){std::fprintf(stderr,"decision probe missing %s\n",sym);std::abort();}return f;}
double ms(){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Candidate{double v,w;int reject,free,max;double mean;bool scored=false;m::ScoreResult score{};m::Pose2D end;};
struct Cycle{std::vector<Candidate> candidates;std::vector<unsigned char> map;unsigned nx=0,ny=0;double res=0,ox=0,oy=0;std::vector<m::Pose2D> last,best;int best_index=-1;std::string diag;};
thread_local Cycle* current=nullptr;
std::ofstream* stream=nullptr;
std::string poses(const std::vector<m::Pose2D>&v){std::ostringstream s;s<<std::setprecision(12)<<'[';bool first=true;for(auto&p:v){if(!first)s<<',';first=false;s<<'['<<p.x<<','<<p.y<<','<<p.theta<<']';}s<<']';return s.str();}
void save(Cycle&c,const Pose&p,const Twist&v,const Command&cmd,bool threw,double duration){
  if(!stream){const char*dir=std::getenv("DECISION_DIR");if(!dir)return;stream=new std::ofstream(std::string(dir)+"/decisions_"+std::to_string(getpid())+".jsonl");}
  double start=ms();std::ostringstream s;s<<std::setprecision(12);s<<"{\"stamp\":"<<cmd.header.stamp.sec+cmd.header.stamp.nanosec*1e-9<<",\"input_stamp\":"<<p.header.stamp.sec+p.header.stamp.nanosec*1e-9<<",\"threw\":"<<(threw?"true":"false")<<",\"compute_with_observation_ms\":"<<duration<<",\"input\":["<<v.linear.x<<','<<v.angular.z<<"],\"command\":["<<cmd.twist.linear.x<<','<<cmd.twist.angular.z<<"],\"pose\":["<<p.pose.position.x<<','<<p.pose.position.y<<','<<2*std::atan2(p.pose.orientation.z,p.pose.orientation.w)<<"],\"best_index\":"<<c.best_index<<",\"rollout_initial_pose\":"<<poses(c.last.empty()?std::vector<m::Pose2D>{}:std::vector<m::Pose2D>{c.last.front()})<<",\"best_poses\":"<<poses(c.best)<<",\"diagnostic\":"<<(c.diag.empty()?"null":c.diag)<<",\"costmap\":{\"width\":"<<c.nx<<",\"height\":"<<c.ny<<",\"resolution\":"<<c.res<<",\"origin\":["<<c.ox<<','<<c.oy<<"],\"data\":[";
  for(size_t i=0;i<c.map.size();++i){if(i)s<<',';s<<unsigned(c.map[i]);}s<<"]},\"candidates\":[";
  bool first=true;for(auto&z:c.candidates){if(!first)s<<',';first=false;s<<"{\"v\":"<<z.v<<",\"w\":"<<z.w<<",\"free\":"<<(z.free?"true":"false")<<",\"rejection\":"<<z.reject<<",\"max\":"<<z.max<<",\"mean\":"<<z.mean<<",\"end\":["<<z.end.x<<','<<z.end.y<<','<<z.end.theta<<"],\"scores\":";if(!z.scored)s<<"null";else{s<<'['<<z.score.obstacle_cost<<','<<z.score.speed_cost<<','<<z.score.goal_cost<<','<<z.score.path_cost<<','<<z.score.heading_cost<<','<<z.score.total_cost<<']';}s<<'}';}s<<"],\"serialize_ms\":"<<ms()-start<<"}\n";(*stream)<<s.str();stream->flush();
}
extern "C" Command compute(void*,const Pose&,const Twist&,void*) asm("_ZN17my_dwa_controller15MyDWAController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE");
extern "C" Command compute(void*self,const Pose&p,const Twist&v,void*g){static auto f=reinterpret_cast<Compute>(original(MY));Cycle c;c.candidates.reserve(1024);Cycle*prev=current;current=&c;double start=ms();try{Command out=f(self,p,v,g);double duration=ms()-start;current=prev;save(c,p,v,out,false,duration);return out;}catch(...){double duration=ms()-start;current=prev;save(c,p,v,Command{},true,duration);throw;}}
using Evaluate=m::ObstacleEvaluation(*)(const m::Trajectory&,const nav2_costmap_2d::Costmap2D&,m::ObstacleRejection*);
extern "C" m::ObstacleEvaluation evaluate(const m::Trajectory&,const nav2_costmap_2d::Costmap2D&,m::ObstacleRejection*) asm("_ZN17my_dwa_controller18evaluateTrajectoryERKNS_10TrajectoryERKN15nav2_costmap_2d9Costmap2DEPNS_17ObstacleRejectionE");
extern "C" m::ObstacleEvaluation evaluate(const m::Trajectory&t,const nav2_costmap_2d::Costmap2D&map,m::ObstacleRejection*r){static auto f=reinterpret_cast<Evaluate>(original(EV));if(current&&current->map.empty()){auto&c=*current;c.nx=map.getSizeInCellsX();c.ny=map.getSizeInCellsY();c.res=map.getResolution();c.ox=map.getOriginX();c.oy=map.getOriginY();c.map.assign(map.getCharMap(),map.getCharMap()+c.nx*c.ny);}auto out=f(t,map,r);if(current){Candidate z{};z.v=t.command.v;z.w=t.command.omega;z.free=out.collision_free;z.max=out.max_cost;z.mean=out.mean_cost;z.reject=r?int(*r):-1;if(!t.poses.empty())z.end=t.poses.back();current->candidates.push_back(z);current->last=t.poses;}return out;}
using Scored=void(*)(void*,const m::Velocity2D&,const m::ScoreResult&,const m::ObstacleEvaluation&,double);
extern "C" void scored(void*,const m::Velocity2D&,const m::ScoreResult&,const m::ObstacleEvaluation&,double) asm("_ZN17my_dwa_controller16CycleDiagnostics12recordScoredERKNS_10Velocity2DERKNS_11ScoreResultERKNS_18ObstacleEvaluationEd");
extern "C" void scored(void*self,const m::Velocity2D&v,const m::ScoreResult&s,const m::ObstacleEvaluation&o,double r){static auto f=reinterpret_cast<Scored>(original(SC));f(self,v,s,o,r);if(current&&!current->candidates.empty()){auto&z=current->candidates.back();z.scored=true;z.score=s;}}
using Best=void(*)(void*,bool);
extern "C" void best(void*,bool) asm("_ZN17my_dwa_controller16CycleDiagnostics10recordBestEb");
extern "C" void best(void*self,bool better){static auto f=reinterpret_cast<Best>(original(BE));f(self,better);if(current){current->best_index=int(current->candidates.size())-1;current->best=current->last;}}
using Report=void(*)(void*,const rclcpp::Logger&,const m::CycleDiagnostics&,const char*,bool);
extern "C" void report(void*,const rclcpp::Logger&,const m::CycleDiagnostics&,const char*,bool) asm("_ZN17my_dwa_controller21ControllerDiagnostics6reportERKN6rclcpp6LoggerERKNS_16CycleDiagnosticsEPKcb");
extern "C" void report(void*self,const rclcpp::Logger&l,const m::CycleDiagnostics&c,const char*e,bool failed){static auto f=reinterpret_cast<Report>(original(RP));if(current){std::ostringstream s;s<<std::setprecision(12)<<"{\"goal_phase\":\""<<c.goal_phase<<"\",\"weights\":["<<c.settings.weights.obstacle<<','<<c.settings.weights.speed<<','<<c.settings.weights.goal<<','<<c.settings.weights.path<<','<<c.settings.weights.heading<<"],\"sampled\":"<<c.sampled<<",\"processed\":"<<c.processed<<",\"valid\":"<<c.valid<<",\"scored\":"<<c.scored<<",\"vmin\":"<<(std::isfinite(c.v_min)?c.v_min:0.)<<",\"vmax\":"<<(std::isfinite(c.v_max)?c.v_max:0.)<<",\"score_ms\":"<<c.score_ms<<",\"failed\":"<<(failed?"true":"false")<<",\"rejected\":[";for(size_t i=0;i<c.rejected.size();++i){if(i)s<<',';s<<c.rejected[i];}s<<"]}";current->diag=s.str();}f(self,l,c,e,failed);}