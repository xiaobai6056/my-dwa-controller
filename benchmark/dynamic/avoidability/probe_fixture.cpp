#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "my_dwa_controller/controller_diagnostics.hpp"
using Pose=geometry_msgs::msg::PoseStamped;using Twist=geometry_msgs::msg::Twist;using Command=geometry_msgs::msg::TwistStamped;
static int calls=0;
extern "C" int fixture_calls(){return calls;}
extern "C" Command fixture(void*,const Pose&,const Twist&,void*) asm("_ZN17my_dwa_controller15MyDWAController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE");
extern "C" Command fixture(void*,const Pose&p,const Twist&,void*check){++calls;namespace m=my_dwa_controller;nav2_costmap_2d::Costmap2D map(20,20,.1,-1,-1,0);m::Trajectory t;t.command={.2,.1};geometry_msgs::msg::Pose2D q;q.x=.1;t.poses.push_back(q);m::ObstacleRejection rejection;m::CycleDiagnostics c;c.sampled=c.processed=1;c.recordSamples({t.command});auto ob=m::evaluateTrajectory(t,map,&rejection);if(!ob.collision_free)throw std::runtime_error("fixture map");c.recordValid(t.command);m::ScoreResult s{.1,.2,.3,.4,.5,.6};c.recordScored(t.command,s,ob,0);c.recordBest(true);m::ControllerDiagnostics diag;diag.report(rclcpp::get_logger("fixture"),c,nullptr,false);if(check)throw std::runtime_error("expected forwarding exception");Command cmd;cmd.header=p.header;cmd.twist.linear.x=.2;cmd.twist.angular.z=.1;return cmd;}