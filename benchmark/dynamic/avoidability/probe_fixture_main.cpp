#include <cassert>
#include <stdexcept>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
using Pose=geometry_msgs::msg::PoseStamped;using Twist=geometry_msgs::msg::Twist;using Command=geometry_msgs::msg::TwistStamped;
extern "C" int fixture_calls();
extern "C" Command fixture(void*,const Pose&,const Twist&,void*) asm("_ZN17my_dwa_controller15MyDWAController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE");
int main(){Pose p;p.header.stamp.sec=7;p.pose.orientation.w=1;Twist v;auto cmd=fixture(nullptr,p,v,nullptr);assert(cmd.twist.linear.x==.2&&cmd.twist.angular.z==.1&&cmd.header.stamp.sec==7);bool caught=false;try{fixture(nullptr,p,v,reinterpret_cast<void*>(1));}catch(const std::runtime_error&){caught=true;}assert(caught);assert(fixture_calls()==2);}