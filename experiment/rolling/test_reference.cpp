#include <gtest/gtest.h>
#include "reference.hpp"
#include "my_dwa_controller/crossing_yield.hpp"
using namespace my_dwa_controller;
ScanSnapshot scan(double t)
{
  ScanSnapshot s;s.stamp=rclcpp::Time(sourceNs(t),RCL_ROS_TIME);s.frame_id="scene";return s;
}
const std::vector<Box> walls{{-2,-2.5,5,-1.5},{-2,1.5,5,2.5}};
void observe(ReferenceGuide & g,double t,double zone_time=-1)
{
  auto s=scan(t);g.observe(&s,exclusion(zone_time<0?t:zone_time),point(.826,0),walls,sourceNs(t));
}
void locked(ReferenceGuide & g) {g.phase=ReferencePhase::Locked;g.target=point(2.22,-.72);g.side=1;g.valid=true;}
TEST(Reference, SelectedStopRelatchesAndNeedsFreshAdmission) {
  EXPECT_TRUE(bypassActionAdmitted(0,{.1,0}));EXPECT_TRUE(bypassActionAdmitted(0,{0,-.1}));
  EXPECT_FALSE(bypassActionAdmitted(-1,{.1,0}));EXPECT_FALSE(bypassActionAdmitted(0,{}));
  ReferenceGuide g;locked(g);EXPECT_FALSE(g.approve(false,bypassActionAdmitted(0,{}),false,true));
  EXPECT_EQ(g.phase,ReferencePhase::Waiting);EXPECT_FALSE(g.target);EXPECT_EQ(g.votes,0);
}
TEST(Reference, TargetHasExtraClearanceBeyondExit) {
  ReferenceGuide g;observe(g,6);ASSERT_TRUE(g.proposal);EXPECT_NEAR(g.proposal->x,2.22,1e-12);
  EXPECT_NEAR(g.proposal->y,-.72,1e-12);EXPECT_GE(exclusion(6).rear-g.proposal->y,.10-1e-12);
}
TEST(Reference, NarrowChannelAndWallsReject) {
  ReferenceGuide g;observe(g,4);EXPECT_FALSE(g.proposal);
  auto s=scan(6);auto blocked=walls;blocked.push_back({1,-1,2.5,-.5});
  g.observe(&s,exclusion(6),point(.826,0),blocked,sourceNs(6));EXPECT_FALSE(g.proposal);
}
TEST(Reference, RequiresFourDifferentSourceStamps) {
  ReferenceGuide g;observe(g,6);EXPECT_FALSE(g.approve(true,true,true,true));EXPECT_EQ(g.votes,1);
  observe(g,6);EXPECT_FALSE(g.approve(true,true,true,true));EXPECT_EQ(g.votes,1);
  for (double t:{6.2,6.4}) {observe(g,t);EXPECT_FALSE(g.approve(true,true,true,true));}
  observe(g,6.6);EXPECT_TRUE(g.approve(true,true,true,true));EXPECT_EQ(g.phase,ReferencePhase::Locked);
}
TEST(Reference, FailureAndNotStoppedResetVotes) {
  ReferenceGuide g;observe(g,6);g.approve(true,true,true,true);observe(g,6.2);
  EXPECT_FALSE(g.approve(true,true,false,true));EXPECT_EQ(g.votes,0);
  observe(g,6.4);EXPECT_FALSE(g.approve(false,true,true,true));EXPECT_EQ(g.votes,0);
}
TEST(Reference, ProposalUpdatesOnlyOnFreshSourceThenLockDoesNotDrift) {
  ReferenceGuide g;observe(g,6);ASSERT_TRUE(g.proposal);const double y=g.proposal->y;g.approve(true,true,true,true);
  observe(g,6,7);ASSERT_TRUE(g.proposal);EXPECT_EQ(g.proposal->y,y);
  for (double t:{6.0,6.2,6.4,6.6}) {observe(g,t);g.approve(true,true,true,true);}
  ASSERT_TRUE(g.target);const auto fixed=*g.target;observe(g,6.8);ASSERT_TRUE(g.target);
  EXPECT_DOUBLE_EQ(g.target->x,fixed.x);EXPECT_DOUBLE_EQ(g.target->y,fixed.y);
}
TEST(Reference, DesiredYawChangesWithRobotPositionOnly) {
  ReferenceGuide g;locked(g);EXPECT_NE(g.desiredYaw(point(.8,0)),g.desiredYaw(point(1.2,-.2)));
  EXPECT_NEAR(g.desiredYaw(point(.8,0)),std::atan2(-.72,1.42),1e-12);
}
TEST(Reference, ReverseOrAssociationLossStops) {
  for (bool reverse:{false,true}) {
    ReferenceGuide g;locked(g);auto s=scan(6);auto z=exclusion(6);
    if (reverse) {z.sign=-1;}else {z.valid=false;}
    g.observe(&s,z,point(.8,0),walls,sourceNs(6));EXPECT_NE(g.phase,ReferencePhase::Locked);
    EXPECT_FALSE(g.approve(true,true,true,true));EXPECT_FALSE(g.target);
  }
}
TEST(Reference, StaleFutureAndOutOfOrderStop) {
  for (int64_t now:{sourceNs(5),sourceNs(6.4)}) {
    ReferenceGuide g;locked(g);auto s=scan(6);g.observe(&s,exclusion(6),point(.8,0),walls,now);
    EXPECT_FALSE(g.target);EXPECT_FALSE(g.approve(true,true,true,true));
  }
  ReferenceGuide g;locked(g);g.last_stamp=sourceNs(6.2);auto s=scan(6);
  g.observe(&s,exclusion(6),point(.8,0),walls,sourceNs(6.2));EXPECT_FALSE(g.target);
}
TEST(Reference, ActiveAdmissionLossStops) {
  ReferenceGuide g;locked(g);EXPECT_FALSE(g.approve(false,false,false,true));EXPECT_FALSE(g.target);
  locked(g);EXPECT_FALSE(g.approve(false,true,true,false));EXPECT_FALSE(g.target);
}
TEST(Reference, ReleaseNeedsActualPassAndSafePathRecovery) {
  ReferenceGuide g;locked(g);auto z=exclusion(8.8);
  EXPECT_FALSE(g.complete(point(2,0),z,true,true));EXPECT_FALSE(g.complete(point(2.3,.2),z,true,true));
  EXPECT_FALSE(g.complete(point(2.3,0),z,false,true));EXPECT_FALSE(g.complete(point(2.3,0),z,true,false));
  EXPECT_TRUE(g.complete(point(2.3,0),z,true,true));EXPECT_FALSE(g.target);EXPECT_EQ(g.phase,ReferencePhase::Done);
}
TEST(Reference, RiskReleaseDoesNotUnlockActiveReference) {
  ReferenceGuide g;locked(g);g.riskReleasedWhileWaiting();EXPECT_EQ(g.phase,ReferencePhase::Locked);EXPECT_TRUE(g.target);
}
TEST(Reference, OnlyHeadingTermChanges) {
  Trajectory tr{{.1,-.1},{point(.8,0),point(.95,-.02)}};tr.poses.back().theta=-.1;
  const std::vector<Pose2D> path{point(.8,0),point(4,0)};auto original=TrajectoryScorer(.26).score(tr,path,80,true);
  auto result=referenceScore(original,tr,point(.8,0),point(2.22,-.72));
  EXPECT_DOUBLE_EQ(result.goal_cost,original.goal_cost);EXPECT_DOUBLE_EQ(result.path_cost,original.path_cost);
  EXPECT_DOUBLE_EQ(result.speed_cost,original.speed_cost);EXPECT_DOUBLE_EQ(result.obstacle_cost,original.obstacle_cost);
  EXPECT_NEAR(result.heading_cost,std::abs(std::atan2(-.72,1.42)+.1)/std::acos(-1.),1e-12);
  EXPECT_NEAR(result.total_cost,.6*original.goal_cost+.2*original.path_cost+.1*original.speed_cost+.1*original.obstacle_cost+.05*result.heading_cost,1e-12);
}
TEST(Reference, CurrentRaysSeeBoxAndOriginalRiskReleases) {
  auto s=scan(4);const auto robot=point(.826,0);auto boxes=walls;boxes.push_back({1.35,-.25,1.85,.25});
  fillCurrentRays(s,robot,boxes);ASSERT_EQ(s.ranges.size(),360u);EXPECT_LT(s.ranges[0],3.5);
  boxes=walls;boxes.push_back(actor(4));fillCurrentRays(s,robot,boxes);
  const auto b0=actor(4);ObstacleObservation initial;initial.points={{{b0.min_x,b0.min_y}},{{b0.max_x,b0.max_y}}};
  initial.partial_motion_rank=2;initial.partial_motion_vy=.25;s.obstacles={initial};
  CrossingYield policy;Trajectory brake{{0,0},{robot}};std::vector<Pose2D> path{point(.826,0),point(2.826,0)};
  // Actual D01 ray at angle zero is clear until the actor enters this corridor.
  auto first=policy.update(&s,path,robot,{},brake,sourceNs(4),true);ASSERT_TRUE(first.hold);policy.latchHold(true);
  bool released=false;double time=-1;
  for (int i=1;i<=30;++i) {
    double t=4+i*.2;s=scan(t);ObstacleObservation o;const auto b=actor(t);
    o.points={{{b.min_x,b.min_y}},{{b.max_x,b.max_y}}};o.previous_cluster_index=0;s.obstacles={o};
    s.previous_stamp_ns=sourceNs(t-.2);boxes=walls;boxes.push_back(b);fillCurrentRays(s,robot,boxes);
    auto y=policy.update(&s,path,robot,{},brake,sourceNs(t),true);
    if (y.released) {released=true;time=t;break;}
  }
  EXPECT_TRUE(released);EXPECT_GE(time,8.6);EXPECT_LE(time,9.0);
}
