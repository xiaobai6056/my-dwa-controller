#include <gtest/gtest.h>
#include "reference.hpp"
#include "my_dwa_controller/path_geometry.hpp"
const std::vector<Box> walls{{-2,-2.5,5,-1.5},{-2,1.5,5,2.5}};
void observe(ReferenceGuide & g,double t,const std::vector<Box>& geometry=walls) {
 ScanSnapshot s;s.stamp=rclcpp::Time(sourceNs(t),RCL_ROS_TIME);
 g.observe(&s,exclusion(t),point(.826,0),geometry,sourceNs(t));
}
TEST(Polyline, CoordinatesAndFullSegments) {
 ReferenceGuide g;g.polyline=true;observe(g,6);auto line=g.line();ASSERT_EQ(line.size(),3u);
 EXPECT_NEAR(line[1].x,.826,1e-12);EXPECT_NEAR(line[1].y,-.72,1e-12);
 EXPECT_NEAR(line[2].x,2.22,1e-12);EXPECT_NEAR(line[2].y,-.72,1e-12);
 EXPECT_TRUE(exclusion(6).clear(Trajectory{{0,0},line},0));
}
TEST(Polyline, ConnectionWallRejectsEvenWhenEndpointsClear) {
 ReferenceGuide g;g.polyline=true;auto w=walls;w.push_back({.75,-.55,.90,-.50});
 EXPECT_GT(segmentDistance(point(.826,0),point(.826,0),w.back()),.37);
 EXPECT_GT(segmentDistance(point(.826,-1.07),point(.826,-1.07),w.back()),.37);
 observe(g,4.6,w);EXPECT_FALSE(g.proposal);EXPECT_TRUE(g.line().empty());
}
TEST(Polyline, NarrowAndLargerMargin) {
 ReferenceGuide g;g.polyline=true;g.extra_clearance=g.forward_offset=.2;observe(g,5);EXPECT_FALSE(g.proposal);
 observe(g,6);ASSERT_EQ(g.line().size(),3u);EXPECT_NEAR(g.line()[2].x,2.32,1e-12);EXPECT_NEAR(g.line()[1].y,-.82,1e-12);
}
TEST(Polyline, FourFreshThenFixedLine) {
 ReferenceGuide g;g.polyline=true;observe(g,6);g.approve(true,true,true,true);observe(g,6);g.approve(true,true,true,true);EXPECT_EQ(g.votes,1);
 for(double t:{6.2,6.4,6.6}){observe(g,t);g.approve(true,true,true,true);}
 ASSERT_EQ(g.phase,ReferencePhase::Locked);auto fixed=g.line();ASSERT_EQ(fixed.size(),3u);
 observe(g,6.8);EXPECT_DOUBLE_EQ(g.line()[1].y,fixed[1].y);EXPECT_DOUBLE_EQ(g.line()[2].y,fixed[2].y);
}
TEST(Polyline, LostEvidenceClearsWholeLine) {
 ReferenceGuide g;g.polyline=true;for(double t:{6.,6.2,6.4,6.6}){observe(g,t);g.approve(true,true,true,true);}
 g.observe(nullptr,exclusion(6.8),point(.826,0),walls,sourceNs(6.8));EXPECT_TRUE(g.line().empty());EXPECT_FALSE(g.approve(true,true,true,true));
}
TEST(Polyline, NativeScorerTerminalTangentDoesNotPointBack) {
 ReferenceGuide g;g.polyline=true;observe(g,6);auto line=g.line();ASSERT_EQ(line.size(),3u);
 Trajectory tr{{.1,0},{point(2.23,-.72),point(2.4,-.72)}};
 auto result=TrajectoryScorer(.26).score(tr,line,0,true);EXPECT_NEAR(result.heading_cost,0,1e-12);
}
TEST(Polyline, ReliableClearStillRequiresSafeRecovery) {
 ReferenceGuide g;g.polyline=true;for(double t:{6.,6.2,6.4,6.6}){observe(g,t);g.approve(true,true,true,true);}
 EXPECT_FALSE(g.complete(point(1,0),exclusion(8.8),true,false,true));
 EXPECT_TRUE(g.complete(point(1,0),exclusion(8.8),true,true,true));EXPECT_TRUE(g.line().empty());
}
TEST(Polyline, ReversalAndBackupLossStop) {
 for(bool reverse:{false,true}) {
  ReferenceGuide g;g.polyline=true;for(double t:{6.,6.2,6.4,6.6}){observe(g,t);g.approve(true,true,true,true);}
  if(reverse){ScanSnapshot s;s.stamp=rclcpp::Time(sourceNs(6.8),RCL_ROS_TIME);auto z=exclusion(6.8);z.sign=-1;g.observe(&s,z,point(.826,0),walls,sourceNs(6.8));}
  else{g.approve(false,true,true,false);}
  EXPECT_TRUE(g.line().empty());EXPECT_FALSE(g.approve(true,true,true,true));
 }
}
TEST(Polyline, ActualPassReleasesWholeLine) {
 ReferenceGuide g;g.polyline=true;for(double t:{6.,6.2,6.4,6.6}){observe(g,t);g.approve(true,true,true,true);}
 EXPECT_TRUE(g.complete(point(2.3,-.72),exclusion(8.8),true,true));EXPECT_TRUE(g.line().empty());
}
TEST(Polyline, NativeScoreUsesSamePathGoalHeadingReference) {
 ReferenceGuide g;g.polyline=true;observe(g,6);auto line=g.line();ASSERT_EQ(line.size(),3u);
 Trajectory t{{.1,-.1},{point(.826,0),point(.85,-.1)}};
 auto base=TrajectoryScorer(.26).score(t,line,80,true);ScoreWeights w;w.heading*=8;
 auto high=TrajectoryScorer(.26,w).score(t,line,80,true);
 EXPECT_DOUBLE_EQ(base.path_cost,high.path_cost);EXPECT_DOUBLE_EQ(base.goal_cost,high.goal_cost);
 EXPECT_DOUBLE_EQ(base.speed_cost,high.speed_cost);EXPECT_DOUBLE_EQ(base.obstacle_cost,high.obstacle_cost);
 EXPECT_NEAR(high.total_cost-base.total_cost,.35*base.heading_cost,1e-12);
}
