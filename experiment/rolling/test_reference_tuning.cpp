#include <gtest/gtest.h>
#include "reference.hpp"
const std::vector<Box> walls{{-2,-2.5,5,-1.5},{-2,1.5,5,2.5}};
void observe(ReferenceGuide & g,double t) {
  ScanSnapshot s;s.stamp=rclcpp::Time(sourceNs(t),RCL_ROS_TIME);
  g.observe(&s,exclusion(t),point(.826,0),walls,sourceNs(t));
}
TEST(ReferenceTuning, DefaultCoordinatesPreserved) {
  ReferenceGuide g;observe(g,6);ASSERT_TRUE(g.proposal);
  EXPECT_DOUBLE_EQ(g.proposal->x,2.22);EXPECT_NEAR(g.proposal->y,-.72,1e-12);
}
TEST(ReferenceTuning, Extra20AndForward20ChangePoint) {
  ReferenceGuide g;g.extra_clearance=.2;g.forward_offset=.2;observe(g,6);
  ASSERT_TRUE(g.proposal);EXPECT_NEAR(g.proposal->x,2.32,1e-12);EXPECT_NEAR(g.proposal->y,-.82,1e-12);
}
TEST(ReferenceTuning, Extra30AndForward30ChangePoint) {
  ReferenceGuide g;g.extra_clearance=.3;g.forward_offset=.3;observe(g,6);
  ASSERT_TRUE(g.proposal);EXPECT_NEAR(g.proposal->x,2.42,1e-12);EXPECT_NEAR(g.proposal->y,-.92,1e-12);
}
TEST(ReferenceTuning, LargerExtraRejectsNarrowerChannel) {
  ReferenceGuide g;g.extra_clearance=g.forward_offset=.2;observe(g,5);EXPECT_FALSE(g.proposal);
  g.extra_clearance=g.forward_offset=.3;observe(g,5.8);EXPECT_FALSE(g.proposal);
}
TEST(ReferenceTuning, HeadingMultiplierDoesNotNormalizeOtherWeights) {
  Trajectory tr{{.1,-.1},{point(.8,0),point(.95,-.02)}};tr.poses.back().theta=-.1;
  std::vector<Pose2D> path{point(.8,0),point(4,0)};
  auto original=TrajectoryScorer(.26).score(tr,path,80,true);
  auto baseline=referenceScore(original,tr,point(.8,0),point(2.22,-.72));
  for (double multiplier:{1.,10.,20.,40.}) {
    auto r=referenceScore(original,tr,point(.8,0),point(2.22,-.72),multiplier);
    EXPECT_DOUBLE_EQ(r.heading_cost,baseline.heading_cost);
    EXPECT_DOUBLE_EQ(r.goal_cost,original.goal_cost);EXPECT_DOUBLE_EQ(r.path_cost,original.path_cost);
    EXPECT_DOUBLE_EQ(r.speed_cost,original.speed_cost);EXPECT_DOUBLE_EQ(r.obstacle_cost,original.obstacle_cost);
    EXPECT_NEAR(r.total_cost,baseline.total_cost+.05*(multiplier-1)*baseline.heading_cost,1e-12);
  }
}
