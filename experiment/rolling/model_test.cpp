#include <gtest/gtest.h>
#include "rolling_model.hpp"
TEST(Scene, FloatingControlTimesDoNotProduceFutureScan) {
  EXPECT_EQ(sourceNs(4.2-1e-12),sourceNs(4.2));
  EXPECT_EQ(sourceNs(7.4-1e-12),sourceNs(7.4));
}
TEST(Scene, OriginalT2Timing) {
  EXPECT_NEAR(actor(2).min_y,-1.35,1e-12);
  EXPECT_NEAR(actor(6).min_y,-.35,1e-12);
  EXPECT_NEAR(actor(10.8).min_y,.85,1e-12);
}
TEST(Geometry, CircleBoxClearanceAndCorner) {
  EXPECT_NEAR(distance(point(1,0),{1.35,-.25,1.85,.25})-.22,.13,1e-12);
  EXPECT_NEAR(distance(point(0,0),{1,1,2,2}),std::sqrt(2.),1e-12);
  EXPECT_LT(distance(point(1.5,0),{1.35,-.25,1.85,.25}),0);
}
TEST(Geometry, SegmentCannotJumpThroughBox) {
  EXPECT_EQ(segmentDistance(point(0,0),point(3,0),{1,-.2,2,.2}),0);
  EXPECT_NEAR(segmentDistance(point(0,-1),point(3,-1),{1,0,2,1}),1,1e-12);
}
TEST(Execution, DelayPersistsAcrossCycles) {
  Plant p;p.issue(0,{.26,0});p.advance(.05);EXPECT_EQ(p.velocity.v,0);
  p.issue(.05,{.26,0});p.advance(.10);EXPECT_EQ(p.velocity.v,0);
  p.issue(.10,{.26,0});p.advance(.15);EXPECT_GT(p.velocity.v,.02);EXPECT_GT(p.pose.x,0);
}
TEST(Execution, AccelerationIsLimited) {
  Plant p;p.issue(0,{.26,1});p.advance(.17);
  EXPECT_NEAR(p.velocity.v,.05,.006);EXPECT_NEAR(p.velocity.omega,.16,.017);
}
TEST(Exclusion, FrontDeniedRearAccepted) {
  auto z=exclusion(6);Trajectory front{{.26,0},{point(0,.5),point(3,.5)}};
  Trajectory rear{{.26,0},{point(0,-1),point(3,-1)}};
  EXPECT_FALSE(z.clear(front,.013));EXPECT_TRUE(z.clear(rear,.013));
}
TEST(Success, ApproachAndSpinAreNotPassage) {
  EXPECT_FALSE(passed(point(1,0),exclusion(8.5)));
  EXPECT_FALSE(passed(point(1.6,0),exclusion(8.5)));
  EXPECT_FALSE(passed(point(2.2,.3),exclusion(8.5)));
  EXPECT_TRUE(passed(point(2.2,0),exclusion(8.5)));
}
TEST(Costmap, OriginalInflationAndResolution) {
  auto map=makeMap(point(1,0),{{1.35,-.25,1.85,.25}});
  EXPECT_DOUBLE_EQ(map->getResolution(),.05);EXPECT_EQ(map->getSizeInCellsX(),60u);
  unsigned int x,y;ASSERT_TRUE(map->worldToMap(1.6,0,x,y));EXPECT_EQ(map->getCost(x,y),254);
  ASSERT_TRUE(map->worldToMap(1.1,0,x,y));EXPECT_GT(map->getCost(x,y),0);
}
