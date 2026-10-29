#include <gtest/gtest.h>
#include "strategy.hpp"
using namespace my_dwa_controller;
TEST(Weights, ExactUnnormalizedTable) {
  const double expected[7][5]={{.05,.2,.6,.1,.1},{0,.2,.6,.1,.1},{0,0,.6,.1,.1},
    {0,0,.6,.2,.1},{0,0,.6,.4,.1},{0,0,.3,.2,.1},{0,0,0,.2,.1}};
  for (int i=0;i<7;++i) {
    const auto w=strategy(std::string(1,char('A'+i)));
    EXPECT_DOUBLE_EQ(w.heading,expected[i][0]);EXPECT_DOUBLE_EQ(w.path,expected[i][1]);
    EXPECT_DOUBLE_EQ(w.goal,expected[i][2]);EXPECT_DOUBLE_EQ(w.speed,expected[i][3]);EXPECT_DOUBLE_EQ(w.obstacle,expected[i][4]);
  }
}
Pose2D p(double x,double y,double angle=0) {Pose2D q;q.x=x;q.y=y;q.theta=angle;return q;}
TEST(Weights, LinearSumWithoutRenormalizing) {
  const Trajectory t{{.13,-.2},{p(0,0),p(.2,-.1,-.3)}};
  const std::vector<Pose2D> path{p(0,0),p(4,0)};
  const auto a=TrajectoryScorer(.26).score(t,path,126,true);
  for (char c='A';c<='G';++c) {
    const auto w=strategy(std::string(1,c));const auto s=TrajectoryScorer(.26,w).score(t,path,126,true);
    const double expected=w.heading*a.heading_cost+w.path*a.path_cost+w.goal*a.goal_cost+w.speed*a.speed_cost+w.obstacle*a.obstacle_cost;
    EXPECT_NEAR(s.total_cost,expected,1e-15);
  }
  const auto d=TrajectoryScorer(.26,strategy("D")).score(t,path,126,true);
  EXPECT_NEAR(d.total_cost,.6*a.goal_cost+.2*a.speed_cost+.1*a.obstacle_cost,1e-15);
}
TEST(Weights, HeadingFlagAndZeroWeightAreDistinct) {
  const Trajectory t{{.13,-.2},{p(0,0),p(.2,-.1,-.3)}};
  const std::vector<Pose2D> path{p(0,0),p(4,0)};
  const auto on=TrajectoryScorer(.26).score(t,path,126,true),off=TrajectoryScorer(.26).score(t,path,126,false);
  EXPECT_GT(on.heading_cost,0);EXPECT_EQ(off.heading_cost,0);
  const auto b=TrajectoryScorer(.26,strategy("B")).score(t,path,126,true);
  EXPECT_DOUBLE_EQ(b.heading_cost,on.heading_cost);
  EXPECT_NEAR(on.total_cost-b.total_cost,.05*on.heading_cost,1e-15);
}
