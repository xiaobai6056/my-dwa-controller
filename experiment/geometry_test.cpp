#include <gtest/gtest.h>
#include "probe_geometry.hpp"
ScanSnapshot scan(int ns = 0)
{
  ScanSnapshot s;s.stamp.sec = 1;s.stamp.nanosec = ns;
  ObstacleObservation o;o.points = {{{1.35, -.8}}, {{1.85, -.4}}};
  o.partial_motion_rank = 1;o.partial_motion_vy = .2;s.obstacles = {o};return s;
}
Exclusion zone() {Exclusion z;z.start(scan(), 0, 1, 0);return z;}
Trajectory line(double y) {return {{.26, 0}, {point(0,y), point(3,y)}};}
TEST(Exclusion, FrontTipCannotBeRounded) {EXPECT_FALSE(zone().clear(line(100), 0));}
TEST(Exclusion, FrontBeyondCurrentEnvelopeStillRejected) {EXPECT_FALSE(zone().clear(line(.1), .013));}
TEST(Exclusion, RearHasStrictClearance) {
  EXPECT_TRUE(zone().clear(line(-1.10), .013));
  EXPECT_FALSE(zone().clear(line(-1.07), 0));
  EXPECT_FALSE(zone().clear(line(-1.08), .013));
}
TEST(Exclusion, ContinuousSegmentsCannotJumpAcross) {EXPECT_FALSE(zone().clear(line(0), 0));}
TEST(Exclusion, NominalClearDoesNotCertifyBackup) {
  EXPECT_TRUE(zone().clear(line(-1.2), 0));EXPECT_FALSE(zone().clear(line(0), 0));
}
TEST(Exclusion, UnknownVelocityRetainsAssociatedDirection) {
  auto z = zone();auto s = scan(200000000);s.previous_stamp_ns = 1000000000;
  s.obstacles[0].previous_cluster_index = 0;s.obstacles[0].partial_motion_rank = 0;
  z.observe(s,1200000000);EXPECT_TRUE(z.valid);EXPECT_EQ(z.sign, 1);
}
TEST(Exclusion, ReversalClosesUntilNewRisk) {
  auto z = zone();auto s = scan(200000000);s.previous_stamp_ns = 1000000000;
  s.obstacles[0].previous_cluster_index = 0;s.obstacles[0].partial_motion_vy = -.2;
  z.observe(s,1200000000);EXPECT_FALSE(z.valid);EXPECT_FALSE(z.clear(line(-2),0));
}
TEST(Exclusion, LostAssociationCloses) {
  auto z = zone();z.observe(scan(200000000),1200000000);EXPECT_FALSE(z.valid);
}
TEST(Exclusion, RepeatedSourceDoesNotUpdateEnvelope) {
  auto z = zone();auto s = scan();s.obstacles[0].points[0][1] = -10;
  z.observe(s,1000000000);EXPECT_DOUBLE_EQ(z.rear,-1.07);
}
TEST(Exclusion, StaleAndFutureClose) {
  auto z = zone();z.observe(scan(),1400000000);EXPECT_FALSE(z.valid);
  z = zone();z.observe(scan(200000000),1100000000);EXPECT_FALSE(z.valid);
}
TEST(Exclusion, OppositeDirectionMirrorsGeometry) {
  auto s = scan();s.obstacles[0].partial_motion_vy = -.2;
  for (auto & p : s.obstacles[0].points) {p[1] *= -1;}
  Exclusion z;z.start(s,0,1,0);EXPECT_TRUE(z.clear(line(1.2),0));EXPECT_FALSE(z.clear(line(-100),0));
}
TEST(Exclusion, InvalidDirectionCannotReviveFromGuessedLocation) {
  auto z = zone();z.observe(scan(200000000),1200000000);
  auto s = scan(400000000);s.previous_stamp_ns = 1200000000;
  s.obstacles[0].previous_cluster_index = 0;
  z.observe(s,1400000000);EXPECT_FALSE(z.valid);
}
TEST(Exclusion, CurrentEnvelopeMovesWithAssociatedObservation) {
  auto z = zone();auto s = scan(200000000);s.previous_stamp_ns = 1000000000;
  s.obstacles[0].previous_cluster_index = 0;s.obstacles[0].partial_motion_rank = 0;
  for (auto & p : s.obstacles[0].points) {p[1] += .1;}
  z.observe(s,1200000000);EXPECT_NEAR(z.rear,-.97,1e-12);
}
TEST(Exclusion, DuplicateAssociationFailsClosed) {
  auto z = zone();auto s = scan(200000000);s.previous_stamp_ns = 1000000000;
  s.obstacles[0].previous_cluster_index = 0;s.obstacles.push_back(s.obstacles[0]);
  z.observe(s,1200000000);EXPECT_FALSE(z.valid);
}
