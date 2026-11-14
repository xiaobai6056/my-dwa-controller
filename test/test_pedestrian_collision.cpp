#include <gtest/gtest.h>

#include <algorithm>

#include "my_dwa_controller/pedestrian_collision.hpp"

namespace my_dwa_controller
{
namespace
{
using Array = msg::PedestrianStateArray;
constexpr int64_t S = 1000000000;
const std::vector<geometry_msgs::msg::Point> footprint = [] {
    std::vector<geometry_msgs::msg::Point> result;
    for (const auto & xy : {std::pair{-.2, -.1}, {.2, -.1}, {.2, .1}, {-.2, .1}}) {
      geometry_msgs::msg::Point p;
      p.x = xy.first;
      p.y = xy.second;
      result.push_back(p);
    }
    return result;
  }();

Pose2D pose(double x, double y, double theta = 0.)
{
  Pose2D p;
  p.x = x;
  p.y = y;
  p.theta = theta;
  return p;
}

Array frame(int64_t stamp, bool position = true, bool velocity = false)
{
  Array batch;
  batch.header.frame_id = "odom";
  batch.header.stamp = rclcpp::Time(stamp);
  batch.scan_stamp = batch.header.stamp;
  auto & state = batch.states.emplace_back();
  state.track_id = "motion-1";
  state.position.x = 1.;
  state.last_observation_stamp = batch.header.stamp;
  state.occupancy_radius = .35;
  state.position_valid = position;
  state.velocity_valid = velocity;
  state.velocity.x = .5;
  return batch;
}

TEST(PedestrianCollision, SamePathAtDifferentTimes)
{
  PedestrianPrediction person;
  person.x = .5;
  person.y = -1.;
  person.vy = 2.;
  person.radius = .1;
  person.horizon = 2.;
  Trajectory early{{}, {pose(0., 0.), pose(1., 0.)}};
  auto hit = pedestrianCollision(early, 1., 1., footprint, {person});
  EXPECT_TRUE(hit.collision);
  EXPECT_LT(hit.first_time, 1.);
  person.y = -4.;
  EXPECT_FALSE(pedestrianCollision(early, 1., 1., footprint, {person}).collision);
}

TEST(PedestrianCollision, FootprintAndRelativeMotion)
{
  Trajectory robot{{}, {pose(0., 0., 1.5707963267948966), pose(.5, 0.)}};
  PedestrianPrediction person;
  person.y = .26;
  person.radius = .07;
  person.horizon = 1.;
  EXPECT_TRUE(pedestrianCollision(robot, .1, 0., footprint, {person}).collision);
  person.y = .4;
  EXPECT_FALSE(pedestrianCollision(robot, .1, .1, footprint, {person}).collision);
  person.y = 0.;
  person.x = .4;
  person.vx = 5.;
  EXPECT_FALSE(pedestrianCollision(robot, .1, .1, footprint, {person}).collision);
  person.vx = 0.;
  EXPECT_TRUE(pedestrianCollision(robot, .1, .1, footprint, {person}).collision);
}

TEST(PedestrianCollision, SameDirectionAndMovingAway)
{
  Trajectory robot{{}, {pose(0., 0.), pose(.3, 0.), pose(.6, 0.)}};
  PedestrianPrediction person;
  person.x = .6;
  person.vx = .3;
  person.radius = .15;
  person.horizon = 2.;
  EXPECT_FALSE(pedestrianCollision(robot, 1., 2., footprint, {person}).collision);
  person.vx = .6;
  EXPECT_FALSE(pedestrianCollision(robot, 1., 2., footprint, {person}).collision);
  person.vx = 0.;
  EXPECT_TRUE(pedestrianCollision(robot, 1., 2., footprint, {person}).collision);
}

TEST(PedestrianCollision, SoftGrowthPenalizesClearTrajectory)
{
  PedestrianPrediction person;
  person.x = .42;
  person.radius = .21;
  person.physical_radius = .15;
  person.growth = .2;
  person.max_growth = .25;
  person.horizon = 1.;
  Trajectory stopped{{}, {pose(0., 0.), pose(0., 0.)}};
  const auto hard = pedestrianCollision(stopped, 1., 1., footprint, {person});
  const auto soft = pedestrianCollision(stopped, 1., 1., footprint, {person}, true);
  EXPECT_TRUE(hard.collision);
  EXPECT_FALSE(soft.collision);
  EXPECT_NEAR(soft.soft_cost, .19, 1e-9);

  person.x = .8;
  const auto outside = pedestrianCollision(stopped, 1., 1., footprint, {person}, true);
  EXPECT_FALSE(outside.collision);
  EXPECT_DOUBLE_EQ(outside.soft_cost, 0.);
}

TEST(PedestrianCollision, SoftModeRejectsPhysicalOverlapBetweenSamples)
{
  PedestrianPrediction person;
  person.x = .5;
  person.y = -1.;
  person.vy = 2.;
  person.radius = .16;
  person.physical_radius = .1;
  person.horizon = 1.;
  Trajectory robot{{}, {pose(0., 0.), pose(1., 0.)}};
  const auto result = pedestrianCollision(robot, 1., 1., footprint, {person}, true);
  EXPECT_TRUE(result.collision);
  EXPECT_GT(result.first_time, 0.);
  EXPECT_LT(result.first_time, 1.);
}

TEST(PedestrianInput, SoftGrowthScalesSpeedAndKeepsStaleFloor)
{
  PedestrianConfig config;
  config.soft_growth_enabled = true;
  for (const double speed : {0., .03, .075, .12, .2}) {
    PedestrianInput input(config, "odom", nullptr);
    auto batch = frame(S, true, true);
    batch.kalman_estimate = true;
    batch.states[0].velocity.x = speed;
    input.receive(batch, S);
    const double scale = std::clamp((speed - .03) / (.12 - .03), 0., 1.);
    EXPECT_NEAR(input.snapshot(S).people.at(0).growth, .08 * scale, 1e-10);
    auto missed = frame(S + 200000000, false, false);
    missed.kalman_estimate = true;
    input.receive(missed, S + 200000000);
    EXPECT_NEAR(input.snapshot(S + 500000000).people.at(0).growth, .24 * scale, 1e-10);
    EXPECT_NEAR(
      input.snapshot(S + 500000001).people.at(0).growth,
      .24 * std::max(scale, .5), 1e-10);
  }

  PedestrianInput unknown(config, "odom", nullptr);
  unknown.receive(frame(S), S);
  EXPECT_DOUBLE_EQ(unknown.snapshot(S).people.at(0).growth, .35);
}

TEST(PedestrianInput, InvalidObservationsDoNotRefreshOrBridgeHistory)
{
  PedestrianInput input({}, "odom", nullptr);
  input.receive(frame(S, true, true), S);
  auto missing = frame(S + S / 5, false, false);
  missing.states[0].last_observation_stamp = rclcpp::Time(S);
  input.receive(missing, S + S / 5);
  auto next = frame(S + 2 * S / 5);
  next.states[0].position.x = 1.3;
  input.receive(next, S + 2 * S / 5);
  auto held = input.snapshot(S + 2 * S / 5);
  ASSERT_EQ(held.people.size(), 1u);
  EXPECT_EQ(held.people[0].source, VelocitySource::Held);
  EXPECT_NEAR(held.people[0].velocity_age, .4, 1e-8);
  auto expired = input.snapshot(S + 3 * S / 5);
  ASSERT_EQ(expired.people.size(), 1u);
  EXPECT_EQ(expired.people[0].source, VelocitySource::Unknown);
  EXPECT_DOUBLE_EQ(expired.people[0].vx, 0.);
  EXPECT_GT(expired.people[0].radiusAt(1.), expired.people[0].radiusAt(0.));
}

TEST(PedestrianInput, KalmanVelocityDoesNotCompeteWithDifference)
{
  PedestrianInput input({}, "odom", nullptr);
  auto batch = frame(S, true, true);
  batch.kalman_estimate = true;
  input.receive(batch, S);
  EXPECT_EQ(input.snapshot(S).people.at(0).source, VelocitySource::Kalman);
  batch = frame(S + S / 5);
  batch.kalman_estimate = true;
  batch.states[0].position.x = 1.1;
  input.receive(batch, S + S / 5);
  const auto row = input.snapshot(S + S / 5).people.at(0);
  EXPECT_EQ(row.source, VelocitySource::Unknown);
  EXPECT_DOUBLE_EQ(row.velocity_age, -1.);
}

TEST(PedestrianInput, ConvergedKalmanCacheUsesObservationTimeUntil850Milliseconds)
{
  PedestrianInput input({}, "odom", nullptr);
  auto batch = frame(S, true, true);
  batch.kalman_estimate = true;
  input.receive(batch, S);

  auto missing = frame(S + S / 5, false, false);
  missing.kalman_estimate = true;
  missing.status = "missing_sort_pair";
  missing.states[0].last_observation_stamp = rclcpp::Time(S);
  input.receive(missing, S + S / 5);

  const auto snapshot = input.snapshot(S + 850000000);
  ASSERT_EQ(snapshot.people.size(), 1u);
  const auto & person = snapshot.people.front();
  EXPECT_EQ(person.source, VelocitySource::Held);
  EXPECT_DOUBLE_EQ(person.age, .85);
  EXPECT_DOUBLE_EQ(person.velocity_age, .85);
  EXPECT_DOUBLE_EQ(person.vx, .5);
  EXPECT_NEAR(person.x, 1.425, 1e-9);
  EXPECT_TRUE(input.snapshot(S + 850000001).people.empty());
}

TEST(PedestrianInput, TwoKalmanCachesAdvanceAndExpireIndependently)
{
  PedestrianInput input({}, "odom", nullptr);
  auto first = frame(S, true, true);
  first.kalman_estimate = true;
  auto second = first.states[0];
  second.track_id = "motion-2";
  second.position.x = 3.;
  second.velocity.x = -.25;
  first.states.push_back(second);
  input.receive(first, S);

  auto update = frame(S + S / 5, true, true);
  update.kalman_estimate = true;
  update.states[0].position.x = 1.1;
  input.receive(update, S + S / 5);
  const auto both = input.snapshot(S + 850000000);
  ASSERT_EQ(both.people.size(), 2u);
  EXPECT_EQ(both.people[0].id, "motion-1");
  EXPECT_EQ(both.people[1].id, "motion-2");
  EXPECT_NEAR(both.people[0].x, 1.425, 1.e-9);
  EXPECT_NEAR(both.people[1].x, 2.7875, 1.e-9);
  EXPECT_EQ(both.people[0].source, VelocitySource::Kalman);
  EXPECT_EQ(both.people[1].source, VelocitySource::Held);
  const auto surviving = input.snapshot(S + 850000001);
  ASSERT_EQ(surviving.people.size(), 1u);
  EXPECT_EQ(surviving.people[0].id, "motion-1");
  EXPECT_TRUE(input.snapshot(S + 1050000001).people.empty());
}

TEST(PedestrianCollision, EveryPersonCanRejectAndSoftCostIsThePeak)
{
  Trajectory stopped{{}, {pose(0., 0.), pose(0., 0.)}};
  PedestrianPrediction a;
  a.x = .45;
  a.id = "motion-1";
  a.physical_radius = .15;
  a.radius = .30;
  a.horizon = 1.;
  auto b = a;
  b.x = -.42;
  b.id = "motion-2";
  const auto soft = pedestrianCollision(stopped, 1., 1., footprint, {a, b}, true);
  EXPECT_FALSE(soft.collision);
  EXPECT_NEAR(soft.soft_cost, .08, 1.e-9);
  b.x = -.30;
  const auto hit = pedestrianCollision(stopped, 1., 1., footprint, {a, b}, true);
  EXPECT_TRUE(hit.collision);
  EXPECT_EQ(hit.id, b.id);
  EXPECT_DOUBLE_EQ(hit.first_time, 0.);
}

TEST(PedestrianInput, ColdStartAndLegacyKeepOriginalCacheAge)
{
  PedestrianInput input({}, "odom", nullptr);
  auto cold = frame(S);
  cold.kalman_estimate = true;
  cold.states[0].track_id = "cold";
  auto legacy = frame(S, true, true);
  input.receive(cold, S);
  // Use a second cache to exercise the legacy publisher contract independently.
  PedestrianInput legacy_input({}, "odom", nullptr);
  legacy_input.receive(legacy, S);
  EXPECT_TRUE(input.snapshot(S + 500000001).people.empty());
  EXPECT_TRUE(legacy_input.snapshot(S + 500000001).people.empty());
}

TEST(PedestrianInput, ColdStartReplacesPriorConvergedVelocityForSameTrack)
{
  PedestrianInput input({}, "odom", nullptr);
  auto batch = frame(S, true, true);
  batch.kalman_estimate = true;
  input.receive(batch, S);
  auto cold = frame(S + S / 5);
  cold.kalman_estimate = true;
  input.receive(cold, S + S / 5);
  const auto snapshot = input.snapshot(S + S / 2);
  ASSERT_EQ(snapshot.people.size(), 1u);
  EXPECT_EQ(snapshot.people.front().source, VelocitySource::Unknown);
  EXPECT_TRUE(input.snapshot(S + 700000001).people.empty());
}

TEST(PedestrianInput, RejectsMalformedFutureAndDuplicateRows)
{
  PedestrianInput input({}, "odom", nullptr);
  auto message = frame(S, true, true);
  message.states[0].last_observation_stamp = rclcpp::Time(S + 1);
  input.receive(message, S);
  EXPECT_TRUE(input.snapshot(S).people.empty());
  message = frame(S + S / 5, true, true);
  message.states[0].position.x = std::numeric_limits<double>::quiet_NaN();
  input.receive(message, S + S / 5);
  EXPECT_TRUE(input.snapshot(S + S / 5).people.empty());
  message = frame(S + 2 * S / 5, true, true);
  const auto duplicate = message.states[0];
  message.states.push_back(duplicate);
  input.receive(message, S + 2 * S / 5);
  EXPECT_TRUE(input.snapshot(S + 2 * S / 5).people.empty());
  message = frame(S + 3 * S / 5, true, true);
  message.epoch = 1;
  input.receive(message, S + 3 * S / 5);
  ASSERT_EQ(input.snapshot(S + 3 * S / 5).people.size(), 1u);
  message.epoch = 0;
  input.receive(message, S + 3 * S / 5);
  EXPECT_EQ(input.snapshot(S + 3 * S / 5).people.at(0).epoch, 1u);
}

TEST(PedestrianInput, DifferenceHoldAgeAndEpoch)
{
  PedestrianInput input({}, "odom", nullptr);
  input.receive(frame(S), S);
  auto first = input.snapshot(S);
  ASSERT_EQ(first.people.size(), 1u);
  EXPECT_EQ(first.people[0].source, VelocitySource::Unknown);
  auto next = frame(S + S / 5);
  next.states[0].position.x = 1.1;
  input.receive(next, S + S / 5);
  auto derived = input.snapshot(S + S / 5);
  ASSERT_EQ(derived.people.size(), 1u);
  EXPECT_EQ(derived.people[0].source, VelocitySource::Difference);
  EXPECT_NEAR(derived.people[0].vx, .5, 1e-8);
  auto ols = frame(S + 2 * S / 5, true, true);
  ols.states[0].position.x = 1.2;
  input.receive(ols, S + 2 * S / 5);
  auto invalid = ols;
  invalid.header.stamp = rclcpp::Time(S + S / 2);
  invalid.states[0].position_valid = invalid.states[0].velocity_valid = false;
  input.receive(invalid, S + S / 2);
  auto held = input.snapshot(S + S / 2);
  ASSERT_EQ(held.people.size(), 1u);
  EXPECT_EQ(held.people[0].source, VelocitySource::Held);
  EXPECT_NEAR(held.people[0].x, 1.25, 1e-8);
  EXPECT_TRUE(input.snapshot(2 * S).people.empty());
  auto reset = frame(2 * S, false, false);
  reset.epoch = 1;
  input.receive(reset, 2 * S);
  EXPECT_TRUE(input.snapshot(2 * S).people.empty());
}

TEST(PedestrianInput, TimeFramesAndInvalidDifference)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto tf = std::make_shared<tf2_ros::Buffer>(clock);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.translation.x = 10.;
  transform.transform.rotation.z = transform.transform.rotation.w = std::sqrt(.5);
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  PedestrianInput input({}, "map", tf);
  input.receive(frame(S, true, true), S - 1);
  EXPECT_TRUE(input.snapshot(S - 1).people.empty());
  input.receive(frame(S, true, true), S);
  auto result = input.snapshot(S);
  ASSERT_EQ(result.people.size(), 1u);
  EXPECT_NEAR(result.people[0].x, 10., 1e-8);
  EXPECT_NEAR(result.people[0].y, 1., 1e-8);
  EXPECT_NEAR(result.people[0].vx, 0., 1e-8);
  EXPECT_NEAR(result.people[0].vy, .5, 1e-8);
  auto missing = frame(S + S / 10, false, true);
  missing.states[0].last_observation_stamp = rclcpp::Time(S);
  input.receive(missing, S + S / 10);
  result = input.snapshot(S + S / 10);
  EXPECT_EQ(result.people.at(0).source, VelocitySource::Held);
  EXPECT_NEAR(result.people.at(0).y, 1.05, 1e-8);
  EXPECT_TRUE(input.snapshot(S / 2).people.empty());

  PedestrianInput difference({}, "odom", nullptr);
  difference.receive(frame(S), S);
  auto teleport = frame(S + S / 100);
  teleport.states[0].position.x = 5.;
  difference.receive(teleport, S + S / 100);
  EXPECT_EQ(difference.snapshot(S + S / 100).people.at(0).source, VelocitySource::Unknown);
  auto gap = frame(S + S / 2);
  gap.states[0].position.x = 5.1;
  difference.receive(gap, S + S / 2);
  EXPECT_EQ(difference.snapshot(S + S / 2).people.at(0).source, VelocitySource::Unknown);
}

TEST(PedestrianCollision, UnknownMotionInflatesWithCap)
{
  PedestrianPrediction person;
  person.radius = .35;
  person.age = .2;
  person.growth = .6;
  person.max_growth = .25;
  EXPECT_NEAR(person.radiusAt(0.), .47, 1e-9);
  EXPECT_NEAR(person.radiusAt(1.), .60, 1e-9);
  EXPECT_NEAR(person.radiusAt(2.), .60, 1e-9);
}
}  // namespace
}  // namespace my_dwa_controller
