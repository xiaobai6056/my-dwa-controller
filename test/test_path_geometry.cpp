#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "my_dwa_controller/path_geometry.hpp"

namespace my_dwa_controller
{
namespace
{
constexpr double kTolerance = 1e-12;

Pose2D makePose(double x, double y, double theta = 0.0)
{
  Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  return pose;
}

void expectPoint(const Pose2D & point, double x, double y)
{
  EXPECT_NEAR(point.x, x, kTolerance);
  EXPECT_NEAR(point.y, y, kTolerance);
}

double pathLength(const std::vector<Pose2D> & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return length;
}

TEST(PathGeometry, ProjectsInsideSegment)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(4.0, 0.0, 2.0)};
  const auto projection = projectOntoPath(makePose(1.0, 2.0, 0.8), path);
  ASSERT_TRUE(projection.has_value());
  expectPoint(projection->point, 1.0, 0.0);
  EXPECT_EQ(projection->segment_index, 0u);
  EXPECT_DOUBLE_EQ(projection->t, 0.25);
  EXPECT_DOUBLE_EQ(projection->squared_distance, 4.0);
  EXPECT_DOUBLE_EQ(projection->point.theta, 0.5);
}

TEST(PathGeometry, SelectsNearestSegment)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(2.0, 3.0)};
  const auto projection = projectOntoPath(makePose(3.0, 2.0), path);
  ASSERT_TRUE(projection.has_value());
  expectPoint(projection->point, 2.0, 2.0);
  EXPECT_EQ(projection->segment_index, 1u);
  EXPECT_NEAR(projection->t, 2.0 / 3.0, kTolerance);
  EXPECT_NEAR(projection->squared_distance, 1.0, kTolerance);
}

TEST(PathGeometry, ClampsProjectionToSegmentEndpoints)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(4.0, 0.0)};
  const auto before = projectOntoPath(makePose(-2.0, 1.0), path);
  const auto after = projectOntoPath(makePose(6.0, 1.0), path);
  ASSERT_TRUE(before.has_value());
  ASSERT_TRUE(after.has_value());
  expectPoint(before->point, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(before->t, 0.0);
  EXPECT_DOUBLE_EQ(before->squared_distance, 5.0);
  expectPoint(after->point, 4.0, 0.0);
  EXPECT_DOUBLE_EQ(after->t, 1.0);
  EXPECT_DOUBLE_EQ(after->squared_distance, 5.0);
}

TEST(PathGeometry, EmptyPathHasNoProjectionAndNoExtractedPoints)
{
  EXPECT_FALSE(projectOntoPath(makePose(0.0, 0.0), {}).has_value());
  EXPECT_TRUE(extractForwardPath({}, PathProjection{}, 2.0).empty());
}

TEST(PathGeometry, SinglePointUsesIndexZeroAndParameterZero)
{
  const std::vector<Pose2D> path{makePose(2.0, 3.0, 0.7)};
  const auto projection = projectOntoPath(makePose(5.0, 7.0), path);
  ASSERT_TRUE(projection.has_value());
  expectPoint(projection->point, 2.0, 3.0);
  EXPECT_EQ(projection->segment_index, 0u);
  EXPECT_DOUBLE_EQ(projection->t, 0.0);
  EXPECT_DOUBLE_EQ(projection->squared_distance, 25.0);
  const auto extracted = extractForwardPath(path, *projection, 10.0);
  ASSERT_EQ(extracted.size(), 1u);
  EXPECT_DOUBLE_EQ(extracted.front().theta, 0.7);
}

TEST(PathGeometry, HandlesZeroLengthSegmentsAndAllRepeatedPoints)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(0.0, 0.0), makePose(2.0, 0.0)};
  const auto projection = projectOntoPath(makePose(1.0, 1.0), path);
  ASSERT_TRUE(projection.has_value());
  expectPoint(projection->point, 1.0, 0.0);
  EXPECT_EQ(projection->segment_index, 1u);
  EXPECT_DOUBLE_EQ(projection->t, 0.5);
  const std::vector<Pose2D> repeated(4, makePose(2.0, 3.0));
  const auto single = projectOntoPath(makePose(3.0, 4.0), repeated);
  ASSERT_TRUE(single.has_value());
  EXPECT_EQ(single->segment_index, 0u);
  EXPECT_DOUBLE_EQ(single->t, 0.0);
  EXPECT_DOUBLE_EQ(single->squared_distance, 2.0);
  EXPECT_EQ(extractForwardPath(repeated, *single, 10.0).size(), 1u);
}

TEST(PathGeometry, EqualDistanceSelectsEarliestSegment)
{
  const std::vector<Pose2D> corner{makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(2.0, 3.0)};
  const auto vertex = projectOntoPath(makePose(2.0, 0.0), corner);
  ASSERT_TRUE(vertex.has_value());
  EXPECT_EQ(vertex->segment_index, 0u);
  EXPECT_DOUBLE_EQ(vertex->t, 1.0);
  const std::vector<Pose2D> crossing{
    makePose(-1.0, -1.0), makePose(1.0, 1.0), makePose(-1.0, 1.0), makePose(1.0, -1.0)};
  const auto intersection = projectOntoPath(makePose(0.0, 0.0), crossing);
  ASSERT_TRUE(intersection.has_value());
  EXPECT_EQ(intersection->segment_index, 0u);
  EXPECT_DOUBLE_EQ(intersection->t, 0.5);
  EXPECT_DOUBLE_EQ(intersection->squared_distance, 0.0);
}

TEST(PathGeometry, KeepsShortNonzeroSegments)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(1e-9, 0.0)};
  const auto projection = projectOntoPath(makePose(0.5e-9, 1.0), path);
  ASSERT_TRUE(projection.has_value());
  EXPECT_NEAR(projection->point.x, 0.5e-9, 1e-24);
  EXPECT_DOUBLE_EQ(projection->t, 0.5);
}

TEST(PathGeometry, FirstSegmentProjectionWinsOverRoundedEqualInitialVertex)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(1e-9, 0.0)};
  const auto projection = projectOntoPath(makePose(0.5e-9, 10.0), path);
  ASSERT_TRUE(projection.has_value());
  EXPECT_NEAR(projection->point.x, 0.5e-9, 1e-24);
  EXPECT_DOUBLE_EQ(projection->t, 0.5);
  EXPECT_DOUBLE_EQ(projection->squared_distance, 100.0);
}

TEST(PathGeometry, ExtractsInterpolatedEndpointOnStraightPath)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(4.0, 0.0, 2.0)};
  const auto projection = projectOntoPath(makePose(1.0, 2.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, 2.0);
  ASSERT_EQ(extracted.size(), 2u);
  expectPoint(extracted.front(), 1.0, 0.0);
  expectPoint(extracted.back(), 3.0, 0.0);
  EXPECT_DOUBLE_EQ(extracted.back().theta, 1.5);
  EXPECT_NEAR(pathLength(extracted), 2.0, kTolerance);
}

TEST(PathGeometry, ExtractsByPolylineArcLengthAcrossCorner)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(2.0, 3.0)};
  const auto projection = projectOntoPath(makePose(1.0, 1.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, 3.0);
  ASSERT_EQ(extracted.size(), 3u);
  expectPoint(extracted[0], 1.0, 0.0);
  expectPoint(extracted[1], 2.0, 0.0);
  expectPoint(extracted[2], 2.0, 2.0);
  EXPECT_NEAR(pathLength(extracted), 3.0, kTolerance);
}

TEST(PathGeometry, ExtractsAcrossDiagonalSegment)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(3.0, 4.0), makePose(6.0, 4.0)};
  const auto projection = projectOntoPath(makePose(1.5, 2.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, 4.0);
  ASSERT_EQ(extracted.size(), 3u);
  expectPoint(extracted[1], 3.0, 4.0);
  expectPoint(extracted.back(), 4.5, 4.0);
  EXPECT_NEAR(pathLength(extracted), 4.0, kTolerance);
}

TEST(PathGeometry, StopsAtOriginalEndWhenRemainingPathIsShorter)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(2.0, 3.0)};
  const auto projection = projectOntoPath(makePose(1.0, 0.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, 100.0);
  ASSERT_EQ(extracted.size(), 3u);
  expectPoint(extracted.back(), 2.0, 3.0);
  EXPECT_NEAR(pathLength(extracted), 4.0, kTolerance);
}

TEST(PathGeometry, ExactLengthStopsAtVertexWithoutDuplicate)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(2.0, 3.0)};
  const auto projection = projectOntoPath(makePose(1.0, 0.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, 1.0);
  ASSERT_EQ(extracted.size(), 2u);
  expectPoint(extracted.back(), 2.0, 0.0);
  EXPECT_DOUBLE_EQ(pathLength(extracted), 1.0);
}

TEST(PathGeometry, DiagonalVertexDoesNotAddRoundedDuplicate)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(1.0, 1.0), makePose(2.0, 1.0)};
  const auto projection = projectOntoPath(makePose(0.0, 0.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, std::sqrt(2.0));
  ASSERT_EQ(extracted.size(), 2u);
  expectPoint(extracted.back(), 1.0, 1.0);
  EXPECT_NEAR(pathLength(extracted), std::sqrt(2.0), kTolerance);
}

TEST(PathGeometry, ZeroLengthReturnsOnlyProjection)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(4.0, 0.0)};
  const auto projection = projectOntoPath(makePose(1.0, 2.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, 0.0);
  ASSERT_EQ(extracted.size(), 1u);
  expectPoint(extracted.front(), 1.0, 0.0);
}

TEST(PathGeometry, ProjectionAtVertexContinuesOnlyForward)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(2.0, 3.0)};
  const auto projection = projectOntoPath(makePose(2.0, 0.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, 1.0);
  ASSERT_EQ(extracted.size(), 2u);
  expectPoint(extracted.front(), 2.0, 0.0);
  expectPoint(extracted.back(), 2.0, 1.0);
}

TEST(PathGeometry, ProjectionAtEndReturnsOnlyEndpoint)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(2.0, 3.0)};
  const auto projection = projectOntoPath(makePose(4.0, 4.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, 5.0);
  ASSERT_EQ(extracted.size(), 1u);
  expectPoint(extracted.front(), 2.0, 3.0);
}

TEST(PathGeometry, SkipsRepeatedCoordinatesDuringExtraction)
{
  const std::vector<Pose2D> path{
    makePose(0.0, 0.0), makePose(0.0, 0.0), makePose(2.0, 0.0),
    makePose(2.0, 0.0), makePose(2.0, 2.0)};
  const auto projection = projectOntoPath(makePose(1.0, 0.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, 2.0);
  ASSERT_EQ(extracted.size(), 3u);
  expectPoint(extracted.front(), 1.0, 0.0);
  expectPoint(extracted[1], 2.0, 0.0);
  expectPoint(extracted.back(), 2.0, 1.0);
  EXPECT_DOUBLE_EQ(pathLength(extracted), 2.0);
}

TEST(PathGeometry, KeepsNonconsecutiveRepeatedCoordinatesOnLoop)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(0.0, 0.0)};
  const auto projection = projectOntoPath(makePose(0.0, 0.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, 4.0);
  ASSERT_EQ(extracted.size(), 3u);
  expectPoint(extracted.back(), 0.0, 0.0);
  EXPECT_DOUBLE_EQ(pathLength(extracted), 4.0);
}

TEST(PathGeometry, InterpolatesThetaAcrossPiUsingShortestAngle)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0, 3.0), makePose(2.0, 0.0, -3.0)};
  const auto projection = projectOntoPath(makePose(1.0, 0.0), path);
  ASSERT_TRUE(projection.has_value());
  const double pi = std::acos(-1.0);
  EXPECT_NEAR(projection->point.theta, pi, kTolerance);
  const auto extracted = extractForwardPath(path, *projection, 0.5);
  ASSERT_EQ(extracted.size(), 2u);
  EXPECT_NEAR(extracted.back().theta, 3.0 + 0.75 * (2.0 * pi - 6.0), kTolerance);
}

TEST(PathGeometry, InterpolatesReversePiCrossingAndPreservesEndpointAngles)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0, -3.0), makePose(2.0, 0.0, 3.0)};
  const auto middle = projectOntoPath(makePose(1.0, 0.0), path);
  ASSERT_TRUE(middle.has_value());
  EXPECT_NEAR(middle->point.theta, -std::acos(-1.0), kTolerance);
  const auto start = projectOntoPath(makePose(0.0, 0.0), path);
  const auto end = projectOntoPath(makePose(2.0, 0.0), path);
  ASSERT_TRUE(start.has_value());
  ASSERT_TRUE(end.has_value());
  EXPECT_DOUBLE_EQ(start->point.theta, -3.0);
  EXPECT_DOUBLE_EQ(end->point.theta, 3.0);
}

TEST(PathGeometry, InterpolatesFromActualSegmentAfterRepeatedVertex)
{
  const std::vector<Pose2D> path{
    makePose(0.0, 0.0, 0.0), makePose(1.0, 0.0, 1.0),
    makePose(1.0, 0.0, 2.0), makePose(2.0, 0.0, 4.0)};
  const auto projection = projectOntoPath(makePose(0.5, 0.0), path);
  ASSERT_TRUE(projection.has_value());
  const auto extracted = extractForwardPath(path, *projection, 1.0);
  ASSERT_EQ(extracted.size(), 3u);
  expectPoint(extracted.back(), 1.5, 0.0);
  // 终点位于最后一段中点，使用该段端点的 theta：从 2 线性插值到 4。
  EXPECT_DOUBLE_EQ(extracted.back().theta, 3.0);
}

TEST(PathGeometry, RejectsNonFiniteRobotComponentsEvenOnEmptyPath)
{
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<double, 3> invalid{{infinity, -infinity, std::nan("")}};
  const std::array<double Pose2D::*, 3> fields{{&Pose2D::x, &Pose2D::y, &Pose2D::theta}};
  for (auto field : fields) {
    for (double value : invalid) {
      auto robot = makePose(0.0, 0.0);
      robot.*field = value;
      EXPECT_THROW(projectOntoPath(robot, {}), std::invalid_argument);
    }
  }
}

TEST(PathGeometry, RejectsNonFinitePathComponentsInBothFunctions)
{
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<double, 3> invalid{{infinity, -infinity, std::nan("")}};
  const std::array<double Pose2D::*, 3> fields{{&Pose2D::x, &Pose2D::y, &Pose2D::theta}};
  for (auto field : fields) {
    for (double value : invalid) {
      std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(1.0, 0.0)};
      path.back().*field = value;
      EXPECT_THROW(projectOntoPath(makePose(0.0, 0.0), path), std::invalid_argument);
      // 即使请求长度为零，也明确拒绝路径后部的非法数据。
      EXPECT_THROW(extractForwardPath(path, PathProjection{}, 0.0), std::invalid_argument);
    }
  }
}

TEST(PathGeometry, RejectsNegativeAndNonFiniteLengths)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0)};
  const std::array<double, 4> invalid{{-1.0, std::nan(""), INFINITY, -INFINITY}};
  for (double length : invalid) {
    EXPECT_THROW(extractForwardPath(path, PathProjection{}, length), std::invalid_argument);
    EXPECT_THROW(extractForwardPath({}, PathProjection{}, length), std::invalid_argument);
  }
}

TEST(PathGeometry, RejectsInvalidProjectionIndexAndParameter)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0)};
  PathProjection projection;
  projection.segment_index = 1;
  EXPECT_THROW(extractForwardPath(path, projection, 1.0), std::invalid_argument);
  projection.segment_index = std::numeric_limits<std::size_t>::max();
  EXPECT_THROW(extractForwardPath(path, projection, 1.0), std::invalid_argument);
  projection.segment_index = 0;
  const std::array<double, 5> invalid{{-0.1, 1.1, std::nan(""), INFINITY, -INFINITY}};
  for (double t : invalid) {
    projection.t = t;
    EXPECT_THROW(extractForwardPath(path, projection, 1.0), std::invalid_argument);
  }
  projection.t = 0.5;
  EXPECT_THROW(extractForwardPath({makePose(0.0, 0.0)}, projection, 1.0), std::invalid_argument);
}

TEST(PathGeometry, RejectsUnrepresentableSquaredDistance)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0)};
  EXPECT_THROW(projectOntoPath(makePose(1e308, 0.0), path), std::overflow_error);
}

TEST(PathGeometry, DistanceToPathReturnsEuclideanDistanceToWholePolyline)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(2.0, 3.0)};
  const auto distance = distanceToPath(makePose(5.0, 2.0), path);
  ASSERT_TRUE(distance.has_value());
  // 最近点为 (2,2)，欧氏距离为 3，而距离平方为 9。
  EXPECT_DOUBLE_EQ(*distance, 3.0);
  const auto on_path = distanceToPath(makePose(2.0, 1.0), path);
  ASSERT_TRUE(on_path.has_value());
  EXPECT_DOUBLE_EQ(*on_path, 0.0);
  const auto beyond_end = distanceToPath(makePose(5.0, 7.0), path);
  ASSERT_TRUE(beyond_end.has_value());
  EXPECT_DOUBLE_EQ(*beyond_end, 5.0);
}

TEST(PathGeometry, DistanceToPathHandlesSingleAndRepeatedPoints)
{
  const std::vector<Pose2D> path{makePose(2.0, 3.0)};
  const auto single = distanceToPath(makePose(5.0, 7.0), path);
  ASSERT_TRUE(single.has_value());
  EXPECT_DOUBLE_EQ(*single, 5.0);
  const auto repeated = distanceToPath(makePose(5.0, 7.0), std::vector<Pose2D>(3, path.front()));
  ASSERT_TRUE(repeated.has_value());
  EXPECT_DOUBLE_EQ(*repeated, 5.0);
}

TEST(PathGeometry, DistanceToPathReturnsNoValueForEmptyPath)
{
  EXPECT_FALSE(distanceToPath(makePose(0.0, 0.0), {}).has_value());
}

TEST(PathGeometry, DistanceToPathPropagatesInvalidInputErrors)
{
  const std::array<double, 3> invalid{{INFINITY, -INFINITY, std::nan("")}};
  const std::array<double Pose2D::*, 3> fields{{&Pose2D::x, &Pose2D::y, &Pose2D::theta}};
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(1.0, 0.0)};
  for (auto field : fields) {
    for (double value : invalid) {
      auto point = makePose(0.0, 0.0);
      point.*field = value;
      EXPECT_THROW(distanceToPath(point, path), std::invalid_argument);
      EXPECT_THROW(distanceToPath(point, {}), std::invalid_argument);
      auto invalid_path = path;
      invalid_path.back().*field = value;
      EXPECT_THROW(distanceToPath(makePose(0.0, 0.0), invalid_path), std::invalid_argument);
    }
  }
}

TEST(PathGeometry, DistanceToPathPropagatesProjectionOverflow)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0)};
  EXPECT_THROW(distanceToPath(makePose(1e308, 0.0), path), std::overflow_error);
}

TEST(PathGeometry, ConvertsPoseInBothDirectionsWithoutChangingPlanarValues)
{
  const auto pose = toPose(makePose(1.5, -2.0, 2.3));
  EXPECT_DOUBLE_EQ(pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(pose.position.y, -2.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
  EXPECT_NEAR(pose.orientation.z, std::sin(2.3 / 2.0), kTolerance);
  EXPECT_NEAR(pose.orientation.w, std::cos(2.3 / 2.0), kTolerance);
  const auto planar = toPose2D(pose);
  expectPoint(planar, 1.5, -2.0);
  EXPECT_NEAR(planar.theta, 2.3, kTolerance);
}

TEST(PathGeometry, NormalizesQuaternionBeforeExtractingYaw)
{
  auto pose = toPose(makePose(1.5, -2.0, -2.3));
  pose.position.z = 4.0;
  pose.orientation.z *= 3.0;
  pose.orientation.w *= 3.0;
  const auto planar = toPose2D(pose);
  expectPoint(planar, 1.5, -2.0);
  EXPECT_NEAR(planar.theta, -2.3, kTolerance);
}

TEST(PathGeometry, PoseConversionRetainsInvalidInputExceptions)
{
  const std::array<double, 3> invalid{{INFINITY, -INFINITY, std::nan("")}};
  for (double value : invalid) {
    for (std::size_t component = 0; component < 7; ++component) {
      auto pose = toPose(makePose(0.0, 0.0));
      const std::array<double *, 7> fields{{
        &pose.position.x, &pose.position.y, &pose.position.z,
        &pose.orientation.x, &pose.orientation.y, &pose.orientation.z, &pose.orientation.w}};
      *fields[component] = value;
      EXPECT_THROW(toPose2D(pose), std::invalid_argument);
    }
  }
  geometry_msgs::msg::Pose zero_quaternion;
  zero_quaternion.orientation.w = 0.0;
  EXPECT_THROW(toPose2D(zero_quaternion), std::invalid_argument);
}


TEST(PathGeometry, LookaheadPointMatchesExtractionAcrossStraightAndCorner)
{
  const std::vector<Pose2D> path{
    makePose(0.0, 0.0), makePose(0.3, 0.0), makePose(0.3, 1.0)};
  const auto projection = projectOntoPath(makePose(0.0, 0.1), path);
  ASSERT_TRUE(projection);
  const auto point = lookAheadOnPath(path, *projection, 0.6);
  ASSERT_TRUE(point);
  expectPoint(point->point, 0.3, 0.3);
  EXPECT_NEAR(point->distance, 0.6, kTolerance);
  ASSERT_TRUE(point->forward_heading);
  EXPECT_NEAR(*point->forward_heading, std::acos(-1.0) / 2.0, kTolerance);
  const auto extracted = extractForwardPath(path, *projection, 0.6);
  expectPoint(point->point, extracted.back().x, extracted.back().y);
  const auto straight = lookAheadOnPath(path, *projection, 0.2);
  ASSERT_TRUE(straight);
  expectPoint(straight->point, 0.2, 0.0);
}

TEST(PathGeometry, LookaheadAtEndKeepsForwardTangentAcrossRepeatedPoints)
{
  const std::vector<Pose2D> path{
    makePose(0.0, 0.0, 2.0), makePose(1.0, 0.0, 2.0),
    makePose(1.0, 0.0, -2.0), makePose(1.0, 0.0, -2.0)};
  const auto projection = projectOntoPath(makePose(2.0, 0.0), path);
  ASSERT_TRUE(projection);
  const auto result = lookAheadOnPath(path, *projection, 0.6);
  ASSERT_TRUE(result);
  expectPoint(result->point, 1.0, 0.0);
  EXPECT_DOUBLE_EQ(result->distance, 0.0);
  ASSERT_TRUE(result->forward_heading);
  EXPECT_DOUBLE_EQ(*result->forward_heading, 0.0);
}

TEST(PathGeometry, LookaheadClampsShortPathAndHasNoDirectionForCoincidentPoints)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(0.1, 0.0)};
  const auto result = lookAheadOnPath(path, *projectOntoPath(makePose(0.0, 0.0), path), 0.6);
  ASSERT_TRUE(result);
  expectPoint(result->point, 0.1, 0.0);
  EXPECT_DOUBLE_EQ(result->distance, 0.1);
  const std::vector<Pose2D> repeated(3, makePose(2.0, 3.0, 1.0));
  const auto single = lookAheadOnPath(
    repeated, *projectOntoPath(makePose(2.0, 3.0), repeated), 0.6);
  ASSERT_TRUE(single);
  EXPECT_FALSE(single->forward_heading);
  EXPECT_DOUBLE_EQ(single->distance, 0.0);
  EXPECT_FALSE(lookAheadOnPath({}, PathProjection{}, 0.6));
}

TEST(PathGeometry, LookaheadValidatesLengthsAndProjection)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(1.0, 0.0)};
  const auto projection = *projectOntoPath(makePose(0.0, 0.0), path);
  for (double bad : {-1.0, std::numeric_limits<double>::infinity(), std::nan("")}) {
    EXPECT_THROW(lookAheadOnPath(path, projection, bad), std::invalid_argument);
  }
  auto invalid = projection;
  invalid.segment_index = 9;
  EXPECT_THROW(lookAheadOnPath(path, invalid, 0.6), std::invalid_argument);
  invalid = projection;
  invalid.t = 1.1;
  EXPECT_THROW(lookAheadOnPath(path, invalid, 0.6), std::invalid_argument);
}


TEST(PathGeometry, LookaheadReportsActualShortArcForVeryLargeRequestedDistance)
{
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(0.1, 0.0)};
  const auto projection = *projectOntoPath(makePose(0.0, 0.0), path);
  const auto result = lookAheadOnPath(path, projection, 1e30);
  ASSERT_TRUE(result);
  expectPoint(result->point, 0.1, 0.0);
  EXPECT_DOUBLE_EQ(result->distance, 0.1);
}

TEST(PathGeometry, LookaheadAgreesWithExtractionForVerticesDuplicatesAndZeroLength)
{
  const std::vector<Pose2D> path{
    makePose(0.0, 0.0), makePose(0.0, 0.0), makePose(0.4, 0.0),
    makePose(0.4, 0.0), makePose(0.4, 0.8), makePose(0.4, 0.8)};
  for (const auto & robot : {makePose(-1.0, 0.0), makePose(0.2, 0.1),
      makePose(0.4, 0.0), makePose(0.4, 2.0)})
  {
    const auto projection = *projectOntoPath(robot, path);
    for (double length : {0.0, 0.1, 0.6, 2.0}) {
      const auto result = lookAheadOnPath(path, projection, length);
      const auto extracted = extractForwardPath(path, projection, length);
      ASSERT_TRUE(result);
      ASSERT_FALSE(extracted.empty());
      expectPoint(result->point, extracted.back().x, extracted.back().y);
    }
  }
}

}  // 匿名命名空间
}  // 命名空间 my_dwa_controller
