#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "nav2_costmap_2d/cost_values.hpp"
#include "my_dwa_controller/path_conflict_monitor.hpp"

namespace my_dwa_controller
{
namespace
{
Pose2D point(double x, double y)
{
  Pose2D p;
  p.x = x;
  p.y = y;
  return p;
}

nav2_costmap_2d::Costmap2D map(double origin = -1.0)
{
  return nav2_costmap_2d::Costmap2D(80, 80, 0.05, origin, -2.0, 0);
}

void core(nav2_costmap_2d::Costmap2D & grid, double x, double y, unsigned char cost = 254)
{
  unsigned int mx, my;
  ASSERT_TRUE(grid.worldToMap(x, y, mx, my));
  grid.setCost(mx, my, cost);
}

void block(nav2_costmap_2d::Costmap2D & grid, double y)
{
  core(grid, 0.825, y);
  core(grid, 0.875, y);
  core(grid, 0.925, y);
}

const std::vector<Pose2D> path{point(0.0, 0.0), point(2.0, 0.0)};
const Velocity2D speed{0.26, 0.0};

TEST(PathConflictMonitor, InnerCoreHasConservativeArcInterval)
{
  PathConflictMonitor monitor;
  auto grid = map();
  block(grid, 0.1);
  const auto e = monitor.observe(path, Pose2D{}, speed, grid, 1.0);
  EXPECT_TRUE(e.inner_conflict);
  EXPECT_EQ(e.inner_core_cells, 3u);
  EXPECT_LT(e.inner.start, 0.83);
  EXPECT_GT(e.inner.end, 0.90);
  EXPECT_TRUE(e.corridor_known);
  EXPECT_TRUE(e.new_map_state);
}

TEST(PathConflictMonitor, StaticWallIsPresenceNotDynamicRiskEvenWhenRobotAndPathMove)
{
  PathConflictMonitor monitor;
  auto grid = map();
  for (double x = 0.1; x < 2.0; x += 0.05) {
    core(grid, x, 0.4);
  }
  monitor.observe(path, Pose2D{}, speed, grid, 1.0);
  for (int i = 1; i <= 10; ++i) {
    const auto e = monitor.observe(
      {point(0.01 * i, 0.02 * i), point(2.0, 0.02 * i)},
      point(0.01 * i, 0.02 * i), speed, grid, 1.0 + 0.05 * i);
    EXPECT_GT(e.outer_core_cells, 0u);
    EXPECT_FALSE(e.outer_risk);
    EXPECT_FALSE(e.new_map_state);
  }
}

TEST(PathConflictMonitor, DetectsInwardCoreExpansionAndDoesNotCountControlDuplicates)
{
  PathConflictMonitor monitor;
  auto grid = map();
  block(grid, 0.45);
  monitor.observe(path, Pose2D{}, speed, grid, 1.0);
  block(grid, 0.25);  // Entering front; removed tail need not be inside this corridor.
  const auto e = monitor.observe(path, Pose2D{}, speed, grid, 1.2);
  EXPECT_TRUE(e.outer_risk);
  EXPECT_GE(e.new_core_cells, 2u);
  EXPECT_TRUE(e.history_comparable);
  EXPECT_TRUE(e.new_map_state);
  const auto duplicate = monitor.observe(path, Pose2D{}, speed, grid, 1.25);
  EXPECT_TRUE(duplicate.outer_risk);  // Hold existing evidence, don't count it twice.
  EXPECT_FALSE(duplicate.new_map_state);
  EXPECT_EQ(duplicate.map_state_id, e.map_state_id);
}

TEST(PathConflictMonitor, InflationAloneNeverBecomesCoreMotion)
{
  PathConflictMonitor monitor;
  auto grid = map();
  block(grid, 0.45);
  monitor.observe(path, Pose2D{}, speed, grid, 1.0);
  block(grid, 0.1);
  for (unsigned int i = 0; i < grid.getSizeInCellsX() * grid.getSizeInCellsY(); ++i) {
    if (grid.getCharMap()[i] == 254 && (i / grid.getSizeInCellsX()) < 45) {
      grid.getCharMap()[i] = 253;
    }
  }
  const auto e = monitor.observe(path, Pose2D{}, speed, grid, 1.2);
  EXPECT_FALSE(e.inner_conflict);
  EXPECT_FALSE(e.outer_risk);
}

TEST(PathConflictMonitor, RollingWindowNewCoverageAndUnknownToCoreAreNotMotion)
{
  PathConflictMonitor monitor;
  auto old = map(-3.0);
  block(old, 0.45);
  monitor.observe(path, Pose2D{}, speed, old, 1.0);
  auto shifted = map(-2.0);
  block(shifted, 0.45);
  core(shifted, 1.2, 0.1);  // Previously outside x<1 coverage.
  const auto e = monitor.observe(path, Pose2D{}, speed, shifted, 1.2);
  EXPECT_FALSE(e.outer_risk);
  EXPECT_FALSE(e.new_map_state);  // Origin shift is not an observed occupancy change.
  EXPECT_EQ(e.map_state, ConflictMapState::Rebased);

  monitor.reset();
  auto unknown = map();
  block(unknown, 0.45);
  block(unknown, 0.1);
  core(unknown, 0.825, 0.1, 255);
  core(unknown, 0.875, 0.1, 255);
  core(unknown, 0.925, 0.1, 255);
  monitor.observe(path, Pose2D{}, speed, unknown, 2.0);
  block(unknown, 0.1);
  EXPECT_FALSE(monitor.observe(path, Pose2D{}, speed, unknown, 2.2).outer_risk);
}

TEST(PathConflictMonitor, UnknownOrOutOfMapCorridorCannotConfirmClear)
{
  PathConflictMonitor monitor;
  auto grid = map();
  core(grid, 0.7, 0.0, 255);
  EXPECT_FALSE(monitor.observe(path, Pose2D{}, speed, grid, 1.0).corridor_known);
  auto short_map = map(-3.0);
  EXPECT_FALSE(monitor.observe(path, Pose2D{}, speed, short_map, 1.2).corridor_known);
}

TEST(PathConflictMonitor, OldHistoryClockRollbackAndResolutionChangeDoNotImplyMotion)
{
  for (int mode : {0, 1, 2}) {
    PathConflictMonitor monitor;
    auto grid = map();
    block(grid, 0.45);
    monitor.observe(path, Pose2D{}, speed, grid, 2.0);
    if (mode == 2) {grid.resizeMap(40, 40, 0.1, -1.0, -2.0);}
    block(grid, 0.1);
    const auto e = monitor.observe(path, Pose2D{}, speed, grid, mode == 1 ? 1.0 : 4.0);
    EXPECT_FALSE(e.outer_risk);
    EXPECT_FALSE(e.history_comparable);
  }
}

TEST(PathConflictMonitor, RejectsInvalidConfigurationAndReportsDegeneratePath)
{
  PathConflictConfig config;
  config.lookahead = -1.0;
  EXPECT_THROW(PathConflictMonitor{config}, std::invalid_argument);
  PathConflictMonitor monitor;
  const auto e = monitor.observe({point(0, 0)}, Pose2D{}, speed, map(), 1.0);
  EXPECT_EQ(e.map_state, ConflictMapState::Invalid);
  EXPECT_FALSE(e.corridor_known);
}
TEST(PathConflictMonitor, FlatCorridorFitsThreeMetreWindowWithoutArtificialEndCaps)
{
  PathConflictMonitor monitor;
  nav2_costmap_2d::Costmap2D grid(60, 60, 0.05, -1.5, -1.5, 0);
  const auto e = monitor.observe(path, Pose2D{}, speed, grid, 1.0);
  EXPECT_TRUE(e.corridor_known);
}

TEST(PathConflictMonitor, RemoteCostChangesDoNotEraseRecentInwardEvidence)
{
  PathConflictMonitor monitor;
  auto grid = map();
  block(grid, 0.525);
  monitor.observe(path, Pose2D{}, speed, grid, 1.0);
  block(grid, 0.325);
  auto e = monitor.observe(path, Pose2D{}, speed, grid, 1.2);
  ASSERT_TRUE(e.outer_risk);
  ASSERT_FALSE(e.inner_conflict);
  for (int i = 1; i <= 3; ++i) {
    core(grid, -0.8, -0.8, static_cast<unsigned char>(i));
    e = monitor.observe(path, Pose2D{}, speed, grid, 1.2 + 0.1 * i);
    EXPECT_TRUE(e.outer_risk);
    EXPECT_TRUE(e.new_map_state);
  }
  EXPECT_FALSE(monitor.observe(path, Pose2D{}, speed, grid, 2.3).outer_risk);
}

TEST(PathConflictMonitor, PartialLowerEdgeCellCannotConfirmCoverage)
{
  PathConflictMonitor monitor;
  const std::vector<Pose2D> edge{point(0, -1.45), point(2, -1.45)};
  EXPECT_FALSE(monitor.observe(edge, point(0, -1.45), speed, map(), 1.0).corridor_known);
}

TEST(PathConflictMonitor, RemoteChangeBeforeNewObstacleDoesNotHideSubsequentExpansion)
{
  PathConflictMonitor monitor;
  auto grid = map();
  monitor.observe(path, Pose2D{}, speed, grid, 1.0);
  core(grid, -0.8, -0.8, 1);
  monitor.observe(path, Pose2D{}, speed, grid, 1.1);
  block(grid, 0.525);
  monitor.observe(path, Pose2D{}, speed, grid, 1.2);
  block(grid, 0.325);
  EXPECT_TRUE(monitor.observe(path, Pose2D{}, speed, grid, 1.4).outer_risk);
}

TEST(PathConflictMonitor, InnerPrefixDistinguishesUnknownBeforeAfterAndOutsideConflict)
{
  for (int location = 0; location < 3; ++location) {
    PathConflictMonitor monitor;
    auto grid = map();
    block(grid, 0.025);
    core(grid, location == 1 ? 1.225 : 0.425, location == 2 ? 0.425 : 0.025, 255);
    const auto e = monitor.observe(path, Pose2D{}, speed, grid, 1.0);
    ASSERT_TRUE(e.inner_conflict);
    EXPECT_FALSE(e.corridor_known);
    if (location == 0) {
      EXPECT_LT(e.known_inner_distance, e.inner.start);
    } else {
      EXPECT_GT(e.known_inner_distance, e.inner.start);
    }
  }
}

TEST(PathConflictMonitor, FoldedPathUsesEarliestIntersectedInnerSegmentForUnknownAndCore)
{
  PathConflictMonitor monitor;
  const std::vector<Pose2D> folded{
    point(0, 0), point(0.55, 0), point(0.55, 0.30), point(0, 0.30)};
  auto grid = map();
  core(grid, 0.475, 0.025);
  core(grid, 0.225, 0.175, 255);
  auto e = monitor.observe(folded, Pose2D{}, speed, grid, 1.0);
  ASSERT_TRUE(e.inner_conflict);
  EXPECT_LT(e.known_inner_distance, 0.23);
  EXPECT_LT(e.known_inner_distance, e.inner.start);
  grid = map();
  core(grid, 0.225, 0.175);  // Also overlaps early leg, although nearer the returning leg.
  e = monitor.observe(folded, Pose2D{}, speed, grid, 1.2);
  EXPECT_LT(e.inner.start, 0.23);
  EXPECT_GT(e.inner.end, 1.0);  // Same cell also intersects the later returning leg.
}
}  // namespace
}  // namespace my_dwa_controller

namespace my_dwa_controller
{
TEST(ScanPathConflict, PredictsCrossingAndMergesIntoGovernorEvidence)
{
  PathConflictMonitor monitor;
  nav2_costmap_2d::Costmap2D map(100, 100, .05, -2, -2, 0);
  ScanTrackSnapshot scan;
  scan.fresh = true;
  scan.sequence = 1;
  ScanObstacleTrack track;
  track.id = 1; track.x = 1.2; track.y = -.9;
  track.vy = .25; track.radius = .25;
  track.confidence = .95; track.observed = true;
  track.motion = ScanMotion::Moving;
  scan.tracks.push_back(track);
  const auto e = monitor.observe(path, Pose2D{}, {.2, 0}, map, 1, &scan);
  EXPECT_TRUE(e.scan_inner_conflict);
  EXPECT_TRUE(e.inner_conflict);
  ASSERT_EQ(e.scan_relations.size(), 1u);
  EXPECT_TRUE(e.scan_relations[0].approaching);
  EXPECT_LT(e.scan_relations[0].first_hit, 2.0);
  EXPECT_NEAR(e.inner.start, .9, .1);
  EXPECT_TRUE(e.scan_new_state);
  EXPECT_FALSE(
    monitor.observe(
      path, Pose2D{}, {.2, 0}, map, 1.1,
      &scan).scan_new_state);
}

TEST(ScanPathConflict, StaticClustersDoNotBecomeMotionRiskAndCostmapStillBlocks)
{
  PathConflictMonitor monitor;
  nav2_costmap_2d::Costmap2D map(100, 100, .05, -2, -2, 0);
  ScanTrackSnapshot scan;
  scan.fresh = true;
  scan.sequence = 1;
  ScanObstacleTrack track;
  track.x = 1.2; track.y = -.5; track.radius = .25;
  track.confidence = 1; track.observed = true;
  scan.tracks.push_back(track);
  auto e = monitor.observe(path, Pose2D{}, {.2, 0}, map, 1, &scan);
  EXPECT_FALSE(e.outer_risk);
  EXPECT_FALSE(e.inner_conflict);
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(1, 0, x, y));
  map.setCost(x, y, 254);
  e = monitor.observe(path, Pose2D{}, {.2, 0}, map, 1.2, &scan);
  EXPECT_TRUE(e.inner_conflict);
  EXPECT_FALSE(e.scan_inner_conflict);
}

TEST(ScanPathConflict, ConflictLocationDoesNotDependOnStraightPathSegmentation)
{
  PathConflictMonitor sparse_monitor, dense_monitor;
  nav2_costmap_2d::Costmap2D grid(100, 100, .05, -2, -2, 0);
  ScanTrackSnapshot scan;
  scan.fresh = true; scan.sequence = 1;
  ScanObstacleTrack track;
  track.x = 1.2; track.y = -.9; track.vy = .25;
  track.radius = .25; track.confidence = .95; track.observed = true;
  track.motion = ScanMotion::Moving;
  scan.tracks.push_back(track);
  std::vector<Pose2D> dense;
  for (int i = 0; i <= 40; ++i) {
    dense.push_back(point(.05 * i, 0));
  }
  const auto sparse = sparse_monitor.observe(path, Pose2D{}, {.2, 0}, grid, 1, &scan);
  const auto sampled = dense_monitor.observe(dense, Pose2D{}, {.2, 0}, grid, 1, &scan);
  EXPECT_NEAR(sparse.inner.start, sampled.inner.start, 1e-8);
}
}  // namespace my_dwa_controller

namespace my_dwa_controller
{
namespace
{
ScanTrackSnapshot emergencyScan(std::size_t seq, bool incoming)
{
  ScanTrackSnapshot s;
  s.sequence = seq; s.fresh = true;
  if (incoming) {
    ScanObstacleTrack t;
    t.id = seq; t.x = 1.2; t.y = -.8; t.vy = .25;
    t.radius = .2; t.confidence = .95; t.observed = true;
    t.motion = ScanMotion::Moving;
    s.tracks.push_back(t);
  }
  return s;
}

void freeRays(ScanTrackSnapshot & s)
{
  for (int i = 0; i < 720; ++i) {
    const double a = i * 2 * std::acos(-1.) / 720;
    ScanRay ray;
    ray.end = point(3 * std::cos(a), 3 * std::sin(a));
    ray.min_range = .1;
    s.rays.push_back(ray);
  }
}

TEST(ScanEmergency, LongitudinalMotionAndUnknownSurfaceCannotCreateRisk)
{
  PathConflictMonitor monitor;
  auto grid = map();
  auto s = emergencyScan(1, true);
  s.tracks[0].vx = .25; s.tracks[0].vy = 0;
  s.tracks[0].y = -.4;
  EXPECT_FALSE(monitor.observe(path, Pose2D{}, speed, grid, 1, &s).scan_inner_conflict);
  s.tracks[0].motion = ScanMotion::Unknown;
  s.tracks[0].vx = 0; s.tracks[0].vy = .25;
  EXPECT_FALSE(monitor.observe(path, Pose2D{}, speed, grid, 1.2, &s).scan_inner_conflict);
}

TEST(ScanEmergency, MissingIdentityHoldsFrozenAreaUntilSweptRegionIsObservedClear)
{
  PathConflictMonitor monitor;
  auto grid = map();
  auto s = emergencyScan(1, true);
  auto e = monitor.observe(path, Pose2D{}, speed, grid, 1, &s);
  ASSERT_TRUE(e.scan_area_active);
  const double original = e.inner.start;
  s = emergencyScan(2, false);  // Old ID gone; an empty packet is not free-space proof.
  e = monitor.observe(path, Pose2D{}, speed, grid, 1.2, &s);
  EXPECT_TRUE(e.inner_conflict);
  EXPECT_FALSE(e.scan_area_clear);
  EXPECT_NEAR(e.inner.start, original, 1e-9);
  freeRays(s);
  e = monitor.observe(path, Pose2D{}, speed, grid, 1.4, &s);
  EXPECT_TRUE(e.scan_area_clear);
  EXPECT_FALSE(e.inner_conflict);
  // Current forward corridor need not be complete when original risk patch is.
  EXPECT_TRUE(e.scan_area_known);
  monitor.releaseScanRisk();
  e = monitor.observe(path, Pose2D{}, speed, grid, 1.6, &s);
  EXPECT_FALSE(e.scan_area_active);
}

TEST(ScanEmergency, OccludedApproachBandUnknownMapAndStaleScanCannotClear)
{
  for (int failure = 0; failure < 3; ++failure) {
    PathConflictMonitor monitor;
    auto grid = map();
    auto incoming = emergencyScan(1, true);
    monitor.observe(path, Pose2D{}, speed, grid, 1, &incoming);
    auto s = emergencyScan(2, false);
    freeRays(s);
    if (failure == 0) {
      // Cross-path itself is visible; the original lower approach band is occluded.
      for (auto & ray : s.rays) {
        if (ray.end.y < -.3) {
          ray.end.x *= .2; ray.end.y *= .2;
        }
      }
    }
    if (failure == 1) {core(grid, 1.2, 0, 255);}
    if (failure == 2) {s.fresh = false;}
    const auto e = monitor.observe(path, Pose2D{}, speed, grid, 1.2, &s);
    EXPECT_FALSE(e.scan_area_clear);
    EXPECT_TRUE(e.inner_conflict);
  }
}
}  // namespace
}  // namespace my_dwa_controller

namespace my_dwa_controller
{
TEST(ScanEmergency, ApproachStripUsesObservedTransverseExtentNotDiagonalRadius)
{
  PathConflictMonitor monitor;
  auto grid = map();
  auto s = emergencyScan(1, true);
  s.tracks[0].y = -.9;
  s.tracks[0].radius = std::sqrt(.2 * .2 + .2 * .2);
  s.tracks[0].half_x = s.tracks[0].half_y = .2;
  const auto e = monitor.observe(path, Pose2D{}, speed, grid, 1, &s);
  ASSERT_TRUE(e.scan_area_active);
  EXPECT_NEAR(e.scan_area.lateral_min, -1.15, 1e-8);
}
}  // namespace my_dwa_controller

namespace my_dwa_controller
{
TEST(ScanEmergency, RotatedLocalCorridorCannotSkipUnknownOrOccupiedCells)
{
  for (const unsigned char cost : {255, 253}) {
    PathConflictMonitor monitor;
    auto grid = map();
    const double a = std::acos(-1.) / 4;
    std::vector<Pose2D> rotated{point(0, -1), point(2, 1)};
    auto s = emergencyScan(1, true);
    s.tracks[0].x = 1 + .8 * std::sin(a);
    s.tracks[0].y = -.8 * std::cos(a);
    s.tracks[0].vx = -.25 * std::sin(a);
    s.tracks[0].vy = .25 * std::cos(a);
    Pose2D robot = point(0, -1); robot.theta = a;
    ASSERT_TRUE(monitor.observe(rotated, robot, speed, grid, 1, &s).scan_area_active);
    s = emergencyScan(2, false); freeRays(s);
    // Interior map cell missed by the old rotated sample lattice.
    core(grid, .875, -.125, cost);
    const auto e = monitor.observe(rotated, robot, speed, grid, 1.2, &s);
    EXPECT_FALSE(e.scan_area_map_clear);
    EXPECT_FALSE(e.scan_area_clear);
    if (cost == 255) {EXPECT_FALSE(e.scan_area_known);}
  }
}

TEST(ScanEmergency, FixedAreaIncludesAcceptedLongitudinalForecast)
{
  PathConflictMonitor monitor;
  auto s = emergencyScan(1, true);
  s.tracks[0].vx = .15;
  const auto e = monitor.observe(path, Pose2D{}, speed, map(), 1, &s);
  ASSERT_TRUE(e.scan_inner_conflict);
  EXPECT_GE(e.scan_area.center.x + e.scan_area.half_length, 1.2 + .15 * 2 + .2);
}
}  // namespace my_dwa_controller

namespace my_dwa_controller
{
TEST(ScanEmergency, GeometricAlignmentSurvivesStoppedOdometryJitterOnlyInScanMode)
{
  PathConflictMonitor monitor;
  auto s = emergencyScan(1, true);
  EXPECT_TRUE(monitor.observe(path, Pose2D{}, {-.0003, 0}, map(), 1, &s).path_aligned);
  EXPECT_FALSE(monitor.observe(path, Pose2D{}, {-.0003, 0}, map(), 1.2).path_aligned);
}
}  // namespace my_dwa_controller
