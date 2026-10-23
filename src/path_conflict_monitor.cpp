#include "my_dwa_controller/path_conflict_monitor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "my_dwa_controller/path_geometry.hpp"

namespace my_dwa_controller
{
namespace
{
constexpr double kHistorySeconds = 1.0;
constexpr std::size_t kExpansionCells = 2;
using Map = nav2_costmap_2d::Costmap2D;

bool alignedLattice(const Map & a, const Map & b)
{
  const double r = a.getResolution();
  if (r != b.getResolution()) {return false;}
  for (double delta : {a.getOriginX() - b.getOriginX(), a.getOriginY() - b.getOriginY()}) {
    if (std::abs(delta / r - std::round(delta / r)) > 1e-6) {return false;}
  }
  return true;
}

bool sameGeometry(const Map & a, const Map & b)
{
  return a.getOriginX() == b.getOriginX() && a.getOriginY() == b.getOriginY() &&
         a.getSizeInCellsX() == b.getSizeInCellsX() &&
         a.getSizeInCellsY() == b.getSizeInCellsY();
}

// Match overlapping world cells, never array indices from different rolling origins.
bool changedOverlap(const Map & a, const Map & b)
{
  for (unsigned int y = 0; y < a.getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < a.getSizeInCellsX(); ++x) {
      double wx, wy;
      a.mapToWorld(x, y, wx, wy);
      unsigned int bx, by;
      if (b.worldToMap(wx, wy, bx, by) && a.getCost(x, y) != b.getCost(bx, by)) {
        return true;
      }
    }
  }
  return false;
}

struct CoreStats
{
  std::size_t count = 0, added = 0;
  double lateral = 0.0, longitudinal = 0.0;
  void add(double distance, double along)
  {
    ++count;
    lateral += distance;
    longitudinal += along;
  }
};

struct CoreComparison
{
  std::array<CoreStats, 2> before{}, current{};

  void addCell(
    const Map * reference, const Pose2D & cell, unsigned char cost,
    std::size_t side, double distance, double along)
  {
    unsigned int x, y;
    if (!reference || !reference->worldToMap(cell.x, cell.y, x, y)) {return;}
    const auto old = reference->getCost(x, y);
    if (old == nav2_costmap_2d::NO_INFORMATION || cost == nav2_costmap_2d::NO_INFORMATION) {
      return;
    }
    if (old == nav2_costmap_2d::LETHAL_OBSTACLE) {before[side].add(distance, along);}
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      current[side].add(distance, along);
      if (old != nav2_costmap_2d::LETHAL_OBSTACLE) {++current[side].added;}
    }
  }

  bool inward(double resolution) const
  {
    for (std::size_t side = 0; side < 2; ++side) {
      const auto & old = before[side];
      const auto & now = current[side];
      if (old.count < kExpansionCells || now.count < kExpansionCells ||
        now.added < kExpansionCells) {continue;}
      const double lateral = old.lateral / old.count - now.lateral / now.count;
      const double longitudinal = std::abs(
        old.longitudinal / old.count - now.longitudinal / now.count);
      if (lateral >= resolution && lateral >= longitudinal) {return true;}
    }
    return false;
  }

  std::size_t added() const {return current[0].added + current[1].added;}
};

void extend(ConflictRegion & region, double first, double last, double cell_radius)
{
  region.start = std::min(region.start, std::max(0.0, first - cell_radius));
  region.end = std::max(region.end, last + cell_radius);
}

struct CorridorHits
{
  double inner = std::numeric_limits<double>::infinity();
  double outer = std::numeric_limits<double>::infinity();
  double inner_end = 0.0, outer_end = 0.0;
};

// A cell can intersect several nearby path legs. Safety boundaries use the
// earliest intersection, while lateral motion statistics use nearest projection.
CorridorHits corridorHits(
  const Pose2D & cell, const std::vector<Pose2D> & path, const std::vector<double> & arc,
  double inner_width, double outer_width)
{
  CorridorHits hits;
  std::vector<Pose2D> segment(2);
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    segment[0] = path[i];
    segment[1] = path[i + 1];
    const auto p = *projectOntoPath(cell, segment);
    const double dot = (cell.x - p.point.x) * (segment[1].x - segment[0].x) +
      (cell.y - p.point.y) * (segment[1].y - segment[0].y);
    if ((i == 0 && p.t == 0.0 && dot < 0.0) ||
      (i + 2 == path.size() && p.t == 1.0 && dot > 0.0)) {continue;}
    const double along = arc[i] + p.t * (arc[i + 1] - arc[i]);
    if (p.squared_distance <= outer_width * outer_width) {
      hits.outer = std::min(hits.outer, along);
      hits.outer_end = std::max(hits.outer_end, along);
    }
    if (p.squared_distance <= inner_width * inner_width) {
      hits.inner = std::min(hits.inner, along);
      hits.inner_end = std::max(hits.inner_end, along);
    }
  }
  return hits;
}
}  // namespace

PathConflictMonitor::PathConflictMonitor(PathConflictConfig config)
: config_(config)
{
  for (double positive : {config.lookahead, config.robot_radius, config.outer_margin}) {
    if (!std::isfinite(positive) || positive <= 0.0) {
      throw std::invalid_argument("path conflict length/radius/outer margin must be positive");
    }
  }
}

ConflictEvidence PathConflictMonitor::observe(
  const std::vector<Pose2D> & local_path, const Pose2D & robot, const Velocity2D & velocity,
  const Map & snapshot, double observation_time, const ScanTrackSnapshot * scan)
{
  ConflictEvidence e;
  if (!std::isfinite(observation_time) || snapshot.getSizeInCellsX() == 0 ||
    snapshot.getSizeInCellsY() == 0 || snapshot.getResolution() <= 0.0)
  {
    resetHistory();
    return e;
  }
  if (observation_time < last_observation_time_) {reset();}
  const auto projection = projectOntoPath(robot, local_path);
  if (!projection) {return e;}
  const auto path = extractForwardPath(local_path, *projection, config_.lookahead);
  if (path.size() < 2) {return e;}
  const auto heading = lookAheadOnPath(path, PathProjection{}, 0.0)->forward_heading;
  e.path_aligned = std::sqrt(projection->squared_distance) <= config_.robot_radius &&
    heading && std::cos(*heading - robot.theta) >= 0.5 && (scan || velocity.v >= 0.0);

  const auto previous_scan_sequence = last_scan_sequence_;
  const auto * reference = updateHistory(snapshot, observation_time, e);
  scanCorridor(path, snapshot, reference, e);
  if (scan) {
    last_scan_sequence_ = previous_scan_sequence;
    mergeScan(path, *scan, snapshot, e);
  }
  return e;
}

const Map * PathConflictMonitor::updateHistory(
  const Map & snapshot, double observation_time, ConflictEvidence & e)
{
  bool copy_latest = false;
  if (!latest_ || !alignedLattice(snapshot, *latest_)) {
    resetHistory();
    e.map_state = ConflictMapState::First;
    e.new_map_state = true;
    copy_latest = true;
  } else if (changedOverlap(snapshot, *latest_)) {
    previous_ = std::move(latest_);
    previous_time_ = latest_time_;
    e.map_state = ConflictMapState::Changed;
    e.new_map_state = true;
    copy_latest = true;
  } else {
    copy_latest = !sameGeometry(snapshot, *latest_);
    e.map_state = copy_latest ? ConflictMapState::Rebased : ConflictMapState::Unchanged;
  }
  if (copy_latest) {latest_ = std::make_unique<Map>(snapshot);}
  if (e.new_map_state) {++state_id_;}
  e.map_state_id = state_id_;
  latest_time_ = observation_time;
  last_observation_time_ = observation_time;
  const bool adjacent_valid = previous_ && alignedLattice(snapshot, *previous_) &&
    observation_time - previous_time_ <= kHistorySeconds;
  if (risk_reference_ && observation_time - risk_reference_time_ > kHistorySeconds) {
    risk_reference_.reset();
  }
  e.history_comparable = adjacent_valid || static_cast<bool>(risk_reference_);

  return adjacent_valid ? previous_.get() : nullptr;
}

void PathConflictMonitor::scanCorridor(
  const std::vector<Pose2D> & path, const Map & snapshot,
  const Map * reference, ConflictEvidence & e)
{
  std::vector<double> arc(path.size(), 0.0);
  double min_x = path.front().x, max_x = min_x, min_y = path.front().y, max_y = min_y;
  for (std::size_t i = 0; i < path.size(); ++i) {
    min_x = std::min(min_x, path[i].x); max_x = std::max(max_x, path[i].x);
    min_y = std::min(min_y, path[i].y); max_y = std::max(max_y, path[i].y);
    if (i > 0) {
      arc[i] = arc[i - 1] + std::hypot(
        path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    }
  }
  const double cell_radius = snapshot.getResolution() * std::sqrt(0.5);
  const double inner_width = config_.robot_radius + cell_radius;
  const double outer_width = inner_width + config_.outer_margin;
  e.corridor_known = true;
  e.known_inner_distance = arc.back();
  int x0, y0, x1, y1;
  snapshot.worldToMapNoBounds(min_x - outer_width, min_y - outer_width, x0, y0);
  snapshot.worldToMapNoBounds(max_x + outer_width, max_y + outer_width, x1, y1);
  // Humble truncates negative indices toward zero; include the lower virtual cell.
  --x0;
  --y0;
  CoreComparison adjacent, retained;
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      Pose2D cell;
      const bool in_map = x >= 0 && y >= 0 &&
        x < static_cast<int>(snapshot.getSizeInCellsX()) &&
        y < static_cast<int>(snapshot.getSizeInCellsY());
      if (in_map) {
        snapshot.mapToWorld(x, y, cell.x, cell.y);
      } else {
        // Virtual cell centers let the SAME corridor mask detect missing coverage,
        // without adding round end caps or clamping out-of-map cells to an edge.
        cell.x = snapshot.getOriginX() + (x + 0.5) * snapshot.getResolution();
        cell.y = snapshot.getOriginY() + (y + 0.5) * snapshot.getResolution();
      }
      const auto cost = in_map ? snapshot.getCost(x, y) : nav2_costmap_2d::NO_INFORMATION;
      if (cost == nav2_costmap_2d::NO_INFORMATION || cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        const auto hits = corridorHits(cell, path, arc, inner_width, outer_width);
        if (cost == nav2_costmap_2d::NO_INFORMATION) {
          if (std::isfinite(hits.outer)) {e.corridor_known = false;}
          if (std::isfinite(hits.inner)) {
            e.known_inner_distance = std::min(
              e.known_inner_distance, std::max(0.0, hits.inner - cell_radius));
          }
          continue;
        }
        if (std::isfinite(hits.outer)) {
          ++e.outer_core_cells;
          extend(e.outer, hits.outer, hits.outer_end, cell_radius);
        }
        if (std::isfinite(hits.inner)) {
          ++e.inner_core_cells;
          extend(e.inner, hits.inner, hits.inner_end, cell_radius);
        }
      }
      const auto p = *projectOntoPath(cell, path);
      const auto & a = path[p.segment_index];
      const auto & b = path[p.segment_index + 1];
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double tangent_dot = (cell.x - p.point.x) * dx + (cell.y - p.point.y) * dy;
      if ((p.segment_index == 0 && p.t == 0.0 && tangent_dot < 0.0) ||
        (p.segment_index + 2 == path.size() && p.t == 1.0 && tangent_dot > 0.0))
      {
        continue;
      }
      const double distance = std::sqrt(p.squared_distance);
      if (distance > outer_width) {continue;}
      const double along = arc[p.segment_index] + p.t * (arc[p.segment_index + 1] -
        arc[p.segment_index]);
      const std::size_t side = dx * (cell.y - p.point.y) - dy * (cell.x - p.point.x) < 0 ? 0 : 1;
      adjacent.addCell(reference, cell, cost, side, distance, along);
      retained.addCell(risk_reference_.get(), cell, cost, side, distance, along);
    }
  }
  e.inner_conflict = e.inner_core_cells > 0;
  const bool new_inward = adjacent.inward(snapshot.getResolution());
  e.outer_risk = new_inward || retained.inward(snapshot.getResolution());
  e.new_core_cells = std::max(adjacent.added(), retained.added());
  if (new_inward && e.new_map_state) {
    // Preserve only a reference that actually produced positive evidence. Adjacent
    // states still detect new entrants; remote changes cannot erase an old warning.
    risk_reference_ = std::make_unique<Map>(*previous_);
    risk_reference_time_ = previous_time_;
  }
}

void PathConflictMonitor::mergeScan(
  const std::vector<Pose2D> & path, const ScanTrackSnapshot & scan,
  const Map & snapshot, ConflictEvidence & e)
{
  e.scan_enabled = true;
  e.scan_fresh = scan.fresh;
  e.scan_sequence = scan.sequence;
  e.scan_new_state = scan.sequence != last_scan_sequence_;
  last_scan_sequence_ = scan.sequence;
  e.outer_risk = false;  // Scan replaces only the old Costmap motion heuristic.
  std::vector<double> arc(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    arc[i] = arc[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  for (const auto & track : scan.tracks) {
    ScanPathRelation relation;
    relation.track = track;
    Pose2D position;
    position.x = track.x; position.y = track.y;
    const auto p = *projectOntoPath(position, path);
    relation.distance = std::sqrt(p.squared_distance);
    relation.along = arc[p.segment_index] + p.t *
      (arc[p.segment_index + 1] - arc[p.segment_index]);
    const double heading = std::atan2(
      path[p.segment_index + 1].y - path[p.segment_index].y,
      path[p.segment_index + 1].x - path[p.segment_index].x);
    const double nx = -std::sin(heading), ny = std::cos(heading);
    const double lateral = (position.x - p.point.x) * nx + (position.y - p.point.y) * ny;
    const double lateral_speed = track.vx * nx + track.vy * ny;
    relation.approaching = lateral * lateral_speed < -.005;
    const double near_width = config_.robot_radius + config_.outer_margin +
      track.radius + std::abs(lateral_speed);
    const bool forward = relation.along > 0.0 && relation.distance <= near_width;
    if (scan.fresh && track.observed && track.motion == ScanMotion::Moving &&
      std::abs(lateral_speed) >= .08 && forward)
    {
      // A short transverse trend is sufficient. Do not depend on a persistent
      // object center or interpret along-wall visible-support drift as crossing.
      for (int step = 0; step <= 20; ++step) {
        const double dt = step * .1;
        Pose2D predicted = position;
        predicted.x += dt * track.vx; predicted.y += dt * track.vy;
        if (*distanceToPath(predicted, path) > config_.robot_radius + track.radius + .05) {
          continue;
        }
        if (!relation.approaching && std::abs(lateral) > config_.robot_radius + track.radius) {
          continue;
        }
        relation.conflict = true;
        relation.first_hit = dt;
        e.scan_inner_conflict = true;
        if (!scan_risk_) {
          ScanRiskArea area;
          area.center = p.point; area.center.theta = heading;
          const double tx = std::cos(heading), ty = std::sin(heading);
          const double half_along = track.half_x > 0 || track.half_y > 0 ?
            std::abs(tx) * track.half_x + std::abs(ty) * track.half_y : track.radius;
          const double half_side = track.half_x > 0 || track.half_y > 0 ?
            std::abs(nx) * track.half_x + std::abs(ny) * track.half_y : track.radius;
          // Enclose the accepted two-second longitudinal sweep as well as the
          // observed support. A later lost trend cannot clear the wrong strip.
          const double sweep_half = track.vx * tx + track.vy * ty;
          area.center.x += sweep_half * tx; area.center.y += sweep_half * ty;
          area.half_length = half_along + std::abs(sweep_half) + .05;
          area.corridor_width = config_.robot_radius + .05;
          // Include the original approach side, not only the currently empty path.
          area.lateral_min = std::min(-area.corridor_width, lateral - half_side - .05);
          area.lateral_max = std::max(area.corridor_width, lateral + half_side + .05);
          scan_risk_ = area;
        }
        break;
      }
    }
    e.scan_relations.push_back(relation);
  }
  inspectScanRisk(path, scan, snapshot, e);
}

namespace
{
bool rayClears(const Pose2D & point, const std::vector<ScanRay> & rays, double tolerance)
{
  for (const auto & ray : rays) {
    const double dx = ray.end.x - ray.origin.x, dy = ray.end.y - ray.origin.y;
    const double length = std::hypot(dx, dy);
    if (!std::isfinite(length) || length <= 0.0) {continue;}
    const double px = point.x - ray.origin.x, py = point.y - ray.origin.y;
    const double along = (px * dx + py * dy) / length;
    if (along < ray.min_range || along + .04 >= length) {continue;}
    if (std::abs(px * dy - py * dx) / length <= tolerance) {return true;}
  }
  return false;
}
}  // namespace

void PathConflictMonitor::inspectScanRisk(
  const std::vector<Pose2D> & path, const ScanTrackSnapshot & scan,
  const Map & map, ConflictEvidence & e)
{
  if (!scan_risk_) {return;}
  const auto & area = *scan_risk_;
  e.scan_area_active = true;
  e.scan_area = area;
  e.scan_area_known = true;
  e.scan_area_map_clear = true;
  e.scan_area_rays_clear = scan.fresh;
  const double resolution = map.getResolution();
  const int length_steps = std::max(
    1, static_cast<int>(std::ceil(
      2 * area.half_length / resolution)));
  const int side_steps = std::max(
    1,
    static_cast<int>(std::ceil((area.lateral_max - area.lateral_min) / resolution)));
  const double c = std::cos(area.center.theta), s = std::sin(area.center.theta);
  for (int i = 0; i <= length_steps; ++i) {
    const double along = -area.half_length + 2 * area.half_length * i / length_steps;
    for (int j = 0; j <= side_steps; ++j) {
      const double side = area.lateral_min + (area.lateral_max - area.lateral_min) * j / side_steps;
      Pose2D point;
      point.x = area.center.x + c * along - s * side;
      point.y = area.center.y + s * along + c * side;
      if (e.scan_area_rays_clear && !rayClears(point, scan.rays, resolution * .5)) {
        e.scan_area_rays_clear = false;
      }
    }
  }
  // Enumerate every intersecting Costmap cell, including virtual out-of-map
  // cells. A rotated sampling lattice can skip occupied/unknown interior cells.
  const double extent_x = std::abs(c) * area.half_length + std::abs(s) * area.corridor_width;
  const double extent_y = std::abs(s) * area.half_length + std::abs(c) * area.corridor_width;
  const int min_x = std::floor((area.center.x - extent_x - map.getOriginX()) / resolution);
  const int max_x = std::floor((area.center.x + extent_x - map.getOriginX()) / resolution);
  const int min_y = std::floor((area.center.y - extent_y - map.getOriginY()) / resolution);
  const int max_y = std::floor((area.center.y + extent_y - map.getOriginY()) / resolution);
  const double cell_extent = resolution * .5 * (std::abs(c) + std::abs(s));
  for (int x = min_x; x <= max_x; ++x) {
    for (int y = min_y; y <= max_y; ++y) {
      const double dx = map.getOriginX() + (x + .5) * resolution - area.center.x;
      const double dy = map.getOriginY() + (y + .5) * resolution - area.center.y;
      if (std::abs(c * dx + s * dy) > area.half_length + cell_extent ||
        std::abs(-s * dx + c * dy) > area.corridor_width + cell_extent) {continue;}
      if (x < 0 || y < 0 || x >= static_cast<int>(map.getSizeInCellsX()) ||
        y >= static_cast<int>(map.getSizeInCellsY()) ||
        map.getCost(x, y) == nav2_costmap_2d::NO_INFORMATION)
      {
        e.scan_area_known = false;
        e.scan_area_map_clear = false;
      } else if (map.getCost(x, y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        e.scan_area_map_clear = false;
      }
    }
  }
  e.scan_area_clear = e.scan_area_rays_clear && e.scan_area_known &&
    e.scan_area_map_clear && !e.scan_inner_conflict;
  e.scan_unresolved = !e.scan_area_clear;
  if (e.scan_area_clear) {return;}
  // Hold the original parking boundary through surface switches, lost IDs and
  // same-goal replans. Only fresh geometric clearance can lift this protection.
  const auto p = *projectOntoPath(area.center, path);
  double along = 0.0;
  for (std::size_t i = 0; i < p.segment_index; ++i) {
    along += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
  }
  along += p.t * std::hypot(
    path[p.segment_index + 1].x - path[p.segment_index].x,
    path[p.segment_index + 1].y - path[p.segment_index].y);
  e.inner_conflict = true;
  e.inner.start = std::min(e.inner.start, std::max(0.0, along - area.half_length));
  e.inner.end = std::max(e.inner.end, along + area.half_length);
}

void PathConflictMonitor::releaseScanRisk()
{
  scan_risk_.reset();
}

void PathConflictMonitor::reset()
{
  resetHistory();
  last_scan_sequence_ = 0;
  releaseScanRisk();
}

void PathConflictMonitor::resetHistory()
{
  latest_.reset();
  previous_.reset();
  risk_reference_.reset();
  risk_reference_time_ = 0.0;
  latest_time_ = previous_time_ = last_observation_time_ = 0.0;
  state_id_ = 0;
}
}  // namespace my_dwa_controller
