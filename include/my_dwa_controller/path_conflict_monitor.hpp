#ifndef MY_DWA_CONTROLLER__PATH_CONFLICT_MONITOR_HPP_
#define MY_DWA_CONTROLLER__PATH_CONFLICT_MONITOR_HPP_

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "my_dwa_controller/types.hpp"
#include "my_dwa_controller/scan_obstacle_tracker.hpp"

namespace my_dwa_controller
{

struct PathConflictConfig
{
  double lookahead = 1.5;
  double robot_radius = 0.22;  // Padded circumscribed radius, supplied by Controller.
  double outer_margin = 0.33;
};

enum class ConflictMapState {First, Unchanged, Changed, Rebased, Invalid};

struct ConflictRegion
{
  double start = std::numeric_limits<double>::infinity();
  double end = 0.0;
};

struct ScanPathRelation
{
  ScanObstacleTrack track;
  double distance = 0.0, along = 0.0;
  double first_hit = std::numeric_limits<double>::infinity();
  bool approaching = false, conflict = false;
};

struct ScanRiskArea
{
  Pose2D center;  // Fixed crossing point; theta is the original path tangent.
  double half_length = 0.0, corridor_width = 0.0;
  double lateral_min = 0.0, lateral_max = 0.0;
};

struct ConflictEvidence
{
  ConflictMapState map_state = ConflictMapState::Invalid;
  std::size_t map_state_id = 0;
  bool new_map_state = false;
  bool corridor_known = false;
  double known_inner_distance = 0.0;  // Observed prefix; unknown touching it is excluded.
  bool path_aligned = false;
  bool history_comparable = false;
  bool inner_conflict = false;
  bool outer_risk = false;
  std::size_t inner_core_cells = 0;
  std::size_t outer_core_cells = 0;
  std::size_t new_core_cells = 0;
  ConflictRegion inner, outer;
  bool scan_enabled = false, scan_fresh = false, scan_unresolved = false;
  bool scan_new_state = false, scan_inner_conflict = false;
  std::size_t scan_sequence = 0;
  std::vector<ScanPathRelation> scan_relations;
  bool scan_area_active = false, scan_area_clear = false, scan_area_known = false;
  bool scan_area_rays_clear = false, scan_area_map_clear = false;
  ScanRiskArea scan_area;
};

/// All inputs share the Costmap world frame; no TF, locking or ROS clocks here.
/// observation_time is snapshot read time, NOT the internal map update time.
class PathConflictMonitor
{
public:
  explicit PathConflictMonitor(PathConflictConfig config = {});
  ConflictEvidence observe(
    const std::vector<Pose2D> & local_path, const Pose2D & robot,
    const Velocity2D & velocity, const nav2_costmap_2d::Costmap2D & snapshot,
    double observation_time, const ScanTrackSnapshot * scan = nullptr);
  void reset();
  void releaseScanRisk();

private:
  const nav2_costmap_2d::Costmap2D * updateHistory(
    const nav2_costmap_2d::Costmap2D & snapshot, double observation_time,
    ConflictEvidence & evidence);
  void scanCorridor(
    const std::vector<Pose2D> & path, const nav2_costmap_2d::Costmap2D & snapshot,
    const nav2_costmap_2d::Costmap2D * reference, ConflictEvidence & evidence);
  void mergeScan(
    const std::vector<Pose2D> & path, const ScanTrackSnapshot & scan,
    const nav2_costmap_2d::Costmap2D & snapshot, ConflictEvidence & evidence);
  void inspectScanRisk(
    const std::vector<Pose2D> & path, const ScanTrackSnapshot & scan,
    const nav2_costmap_2d::Costmap2D & snapshot, ConflictEvidence & evidence);
  void resetHistory();
  std::optional<ScanRiskArea> scan_risk_;
  std::size_t last_scan_sequence_ = 0;
  PathConflictConfig config_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> latest_, previous_, risk_reference_;
  double risk_reference_time_ = 0.0;
  double latest_time_ = 0.0, previous_time_ = 0.0, last_observation_time_ = 0.0;
  std::size_t state_id_ = 0;
};

}  // namespace my_dwa_controller
#endif  // MY_DWA_CONTROLLER__PATH_CONFLICT_MONITOR_HPP_
