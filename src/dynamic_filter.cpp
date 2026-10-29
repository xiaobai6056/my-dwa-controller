#include "my_dwa_controller/dynamic_filter.hpp"

namespace my_dwa_controller
{
PreparedFilter DynamicFilter::prepare(
  const std::vector<const Trajectory *> & trajectories, const ScanSnapshot * scan,
  const std::string & frame, const rclcpp::Time & now) const
{
  PreparedFilter result;
  result.input = validation_.prepare(trajectories, scan, frame, now);
  if (!result.input.input_available) {return result;}
  for (std::size_t index = 0; index < scan->obstacles.size(); ++index) {
    if (!scan->obstacles[index].velocity_valid) {
      const auto & points = scan->obstacles[index].points;
      result.unknown_points.insert(result.unknown_points.end(), points.begin(), points.end());
      continue;
    }
    ScanSnapshot one;
    one.stamp = scan->stamp;
    one.frame_id = scan->frame_id;
    one.obstacles.push_back(scan->obstacles[index]);
    auto screened = geometry_.prepareScan(trajectories, &one, frame, now);
    if (!screened) {result.input.input_available = false; return result;}
    result.clusters.emplace_back(index, std::move(*screened));
  }
  return result;
}

FilterResult DynamicFilter::evaluate(
  const Trajectory & trajectory, const PreparedFilter & prepared) const
{
  FilterResult result;
  if (!prepared.input.input_available) {return result;}
  result.available = true;
  for (const auto & cluster : prepared.clusters) {
    const auto value = geometry_.evaluate(
      trajectory, &cluster.second, cluster.second.frame_id, prepared.input.now);
    if (!value.valid) {return {};}
    if (value.min_clearance &&
      (!result.min_clearance || *value.min_clearance < *result.min_clearance))
    {
      result.min_clearance = value.min_clearance;
    }
    if (value.collision) {result.conflicts.push_back({cluster.first, *value.min_clearance});}
  }
  return result;
}
bool DynamicFilter::observedGeometryCollision(
  const Trajectory & trajectory, const PreparedFilter & prepared) const
{
  return prepared.input.input_available &&
         geometry_.collidesWithObservedPoints(trajectory, prepared.unknown_points);
}
}  // namespace my_dwa_controller
