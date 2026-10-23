// Offline evidence replay only. No ROS node, Costmap reconstruction or robot commands.
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "nav2_costmap_2d/footprint.hpp"
#include "my_dwa_controller/risk_governor.hpp"

int main(int argc, char ** argv)
{
  if (argc != 11) {return 2;}
  my_dwa_controller::RiskGovernorConfig config;
  auto footprint = nav2_costmap_2d::makeFootprintFromRadius(std::stod(argv[1]));
  nav2_costmap_2d::padFootprint(footprint, std::stod(argv[2]));
  double inscribed;
  nav2_costmap_2d::calculateMinAndMaxDistances(footprint, inscribed, config.robot_radius);
  config.brake_acceleration = std::stod(argv[3]);
  config.reaction_time = std::stod(argv[4]);
  config.stop_margin = std::stod(argv[5]);
  config.control_dt = std::stod(argv[6]);
  config.stopped_velocity = std::stod(argv[7]);
  config.angular_acceleration = std::stod(argv[8]);
  config.clear_states = std::stoul(argv[9]);
  config.warning_speed = std::stod(argv[10]);
  std::cerr << std::setprecision(17) << "padded_radius=" << config.robot_radius << '\n';
  my_dwa_controller::RiskGovernor governor(config);
  std::cout << "line,stamp,state,cap,stop,available,allow_wait,cannot_stop\n";
  std::size_t line, map_id;
  double stamp, v, w, maximum, prefix;
  int kind, fresh, known, aligned, inner, outer;
  std::string region_start;
  while (std::cin >> line >> stamp >> map_id >> kind >> fresh >> known >> aligned >>
    inner >> outer >> region_start >> prefix >> v >> w >> maximum)
  {
    my_dwa_controller::ConflictEvidence evidence;
    evidence.map_state_id = map_id;
    evidence.map_state = static_cast<my_dwa_controller::ConflictMapState>(kind);
    evidence.new_map_state = fresh;
    evidence.corridor_known = known;
    evidence.path_aligned = aligned;
    evidence.inner_conflict = inner;
    evidence.outer_risk = outer;
    evidence.inner.start = std::stod(region_start);
    evidence.known_inner_distance = prefix;
    const auto result = governor.decide(evidence, {v, w}, maximum);
    std::cout << std::setprecision(17) << line << ',' << stamp << ',' <<
      my_dwa_controller::riskStateName(result.state) << ',' << result.max_linear_speed << ',' <<
      result.stopping_distance << ',' << result.available_distance << ',' << result.allow_wait <<
      ',' << result.cannot_stop << '\n';
  }
}
