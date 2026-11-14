# Position is the center of the circular occupancy in the enclosing array frame.
string track_id
geometry_msgs/Point position
float64 occupancy_radius
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 direct_velocity
geometry_msgs/Vector3 diagnostic_velocity_0_8s
bool position_valid
bool velocity_valid
bool direct_velocity_valid
bool diagnostic_velocity_valid
# Stamp 0 and age -1 s indicate that this SORT ID has never had a LiDAR measurement.
builtin_interfaces/Time last_observation_stamp
builtin_interfaces/Duration observation_age
