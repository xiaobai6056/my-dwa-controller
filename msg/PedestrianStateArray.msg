# Odom frame. The header stamp is the ROS-clock time at publication.
std_msgs/Header header
# The scan's first-ray stamp. image_stamp is 0 when no SORT frame was paired.
builtin_interfaces/Time scan_stamp
builtin_interfaces/Time image_stamp
uint32 epoch
string status
# The KF position and velocity are a posterior at the last accepted source stamp.
# False preserves legacy difference/OLS selection in the controller.
bool kalman_estimate
my_dwa_controller/PedestrianState[] states
