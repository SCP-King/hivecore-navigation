# Refreshed admission certificate. Source spline and goal identity never retimestamped.
std_msgs/Header header
uint64 goal_id
int64 revision
Bspline spline
geometry_msgs/Point goal
geometry_msgs/Pose[] samples
float64 max_speed
float64 max_accel
float64 environment_margin
float64[] chassis_geometry
geometry_msgs/Point[] footprint
sensor_msgs/PointCloud2 obstacles
float64[3] ground_plane
builtin_interfaces/Time scene_stamp
bool valid
string reason
float64 goal_yaw
bool rotation_observed
float64[4] window_bounds
# A station already within arrival tolerance needs scene/model for final alignment,
# not an invented nonzero motion spline.
bool hold_position
geometry_msgs/Point scene_center
float64 scene_radius
float64 max_steering_rate
float64 max_wheel_rate
string map_id
builtin_interfaces/Time rotation_stamp
# Scene/model feedback may remain valid when a new spline is rejected.
# This updates collision evidence for an existing segment; never authorizes the rejected spline.
bool scene_valid
