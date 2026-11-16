int32 order
int64 traj_id
builtin_interfaces/Time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt

# Identity of the accepted goal; zero only for non-navigation legacy tools.
uint64 goal_id
