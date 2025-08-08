# Waypoint.msg
int32 waypoint_id
float64 latitude
float64 longitude
float32 altitude_relative
float32 altitude_msl
float32 speed  # desired speed to this waypoint (m/s)
float32 yaw    # desired yaw in degrees (-180 to 180)
bool use_yaw   # whether to use the yaw value
int32 hold_time  # time to hold at waypoint in seconds
string command  # WAYPOINT, TAKEOFF, LAND, LOITER
