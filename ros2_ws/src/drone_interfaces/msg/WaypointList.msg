# WaypointList.msg
std_msgs/Header header
string mission_id
drone_interfaces/Waypoint[] waypoints
bool is_cyclic  # whether to repeat the mission
