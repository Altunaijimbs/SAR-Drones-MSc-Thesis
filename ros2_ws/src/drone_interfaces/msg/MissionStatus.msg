# MissionStatus.msg
std_msgs/Header header
string mission_id
string status  # IDLE, EXECUTING, PAUSED, COMPLETED, FAILED, ABORTED
int32 total_waypoints
int32 current_waypoint_index
float32 progress_percentage
string current_action
string failure_reason  # if status is FAILED
geometry_msgs/Point next_waypoint_position
float32 distance_to_next_waypoint
float32 eta_seconds  # estimated time to next waypoint
