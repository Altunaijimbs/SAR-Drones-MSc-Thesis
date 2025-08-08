# LLMCommand.msg
std_msgs/Header header
string command_id
string action  # MOVE_TO, HOVER, LAND, SEARCH, AVOID, INVESTIGATE, RTL
geometry_msgs/Point target_position
float32 target_yaw  # in degrees
float32 speed  # m/s
string reasoning  # LLM's explanation for the command
string safety_priority  # HIGH, MEDIUM, LOW
float32 confidence  # 0.0 to 1.0
string[] preconditions
string[] expected_outcomes

# Optional search pattern for SEARCH action
drone_interfaces/SearchPattern search_pattern
