# DroneState.msg
std_msgs/Header header
string drone_id
string mode  # MANUAL, GUIDED, AUTO, LAND, RTL
bool armed
bool connected
float32 battery_percentage
float32 battery_voltage
geometry_msgs/Pose current_pose
geometry_msgs/Twist current_velocity
float32 altitude_relative
float32 altitude_msl
float32 heading_deg
sensor_msgs/NavSatFix gps_fix
