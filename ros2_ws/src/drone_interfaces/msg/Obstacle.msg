# Obstacle.msg
int32 obstacle_id
string type  # STATIC, DYNAMIC, UNKNOWN
geometry_msgs/Point position
geometry_msgs/Vector3 dimensions  # x=length, y=width, z=height
geometry_msgs/Twist velocity
float32 confidence  # 0.0 to 1.0
