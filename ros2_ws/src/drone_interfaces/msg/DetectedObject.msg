# DetectedObject.msg
int32 object_id
string label
float32 confidence  # 0.0 to 1.0
geometry_msgs/Point position  # relative to drone
sensor_msgs/RegionOfInterest bounding_box  # in image coordinates
float32 distance  # meters from drone
string color
string size  # small, medium, large
