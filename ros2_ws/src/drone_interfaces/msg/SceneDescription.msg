# SceneDescription.msg
std_msgs/Header header
string general_description
drone_interfaces/DetectedObject[] detected_objects
string[] spatial_relationships
string safety_assessment
float32 visibility_score  # 0.0 to 1.0
string weather_conditions
string lighting_conditions
string[] anomalies
