cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneState.msg"
  "msg/Waypoint.msg"
  "msg/WaypointList.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/MissionStatus.msg"
  "msg/SceneDescription.msg"
  "msg/DetectedObject.msg"
  "msg/LLMCommand.msg"
  "msg/SearchPattern.msg"
  "srv/SetMission.srv"
  "srv/ArmDrone.srv"
  "srv/TakeoffLand.srv"
  "srv/EmergencyStop.srv"
  "srv/GetSceneDescription.srv"
  "action/NavigateToGoal.action"
  "action/ExecuteMission.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
