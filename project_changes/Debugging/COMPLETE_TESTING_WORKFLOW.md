Errors in provided COMPLETE_TESTING_WORKFLOW.MD 



1) WRONG AIRSIM NODE LAUNCH SEQUENCE:

mbs@mbs:~$ cd ~/Desktop/airsim/Cosys-AirSim/ros2
source install/setup.bash
ros2 launch airsim_ros_wrapper airsim_node.launch.py
Package 'airsim_ros_wrapper' not found: "package 'airsim_ros_wrapper' not found, searching: ['/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/search_patterns', '/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/llm_controller', '/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/drone_vision_interpreter', '/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/drone_interfaces', '/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/airsim_interfaces', '/home/mbs/px4_ros2_ws/install/px4_simple_avoid', '/home/mbs/px4_ros2_ws/install/px4_ros_com', '/home/mbs/px4_ros2_ws/install/px4_msgs', '/home/mbs/Desktop/airsim/Cosys-AirSim/ros2/install/airsim_ros_pkgs', '/home/mbs/Desktop/airsim/Cosys-AirSim/ros2/install/airsim_interfaces', '/opt/ros/humble']"

----------------

correct launch command is:
cd /home/mbs/Desktop/airsim/Cosys-AirSim/ros2
source install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py

2)MAVROS BRIDGE ERROR:

mbs@mbs:~$ source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch.py fcu_url:="udp://:14540@127.0.0.1:14557"
file 'px4.launch.py' was not found in the share directory of package 'mavros' which is at '/opt/ros/humble/share/mavros'
------------------------
TRIED FOLLOWING AND IT WORKED:
mbs@mbs:~$ ros2 run mavros mavros_node --ros-args \
  --param fcu_url:="udp://:14550@127.0.0.1:14540" \
  --param target_system_id:=1 \
  --param target_component_id:=1

WITH THE FOLLOWING OUTPUT:
[INFO] [1754109923.789810734] [mavros_node]: Starting mavros_node container
[INFO] [1754109923.789881009] [mavros_node]: FCU URL: udp://:14550@127.0.0.1:14540
[INFO] [1754109923.789890980] [mavros_node]: GCS URL: 
[INFO] [1754109923.789898242] [mavros_node]: UAS Prefix: /uas1
[INFO] [1754109923.789904982] [mavros_node]: Starting mavros router node
[INFO] [1754109923.794173646] [mavros_router]: Built-in SIMD instructions: SSE, SSE2
[INFO] [1754109923.794198990] [mavros_router]: Built-in MAVLink package version: 2025.6.6
[INFO] [1754109923.794211327] [mavros_router]: Known MAVLink dialects: common ardupilotmega ASLUAV AVSSUAS all csAirLink cubepilot development icarous loweheiser matrixpilot paparazzi standard storm32 uAvionix ualberta
[INFO] [1754109923.794219842] [mavros_router]: MAVROS Router started
[INFO] [1754109923.794241128] [mavros_router]: Requested to add endpoint: type: 0, url: udp://:14550@127.0.0.1:14540
[INFO] [1754109923.794261895] [mavros_router]: Endpoint link[1000] created
[INFO] [1754109923.794545151] [mavros_router]: link[1000] opened successfully
[INFO] [1754109923.794577617] [mavros_router]: Requested to add endpoint: type: 2, url: /uas1
[INFO] [1754109923.794592497] [mavros_router]: Endpoint link[1001] created
[INFO] [1754109923.796432236] [mavros_router]: link[1001] opened successfully
[INFO] [1754109923.796461174] [mavros_node]: Starting mavros uas node
[INFO] [1754109923.797777538] [mavros_router]: link[1000] detected remote address 1.1
[INFO] [1754109923.835552334] [mavros]: UAS Executor started, threads: 16
[INFO] [1754109923.847138273] [mavros]: Plugin actuator_control created
[INFO] [1754109923.847169634] [mavros]: Plugin actuator_control initialized
[INFO] [1754109923.859248366] [mavros]: Plugin adsb created
[INFO] [1754109923.859275797] [mavros]: Plugin adsb initialized
[INFO] [1754109923.860684504] [mavros]: Plugin altitude created
[INFO] [1754109923.860706139] [mavros]: Plugin altitude initialized
[INFO] [1754109923.862288455] [mavros]: Plugin cam_imu_sync created
[INFO] [1754109923.862336356] [mavros]: Plugin cam_imu_sync initialized
[INFO] [1754109923.863650965] [mavros]: Plugin camera created
[INFO] [1754109923.863668325] [mavros]: Plugin camera initialized
[INFO] [1754109923.866260773] [mavros]: Plugin cellular_status created
[INFO] [1754109923.866275097] [mavros]: Plugin cellular_status initialized
[INFO] [1754109923.869686330] [mavros]: Plugin command created
[INFO] [1754109923.869716153] [mavros]: Plugin command initialized
[INFO] [1754109923.871259006] [mavros]: Plugin companion_process_status created
[INFO] [1754109923.871273999] [mavros]: Plugin companion_process_status initialized
[INFO] [1754109923.873051132] [mavros]: Plugin debug_value created
[INFO] [1754109923.873074308] [mavros]: Plugin debug_value initialized
[INFO] [1754109923.875291557] [mavros.distance_sensor]: DS: Plugin not configured!
[INFO] [1754109923.875358913] [mavros]: Plugin distance_sensor created
[INFO] [1754109923.875379452] [mavros]: Plugin distance_sensor initialized
[INFO] [1754109923.879194338] [mavros]: Plugin esc_status created
[INFO] [1754109923.879281702] [mavros]: Plugin esc_status initialized
[INFO] [1754109923.882662425] [mavros]: Plugin esc_telemetry created
[INFO] [1754109923.882694623] [mavros]: Plugin esc_telemetry initialized
[INFO] [1754109923.885351257] [mavros]: Plugin fake_gps created
[INFO] [1754109923.885374513] [mavros]: Plugin fake_gps initialized
[INFO] [1754109923.891038334] [mavros]: Plugin ftp created
[INFO] [1754109923.891078484] [mavros]: Plugin ftp initialized
[INFO] [1754109923.896886521] [mavros]: Plugin geofence created
[INFO] [1754109923.896941102] [mavros]: Plugin geofence initialized
[INFO] [1754109923.905643981] [mavros]: Plugin gimbal_control created
[INFO] [1754109923.905692932] [mavros]: Plugin gimbal_control initialized
[INFO] [1754109923.915592742] [mavros]: Plugin global_position created
[INFO] [1754109923.915650729] [mavros]: Plugin global_position initialized
[INFO] [1754109923.920669377] [mavros]: Plugin gps_input created
[INFO] [1754109923.920707929] [mavros]: Plugin gps_input initialized
[INFO] [1754109923.925892147] [mavros]: Plugin gps_rtk created
[INFO] [1754109923.925922906] [mavros]: Plugin gps_rtk initialized
[INFO] [1754109923.931285115] [mavros]: Plugin gps_status created
[INFO] [1754109923.931329929] [mavros]: Plugin gps_status initialized
[INFO] [1754109923.934849420] [mavros]: Plugin guided_target created
[INFO] [1754109923.934875392] [mavros]: Plugin guided_target initialized
[INFO] [1754109923.938372777] [mavros]: Plugin hil created
[INFO] [1754109923.938400960] [mavros]: Plugin hil initialized
[INFO] [1754109923.941373536] [mavros]: Plugin home_position created
[INFO] [1754109923.941401848] [mavros]: Plugin home_position initialized
[INFO] [1754109923.945101119] [mavros]: Plugin imu created
[INFO] [1754109923.945138048] [mavros]: Plugin imu initialized
[INFO] [1754109923.948963930] [mavros]: Plugin landing_target created
[INFO] [1754109923.948990340] [mavros]: Plugin landing_target initialized
[INFO] [1754109923.952937964] [mavros]: Plugin local_position created
[INFO] [1754109923.952968050] [mavros]: Plugin local_position initialized
[INFO] [1754109923.957013457] [mavros]: Plugin log_transfer created
[INFO] [1754109923.957046912] [mavros]: Plugin log_transfer initialized
[INFO] [1754109923.959975514] [mavros]: Plugin mag_calibration_status created
[INFO] [1754109923.960001072] [mavros]: Plugin mag_calibration_status initialized
[INFO] [1754109923.963467471] [mavros]: Plugin manual_control created
[INFO] [1754109923.963498014] [mavros]: Plugin manual_control initialized
[INFO] [1754109923.966787399] [mavros]: Plugin mocap_pose_estimate created
[INFO] [1754109923.966809444] [mavros]: Plugin mocap_pose_estimate initialized
[INFO] [1754109923.970776505] [mavros]: Plugin mount_control created
[INFO] [1754109923.970806297] [mavros]: Plugin mount_control initialized
[INFO] [1754109923.974139956] [mavros]: Plugin nav_controller_output created
[INFO] [1754109923.974166889] [mavros]: Plugin nav_controller_output initialized
[INFO] [1754109923.977405077] [mavros]: Plugin obstacle_distance created
[INFO] [1754109923.977431063] [mavros]: Plugin obstacle_distance initialized
[INFO] [1754109923.984071155] [mavros]: Plugin odometry created
[INFO] [1754109923.984119543] [mavros]: Plugin odometry initialized
[INFO] [1754109923.990626950] [mavros]: Plugin onboard_computer_status created
[INFO] [1754109923.990656847] [mavros]: Plugin onboard_computer_status initialized
[INFO] [1754109923.995380411] [mavros]: Plugin open_drone_id created
[INFO] [1754109923.995408632] [mavros]: Plugin open_drone_id initialized
[INFO] [1754109923.999836073] [mavros]: Plugin optical_flow created
[INFO] [1754109923.999867219] [mavros]: Plugin optical_flow initialized
[INFO] [1754109924.004091046] [mavros]: Plugin param created
[INFO] [1754109924.004122032] [mavros]: Plugin param initialized
[INFO] [1754109924.007849384] [mavros]: Plugin play_tune created
[INFO] [1754109924.007876185] [mavros]: Plugin play_tune initialized
[INFO] [1754109924.012317181] [mavros]: Plugin px4flow created
[INFO] [1754109924.012354499] [mavros]: Plugin px4flow initialized
[INFO] [1754109924.017547283] [mavros]: Plugin rallypoint created
[INFO] [1754109924.017588192] [mavros]: Plugin rallypoint initialized
[INFO] [1754109924.023900976] [mavros]: Plugin rangefinder created
[INFO] [1754109924.023949752] [mavros]: Plugin rangefinder initialized
[INFO] [1754109924.030851831] [mavros]: Plugin rc_io created
[INFO] [1754109924.030885916] [mavros]: Plugin rc_io initialized
[INFO] [1754109924.035170490] [mavros]: Plugin setpoint_accel created
[INFO] [1754109924.035198600] [mavros]: Plugin setpoint_accel initialized
[INFO] [1754109924.040492201] [mavros]: Plugin setpoint_attitude created
[INFO] [1754109924.040526881] [mavros]: Plugin setpoint_attitude initialized
[INFO] [1754109924.046441570] [mavros]: Plugin setpoint_position created
[INFO] [1754109924.046470566] [mavros]: Plugin setpoint_position initialized
[INFO] [1754109924.052930525] [mavros]: Plugin setpoint_raw created
[INFO] [1754109924.052966900] [mavros]: Plugin setpoint_raw initialized
[INFO] [1754109924.059280387] [mavros]: Plugin setpoint_trajectory created
[INFO] [1754109924.059308175] [mavros]: Plugin setpoint_trajectory initialized
[INFO] [1754109924.064032175] [mavros]: Plugin setpoint_velocity created
[INFO] [1754109924.064060313] [mavros]: Plugin setpoint_velocity initialized
[INFO] [1754109924.081794816] [mavros]: Plugin sys_status created
[INFO] [1754109924.081859044] [mavros]: Plugin sys_status initialized
[INFO] [1754109924.092794852] [mavros.time]: TM: Timesync mode: MAVLINK
[INFO] [1754109924.096508264] [mavros]: Plugin sys_time created
[INFO] [1754109924.096554573] [mavros]: Plugin sys_time initialized
[INFO] [1754109924.108928987] [mavros]: Plugin tdr_radio created
[INFO] [1754109924.108976613] [mavros]: Plugin tdr_radio initialized
[INFO] [1754109924.121675928] [mavros]: Plugin terrain created
[INFO] [1754109924.121735194] [mavros]: Plugin terrain initialized
[INFO] [1754109924.136152620] [mavros]: Plugin trajectory created
[INFO] [1754109924.136200821] [mavros]: Plugin trajectory initialized
[INFO] [1754109924.149606882] [mavros]: Plugin tunnel created
[INFO] [1754109924.149657160] [mavros]: Plugin tunnel initialized
[INFO] [1754109924.162525461] [mavros]: Plugin vfr_hud created
[INFO] [1754109924.162601990] [mavros]: Plugin vfr_hud initialized
[INFO] [1754109924.175852010] [mavros]: Plugin vibration created
[INFO] [1754109924.175897557] [mavros]: Plugin vibration initialized
[INFO] [1754109924.186619666] [mavros]: Plugin vision_pose created
[INFO] [1754109924.186653419] [mavros]: Plugin vision_pose initialized
[INFO] [1754109924.193439967] [mavros]: Plugin vision_speed created
[INFO] [1754109924.193469720] [mavros]: Plugin vision_speed initialized
[INFO] [1754109924.202061094] [mavros]: Plugin waypoint created
[INFO] [1754109924.202114757] [mavros]: Plugin waypoint initialized
[INFO] [1754109924.210342177] [mavros]: Plugin wheel_odometry created
[INFO] [1754109924.210385320] [mavros]: Plugin wheel_odometry initialized
[INFO] [1754109924.217284198] [mavros]: Plugin wind_estimation created
[INFO] [1754109924.217319544] [mavros]: Plugin wind_estimation initialized
[INFO] [1754109924.218142400] [mavros]: Built-in SIMD instructions: SSE, SSE2
[INFO] [1754109924.218155731] [mavros]: Built-in MAVLink package version: 2025.6.6
[INFO] [1754109924.218158937] [mavros]: Known MAVLink dialects: common ardupilotmega ASLUAV AVSSUAS all csAirLink cubepilot development icarous loweheiser matrixpilot paparazzi standard storm32 uAvionix ualberta
[INFO] [1754109924.218162023] [mavros]: MAVROS UAS via /uas1 started. MY ID 1.191, TARGET ID 1.1
[INFO] [1754109924.231874720] [mavros.imu]: IMU: Attitude quaternion IMU detected!
[INFO] [1754109924.879023437] [mavros]: CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
[INFO] [1754109924.879202932] [mavros.mission]: WP: detected enable_partial_push: 0
[INFO] [1754109924.879508061] [mavros.imu]: IMU: Attitude quaternion IMU detected!
[INFO] [1754109925.074633352] [mavros_router]: link[1001] detected remote address 1.191
[INFO] [1754109925.757802965] [mavros_router]: link[1000] detected remote address 135.1
[INFO] [1754109925.887097051] [mavros.geofence]: GF: Using MISSION_ITEM_INT
[INFO] [1754109925.887139983] [mavros.rallypoint]: RP: Using MISSION_ITEM_INT
[INFO] [1754109925.887152741] [mavros.mission]: WP: Using MISSION_ITEM_INT
[INFO] [1754109925.887164505] [mavros.sys]: VER: 1.1: Capabilities         0x000000000000e8ff
[INFO] [1754109925.887178758] [mavros.sys]: VER: 1.1: Flight software:     010f01ff (93eef0c787000000)
[INFO] [1754109925.887187802] [mavros.sys]: VER: 1.1: Middleware software: 010f01ff (93eef0c787000000)
[INFO] [1754109925.887195771] [mavros.sys]: VER: 1.1: OS software:         060800ff (0000000000000000)
[INFO] [1754109925.887203125] [mavros.sys]: VER: 1.1: Board hardware:      00000001
[INFO] [1754109925.887211144] [mavros.sys]: VER: 1.1: VID/PID:             0000:0000
[INFO] [1754109925.887218598] [mavros.sys]: VER: 1.1: UID:                 4954414c44494e4f
[WARN] [1754109925.890011417] [mavros.cmd]: CMD: Unexpected command 520, result 0
[INFO] [1754109939.880801535] [mavros.mission]: WP: mission received
[INFO] [1754109944.881262340] [mavros.rallypoint]: RP: mission received
[INFO] [1754109949.880944296] [mavros.geofence]: GF: mission received
---------------------------

3)SEARCH PATTERN NODE DIDNT WORK WITH ERROR:

mbs@mbs:~$ cd ~/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 launch search_patterns full_system.launch.py use_lidar:=true use_fusion:=true
file 'full_system.launch.py' was not found in the share directory of package 'search_patterns' which is at '/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/search_patterns/share/search_patterns'
-------------------------------------
