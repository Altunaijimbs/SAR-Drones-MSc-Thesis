import airsim
import numpy as np
import time
import random

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, vehicle_name="PX4")
client.armDisarm(True, vehicle_name="PX4")

# Parameters
initial_z = -20  # takeoff height (NED, so negative = up)
vertical_speed = 2
forward_speed = 4
loop_rate = 0.12  # 10 Hz
safe_landing_z = -0.99  # corresponds to z=100 in NED frame (ground check threshold)
obstacle_fov_deg = 60
obstacle_forward_dist = 5.0

# Takeoff
client.takeoffAsync(vehicle_name="PX4").join()
client.moveToZAsync(initial_z, vertical_speed, vehicle_name="PX4").join()

# Generate random target coordinates within a defined range
random_x = random.uniform(50, 200)
random_y = random.uniform(-100, 100)
print(f"Random waypoint generated: x={random_x:.2f}, y={random_y:.2f}")

# Function to check for ground below using LiDAR
def clear_ground_below(lidar_points, ground_threshold=2.0):
    points = np.array(lidar_points, dtype=np.float32).reshape(-1, 3)
    if points.size == 0:
        return False

    # Points within 1 meter in XY plane and Z must be significantly below sensor
    ground_points = points[(points[:, 0]**2 + points[:, 1]**2) < 1.0]
    if len(ground_points) == 0:
        return False

    # Check if any point is far enough below the sensor to be the ground
    return np.any(ground_points[:, 2] > ground_threshold)


# Function to detect forward obstacles ignoring Z direction
def obstacle_ahead(lidar_points, forward_dist=5.0, fov_deg=60):
    points = np.array(lidar_points, dtype=np.float32).reshape(-1, 3)
    if points.size == 0:
        return False

    forward_points = points[points[:, 0] > 0]  # Points in front of drone
    angles = np.degrees(np.arctan2(forward_points[:, 1], forward_points[:, 0]))
    cone_points = forward_points[np.abs(angles) < (fov_deg / 2)]
    cone_points[:, 2] = 0  # Ignore Z in distance calculation
    distances = np.linalg.norm(cone_points, axis=1)
    return np.any(distances < forward_dist)

# Navigate to waypoint and find valid landing spot
current_z = initial_z
try:
    while True:
        state = client.getMultirotorState(vehicle_name="PX4")
        pos = state.kinematics_estimated.position

        if state.landed_state == airsim.LandedState.Landed:
            print("Drone landed unexpectedly. Aborting mission.")
            break

        dx = random_x - pos.x_val
        dy = random_y - pos.y_val
        dz = current_z - pos.z_val
        dist = np.sqrt(dx**2 + dy**2 + dz**2)

        lidar = client.getLidarData(lidar_name="Lidar1", vehicle_name="PX4")

        if dist < 1.0:
            if clear_ground_below(lidar.point_cloud):
                print("Safe ground detected. Initiating landing.")
                break
            else:
                print("Ground not safe. Ascending and shifting forward.")
                current_z -= 1
                random_x += 5

        elif obstacle_ahead(lidar.point_cloud, forward_dist=obstacle_forward_dist, fov_deg=obstacle_fov_deg):
            print("Obstacle detected ahead! Increasing altitude.")
            current_z -= 1

        client.moveToPositionAsync(random_x, random_y, current_z, forward_speed, vehicle_name="PX4")
        time.sleep(loop_rate)

    client.hoverAsync(vehicle_name="PX4").join()
    time.sleep(1)
    client.landAsync(vehicle_name="PX4").join()

finally:
    client.armDisarm(False, vehicle_name="PX4")
    client.enableApiControl(False, vehicle_name="PX4")
    print("Mission complete.")
