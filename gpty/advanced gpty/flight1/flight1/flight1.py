import airsim
import numpy as np
import time

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Function to check obstacle ahead

def obstacle_ahead(lidar_points, threshold=5.0):
    points = np.array(lidar_points, dtype=np.float32).reshape(-1, 3)
    # Considering points in front of drone within a 120-degree cone
    points_ahead = points[points[:,0] > 0]
    distance = np.linalg.norm(points_ahead, axis=1)
    return np.any(distance < threshold)

# Take off
takeoff_height = -5
client.takeoffAsync().join()
client.moveToZAsync(takeoff_height, 3).join()

# Define initial parameters
target_x, target_y, target_z = 10, 0, takeoff_height

while True:
    # Move forward in small increments to continuously check for obstacles
    position = client.getMultirotorState().kinematics_estimated.position
    current_x, current_y, current_z = position.x_val, position.y_val, position.z_val

    if current_x >= target_x:
        print("Target reached")
        break

    # Check LiDAR data for obstacles
    lidar_data = client.getLidarData()
    if len(lidar_data.point_cloud) > 0 and obstacle_ahead(lidar_data.point_cloud):
        print("Obstacle detected! Increasing altitude...")
        current_z -= 1  # increase altitude by 1 meter (z-axis negative downwards)
        client.moveToZAsync(current_z, 2).join()
    else:
        # Move forward by 1 meter increments
        next_x = min(current_x + 1, target_x)
        client.moveToPositionAsync(next_x, target_y, current_z, 2).join()

    time.sleep(0.2)

# Hover for a moment
client.hoverAsync().join()
time.sleep(3)

# Land safely
client.landAsync().join()

# Release drone
client.armDisarm(False)
client.enableApiControl(False)
print("Mission completed")
