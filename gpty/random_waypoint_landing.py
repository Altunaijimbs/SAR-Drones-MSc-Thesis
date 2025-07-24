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
initial_z = -5  # takeoff height (NED, so negative = up)
vertical_speed = 2
forward_speed = 4
loop_rate = 0.1  # 10 Hz
safe_landing_z = -100  # corresponds to z=100 in NED frame (ground check threshold)

# Takeoff
client.takeoffAsync(vehicle_name="PX4").join()
client.moveToZAsync(initial_z, vertical_speed, vehicle_name="PX4").join()

# Generate random target coordinates within a defined range
random_x = random.uniform(5, 20)
random_y = random.uniform(-10, 10)
print(f"Random waypoint generated: x={random_x:.2f}, y={random_y:.2f}")

# Function to check for ground below using LiDAR
def clear_ground_below(lidar_points):
    points = np.array(lidar_points, dtype=np.float32).reshape(-1, 3)
    if points.size == 0:
        return False

    ground_points = points[(points[:, 0]**2 + points[:, 1]**2) < 1.0]  # points roughly below
    if len(ground_points) == 0:
        return False

    # Check if any point is near ground threshold
    return np.any(ground_points[:, 2] > safe_landing_z)

# Navigate to waypoint and find valid landing spot
current_z = initial_z
try:
    while True:
        state = client.getMultirotorState(vehicle_name="PX4")
        pos = state.kinematics_estimated.position

        if state.landed_state == airsim.LandedState.Landed:
            print("Drone landed unexpectedly. Aborting mission.")
            break

        # Check distance to target
        dx = random_x - pos.x_val
        dy = random_y - pos.y_val
        dz = current_z - pos.z_val
        dist = np.sqrt(dx**2 + dy**2 + dz**2)

        if dist < 1.0:
            # At target position, check if ground is safe for landing
            lidar = client.getLidarData(lidar_name="Lidar1", vehicle_name="PX4")
            if clear_ground_below(lidar.point_cloud):
                print("Safe ground detected. Initiating landing.")
                break
            else:
                print("Ground not safe. Ascending and shifting forward.")
                current_z -= 1  # ascend
                random_x += 5   # move forward

        # Continuously stream position setpoints
        client.moveToPositionAsync(random_x, random_y, current_z, forward_speed, vehicle_name="PX4")
        time.sleep(loop_rate)

    # Land if clear
    client.hoverAsync(vehicle_name="PX4").join()
    time.sleep(1)
    client.landAsync(vehicle_name="PX4").join()

finally:
    client.armDisarm(False, vehicle_name="PX4")
    client.enableApiControl(False, vehicle_name="PX4")
    print("Mission complete.")
