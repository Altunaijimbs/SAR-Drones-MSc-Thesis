import airsim
import numpy as np
import time

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, vehicle_name="PX4")
client.armDisarm(True, vehicle_name="PX4")

# Parameters
initial_height = -5             # NED (negative up)
forward_speed  = 5              # m/s
vertical_step  = 1              # raise by 1 m on obstacle
loop_hz        = 10             # off‑board heartbeat (≥2 Hz required)
loop_dt        = 1/loop_hz

# Take‑off
client.takeoffAsync(vehicle_name="PX4").join()
client.moveToZAsync(initial_height, 2, vehicle_name="PX4").join()

# --- helper: obstacle check (XY only) ---

def obstacle_ahead(points, dist=5.0, fov=60):
    pts = np.array(points, dtype=np.float32).reshape(-1, 3)
    if pts.size == 0:
        return False
    fwd   = pts[pts[:, 0] > 0]                       # in front
    angle = np.degrees(np.arctan2(fwd[:,1], fwd[:,0]))
    cone  = fwd[np.abs(angle) < fov/2]
    cone[:,2] = 0                                    # ignore Z
    return np.any(np.linalg.norm(cone, axis=1) < dist)

# Mission target
target_x = 10
z_set    = initial_height

try:
    while True:
        state = client.getMultirotorState(vehicle_name="PX4")
        if state.landed_state == airsim.LandedState.Landed:
            print("Failsafe: drone landed unexpectedly → abort")
            break

        pos = state.kinematics_estimated.position
        if pos.x_val >= target_x:
            print("Reached target X.")
            break

        # LiDAR obstacle check
        lidar = client.getLidarData(lidar_name="Lidar1", vehicle_name="PX4")
        if obstacle_ahead(lidar.point_cloud):
            print("Obstacle ahead → ascend 1 m")
            z_set -= vertical_step                  # more negative = up

        # Always stream a fresh set‑point (heartbeat)
        next_x = min(pos.x_val + 1, target_x)
        client.moveToPositionAsync(next_x, 0, z_set, forward_speed, vehicle_name="PX4")
        time.sleep(loop_dt)

    # keep sending hover set‑points for 2 s before landing
    for _ in range(int(2/loop_dt)):
        client.moveToPositionAsync(pos.x_val, 0, z_set, 0.5, vehicle_name="PX4")
        time.sleep(loop_dt)

    client.landAsync(vehicle_name="PX4").join()
finally:
    client.armDisarm(False, vehicle_name="PX4")
    client.enableApiControl(False, vehicle_name="PX4")
    print("Mission finished.")
