import airsim
import numpy as np
import plotly.graph_objects as go

# Connect to AirSim and explicitly specify vehicle_name
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, vehicle_name="PX4")
client.armDisarm(True, vehicle_name="PX4")

# Helper: convert AirSim LiDAR local frame (X fwd, Y right, Z down)
#            to Plotly view (X right, Y fwd, Z up)
def convert_frame(points):
    pts = points[:, [1, 0, 2]]  # swap X↔Y
    pts[:, 2] *= -1             # flip Z (down → up)
    return pts

# Fetch LiDAR data
lidar_data1 = client.getLidarData(lidar_name="Lidar1", vehicle_name="PX4")
lidar_data2 = client.getLidarData(lidar_name="Lidar2", vehicle_name="PX4")

# ---- LiDAR1 ----
if len(lidar_data1.point_cloud) > 0:
    points1 = np.array(lidar_data1.point_cloud, dtype=np.float32).reshape(-1, 3)
    points1 = convert_frame(points1)
    fig1 = go.Figure(data=[go.Scatter3d(
        x=points1[:, 0], y=points1[:, 1], z=points1[:, 2],
        mode='markers', marker=dict(size=1, color='blue')
    )])
    fig1.update_layout(title='LiDAR1 Point Cloud (Corrected)', scene=dict(
        xaxis_title='X (Right, m)', yaxis_title='Y (Forward, m)', zaxis_title='Z (Up, m)'))
    fig1.write_html('lidar1_point_cloud.html')
else:
    print("No points from Lidar1")

# ---- LiDAR2 ----
if len(lidar_data2.point_cloud) > 0:
    points2 = np.array(lidar_data2.point_cloud, dtype=np.float32).reshape(-1, 3)
    points2 = convert_frame(points2)
    fig2 = go.Figure(data=[go.Scatter3d(
        x=points2[:, 0], y=points2[:, 1], z=points2[:, 2],
        mode='markers', marker=dict(size=1, color='red')
    )])
    fig2.update_layout(title='LiDAR2 Point Cloud (Corrected)', scene=dict(
        xaxis_title='X (Right, m)', yaxis_title='Y (Forward, m)', zaxis_title='Z (Up, m)'))
    fig2.write_html('lidar2_point_cloud.html')
else:
    print("No points from Lidar2")

# Release drone control
client.armDisarm(False, vehicle_name="PX4")
client.enableApiControl(False, vehicle_name="PX4")

print("Corrected LiDAR plots saved as lidar1_point_cloud.html and lidar2_point_cloud.html")
