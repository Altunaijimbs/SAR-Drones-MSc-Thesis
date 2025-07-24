import airsim
import numpy as np
import open3d as o3d

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Get Lidar data
lidar_data = client.getLidarData(lidar_name="Lidar1")

if lidar_data.point_cloud:
    # Convert flat point list to Nx3 numpy array
    points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
    print(f"Received {points.shape[0]} points from Lidar.")

    # Flip Z axis to match "up is up"
    points[:, 2] = -points[:, 2]

    # Create Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Optionally color by height (z-value)
    if points.shape[0] > 0:
        z_vals = points[:, 2]
        z_ptp = np.ptp(z_vals)
        z_norm = (z_vals - z_vals.min()) / (z_ptp if z_ptp > 0 else 1)

        colors = np.zeros_like(points)
        colors[:, 2] = z_norm  # blue gradient
        pcd.colors = o3d.utility.Vector3dVector(colors)

    # Visualize
    o3d.visualization.draw_geometries([pcd], window_name='AirSim Lidar Point Cloud (Z axis flipped)')
else:
    print("No Lidar points received. Make sure the sensor is enabled and AirSim is running.")
