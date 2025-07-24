import airsim
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

client = airsim.MultirotorClient()
client.confirmConnection()

# Get Lidar data (use the lidar name from settings.json, e.g. "Lidar1")
lidar_data = client.getLidarData(lidar_name="Lidar1",)

if lidar_data.point_cloud:
    # point_cloud is a flat array: [x1, y1, z1, x2, y2, z2, ...]
    points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
    print("Lidar points shape:", points.shape)
    print("First 5 points:\n", points[:5])

    # Plot using matplotlib
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1)
    ax.set_xlabel('X (Forward, m)')
    ax.set_ylabel('Y (Right, m)')
    ax.set_zlabel('Z (Down, m)')
    ax.set_title('AirSim Lidar Point Cloud')
    plt.ion()
    plt.show()
    input("Press Enter to close plot...")
    plt.close()


else:
    print("No Lidar points received. Make sure the sensor is enabled and AirSim is running.")
