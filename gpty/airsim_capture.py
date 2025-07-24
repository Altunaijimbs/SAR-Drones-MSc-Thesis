import airsim
import numpy as np
import cv2

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Takeoff (optional, comment if you don't want auto-takeoff)
client.takeoffAsync().join()

# Capture image from camera "0"
response = client.simGetImage("0", airsim.ImageType.Scene)
if response:
    # Convert bytes to numpy array and save
    img1d = np.frombuffer(response, dtype=np.uint8)  
    img_rgba = cv2.imdecode(img1d, cv2.IMREAD_UNCHANGED)
    cv2.imwrite("airsim_capture.png", img_rgba)
    print("Image saved as airsim_capture.png")
else:
    print("No image received. Make sure AirSim is running with camera 0.")

# Get drone position (in NED: North-East-Down coordinates)
state = client.getMultirotorState()
pos = state.kinematics_estimated.position
print(f"Drone position (meters): x={pos.x_val:.2f}, y={pos.y_val:.2f}, z={pos.z_val:.2f}")

# Example: Distance from ground (altitude) is just abs(z), since AirSim Z is negative when above ground
altitude = abs(pos.z_val)
print(f"Drone altitude: {altitude:.2f} meters")
