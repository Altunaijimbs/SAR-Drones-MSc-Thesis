import airsim
import numpy as np
import cv2

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# (Optional) Takeoff for a better view
client.takeoffAsync().join()

# Request image from front camera (change 'front_center' if you named it differently)
responses = client.simGetImages([
    airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
])

response = responses[0]
img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
img_rgb = img1d.reshape(response.height, response.width, 3)

# Save the image locally
cv2.imwrite("front_camera_image.png", img_rgb)

print("Image saved as front_camera_image.png")

# (Optional) Land and release control
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
