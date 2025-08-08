
import cosysairsim as airsim
import numpy as np

client = airsim.MultirotorClient()
client.confirmConnection()
print("Connected with cosysairsim!")

# Try to get camera image
try:
    # Method 1: Standard call
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    if responses and responses[0].width > 0:
        print(f"✅ Camera works! Got {responses[0].width}x{responses[0].height} image")
    else:
        print("❌ No image data")
except Exception as e:
    print(f"❌ Camera failed: {e}")
