import airsim
import time
import threading
import numpy as np
import cv2

# Drones
drones = ["Leader", "Follower"]
offset = (-5, 0)  # Follower is 5m behind leader in X

clients = {name: airsim.MultirotorClient() for name in drones}
for name, client in clients.items():
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name=name)
    client.armDisarm(True, vehicle_name=name)

# Takeoff all
def takeoff(name):
    clients[name].takeoffAsync(vehicle_name=name).join()
threads = [threading.Thread(target=takeoff, args=(name,)) for name in drones]
for t in threads: t.start()
for t in threads: t.join()

altitude = -5
for name in drones:
    clients[name].moveToZAsync(altitude, 2, vehicle_name=name)
time.sleep(4)

print("🟢 Flying leader-follower... Press Ctrl+C to stop.")

# Helper: Detect orange in AirSim image
def detect_orange(img):
    # Convert image to HSV and find orange
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Orange HSV range may need tweaking for your sphere!
    lower_orange = np.array([10, 120, 120])
    upper_orange = np.array([25, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # If big orange area found
    return any(cv2.contourArea(cnt) > 500 for cnt in cnts)

# Main loop: Leader moves, follower tracks, both check camera
detected = False
steps = 20
step_size = 2
try:
    for step in range(steps):
        # Move leader
        leader_x = step * step_size
        leader_y = 0
        clients["Leader"].moveToPositionAsync(leader_x, leader_y, altitude, 3, vehicle_name="Leader")

        # Move follower to offset from leader
        follower_x = leader_x + offset[0]
        follower_y = leader_y + offset[1]
        clients["Follower"].moveToPositionAsync(follower_x, follower_y, altitude, 3, vehicle_name="Follower")

        time.sleep(1.2)

        # Each drone checks its front camera for orange sphere
        for name in drones:
            responses = clients[name].simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
            ], vehicle_name=name)
            if responses and responses[0].width > 0:
                img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
                img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
                if detect_orange(img_rgb):
                    print(f"🟠 Orange sphere detected by {name} at step {step}!")
                    detected = True
                    break
        if detected:
            break
except KeyboardInterrupt:
    print("Stopped by user.")

# Stop & land all drones
print("🛑 Stopping. Landing all drones...")
def land(name):
    clients[name].landAsync(vehicle_name=name).join()
threads = [threading.Thread(target=land, args=(name,)) for name in drones]
for t in threads: t.start()
for t in threads: t.join()
for name, client in clients.items():
    client.armDisarm(False, vehicle_name=name)
    client.enableApiControl(False, vehicle_name=name)
print("✅ Mission complete.")
