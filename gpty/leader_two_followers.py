import airsim
import time
import threading
import numpy as np
import cv2

drones = ["drone1", "Follower1", "Follower2"]
offsets = [
    (0, 0),      # Leader
    (-5, -3),    # Follower1
    (-5, 3)      # Follower2
]
altitude = -5
clients = {name: airsim.MultirotorClient() for name in drones}
for name, client in clients.items():
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name=name)
    client.armDisarm(True, vehicle_name=name)

def takeoff(name):
    clients[name].takeoffAsync(vehicle_name=name).join()
threads = [threading.Thread(target=takeoff, args=(name,)) for name in drones]
for t in threads: t.start()
for t in threads: t.join()
for name in drones:
    clients[name].moveToZAsync(altitude, 2, vehicle_name=name)
time.sleep(4)

def detect_orange(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([10, 120, 120])
    upper_orange = np.array([25, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return any(cv2.contourArea(cnt) > 500 for cnt in cnts)

print("🟢 Formation with real sensor-based avoidance.")
detected = False
steps = 20
step_size = 2

try:
    for step in range(steps):
        for i, name in enumerate(drones):
            x = step * step_size + offsets[i][0]
            y = offsets[i][1]
            client = clients[name]

            # Sensor-based avoidance!
            sensor = client.getDistanceSensorData("Front_Distance", vehicle_name=name)
            # If distance sensor detects obstacle within 3 meters, sidestep
            if sensor.distance < 3.0 and sensor.distance > 0:
                print(f"🚨 {name}: Obstacle detected at {sensor.distance:.2f}m! Sidestepping...")
                # Move sideways to avoid (left for Follower1, right for Follower2, left for Leader)
                y += 5 if i != 2 else -5  # Sidestep direction per drone
                # Optionally: You could also move in Y randomly or alternate direction

            client.moveToPositionAsync(x, y, altitude, 3, vehicle_name=name)

        time.sleep(1.2)

        # Detections
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
