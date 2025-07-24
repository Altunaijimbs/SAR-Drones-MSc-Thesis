import airsim
import time
import threading

# Names should match your AirSim settings.json
drones = ["Drone1", "Drone2", "Drone3"]
clients = {name: airsim.MultirotorClient() for name in drones}

# Connect, enable API, and arm each drone
for name, client in clients.items():
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name=name)
    client.armDisarm(True, vehicle_name=name)

# Takeoff all drones in parallel
def takeoff(name):
    clients[name].takeoffAsync(vehicle_name=name).join()

threads = [threading.Thread(target=takeoff, args=(name,)) for name in drones]
for t in threads: t.start()
for t in threads: t.join()

print(" All drones have taken off.")

# Move to triangle altitudes to avoid collision
for i, name in enumerate(drones):
    z = -5 - i  # -5, -6, -7 meters (AirSim: negative Z is up)
    clients[name].moveToZAsync(z, 2, vehicle_name=name)
time.sleep(5)

print(" Hovering in triangle formation...")
time.sleep(3)

# Land all drones in parallel
def land(name):
    clients[name].landAsync(vehicle_name=name).join()

threads = [threading.Thread(target=land, args=(name,)) for name in drones]
for t in threads: t.start()
for t in threads: t.join()

# Disarm and release API control
for name, client in clients.items():
    client.armDisarm(False, vehicle_name=name)
    client.enableApiControl(False, vehicle_name=name)

print(" All drones have landed and disarmed.")
