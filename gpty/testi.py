import airsim
print(airsim.__file__)
client = airsim.MultirotorClient()
client.confirmConnection()
# Will error if no drone, but prints the path of airsim package in use
