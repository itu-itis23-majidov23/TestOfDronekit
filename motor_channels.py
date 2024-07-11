from dronekit import connect

# Connect to the vehicle
vehicle = connect('COM5', baud=57600, wait_ready=True)

# Print motor channels
print("Motor Channels:")
for channel in vehicle.channels:
    print(channel)
