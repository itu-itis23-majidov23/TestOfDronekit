from dronekit import connect

vehicle = connect('COM5', baud=57600, wait_ready=True)
print("Motor Channels:")
for channel in vehicle.channels:
    print(channel)
