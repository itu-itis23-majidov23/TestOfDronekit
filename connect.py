from dronekit import connect, VehicleMode

# Replace 'COM6' with the correct COM port and ensure the baud rate is correct
connection_string = 'COM6'
baud_rate = 57600

print("Connecting to vehicle on:", connection_string)

try:
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    print("Connected to vehicle")
except Exception as e:
    print(f"Error connecting to vehicle: {e}")
    exit(1)

# Example to check the vehicle's mode
print("Current vehicle mode:", vehicle.mode.name)

# Close vehicle object
vehicle.close()
print("Connection closed")
