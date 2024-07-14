from dronekit import connect, VehicleMode
import time

# Replace with your vehicle connection string
connection_string = 'COM5'

# Connect to the vehicle
print(f"Connecting to vehicle on: {connection_string}")
vehicle = connect(connection_string, wait_ready=True)

try:
    # Check if vehicle is armable
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)

    # Change vehicle mode to Loiter
    print("Switching to Loiter mode...")
    vehicle.mode = VehicleMode("LOITER")

    # Wait for the mode to change
    while vehicle.mode.name != "LOITER":
        print(f"Waiting for mode change to {vehicle.mode.name}...")
        time.sleep(1)

    print("Vehicle is now in Loiter mode.")

    # Perform other tasks while in Loiter mode
    time.sleep(10)  # Example: wait for 10 seconds

finally:
    # Close vehicle object before exiting script
    vehicle.close()
    print("Vehicle connection closed.")
