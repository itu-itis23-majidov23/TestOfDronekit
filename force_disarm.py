from dronekit import connect, VehicleMode
import time

# Connect to the Vehicle
print("Connecting to vehicle...")
vehicle = connect('COM5', wait_ready=True)  # Update connection string as per your setup

def force_disarm(vehicle):
    print("Setting vehicle mode to STABILIZE...")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(2)

    print("Current vehicle mode: ", vehicle.mode.name)
    print("Arming status: ", vehicle.armed)

    print("Disarming the drone...")
    vehicle.armed = False
    while vehicle.armed:
        print(" Waiting for disarm...")
        time.sleep(1)
        print("Arming status: ", vehicle.armed)

    print("Drone disarmed.")

# Attempt to disarm the drone
force_disarm(vehicle)

# Close the vehicle object
vehicle.close()
print("Vehicle disconnected")
