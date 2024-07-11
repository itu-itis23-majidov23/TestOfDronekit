from dronekit import connect, VehicleMode
import time

print("Connecting to vehicle...")
vehicle = connect('COM5', wait_ready=True) 

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

force_disarm(vehicle)
vehicle.close()
print("Vehicle disconnected")
