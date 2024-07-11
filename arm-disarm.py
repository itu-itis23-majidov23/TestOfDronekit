from dronekit import connect, VehicleMode
import time

# Replace 'COM5' with the correct port for your telemetry module
connection_string = 'COM5'  # Adjust as needed for your setup

try:
    # Connect to the Vehicle
    print(f"Connecting to vehicle on: {connection_string}")
    vehicle = connect(connection_string, baud=57600, wait_ready=True)

    # Display basic vehicle state
    print(" Type: %s" % vehicle._vehicle_type)
    print(" Armed: %s" % vehicle.armed)
    print(" System status: %s" % vehicle.system_status.state)

    # Arm the vehicle without waiting for GPS HDOP
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")  # Set mode to guided for arming

    # Wait for arming with a timeout
    timeout = 30  # Timeout in seconds
    start_time = time.time()
    while not vehicle.armed:
        if time.time() - start_time > timeout:
            raise Exception("Timeout waiting for arming")

        # Attempt to arm without GPS check
        vehicle.armed = True
        time.sleep(1)  # Wait for a second to allow the vehicle to change state

    # Vehicle is armed, perform operations here
    print("Vehicle armed and ready!")

    # Example operations...
    time.sleep(5)  # Placeholder for operations

    # Disarm the vehicle
    print("Disarming motors...")
    vehicle.armed = False

    # Wait for disarming to complete
    while vehicle.armed:
        print(" Waiting for disarming...")
        time.sleep(1)

    print("Vehicle disarmed.")

except Exception as e:
    print(f"Error: {e}")

finally:
    # Close vehicle object before exiting script
    if 'vehicle' in locals():
        vehicle.close()
        print("Connection closed")

print("Completed")
