from dronekit import connect, VehicleMode
import sys

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
    print(" GPS: %s" % vehicle.gps_0)
    print(" Altitude: %s" % vehicle.location.global_relative_frame.alt)

    # Continuously print location until user decides to disconnect
    while True:
        # Print current latitude and longitude
        print("Latitude: %s" % vehicle.location.global_relative_frame.lat)
        print("Longitude: %s" % vehicle.location.global_relative_frame.lon)

        # Wait for user input to disconnect
        user_input = input("Press 'q' to disconnect: ")
        if user_input.lower() == 'q':
            break  # Exit the loop and close the connection

except Exception as e:
    print(f"Error connecting to vehicle: {e}")
    sys.exit(1)

finally:
    # Close vehicle object before exiting script
    if 'vehicle' in locals():
        vehicle.close()
        print("Connection closed")

print("Completed")
