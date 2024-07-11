from dronekit import connect, VehicleMode
import sys

connection_string = 'COM5' 
try:
    print(f"Connecting to vehicle on: {connection_string}")
    vehicle = connect(connection_string, baud=57600, wait_ready=True)
    print(" Type: %s" % vehicle._vehicle_type)
    print(" Armed: %s" % vehicle.armed)
    print(" System status: %s" % vehicle.system_status.state)
    print(" GPS: %s" % vehicle.gps_0)
    print(" Altitude: %s" % vehicle.location.global_relative_frame.alt)

    while True:
        print("Latitude: %s" % vehicle.location.global_relative_frame.lat)
        print("Longitude: %s" % vehicle.location.global_relative_frame.lon)
        user_input = input("Press 'q' to disconnect: ")
        if user_input.lower() == 'q':
            break  

except Exception as e:
    print(f"Error connecting to vehicle: {e}")
    sys.exit(1)

finally:
    if 'vehicle' in locals():
        vehicle.close()
        print("Connection closed")

print("Completed")
