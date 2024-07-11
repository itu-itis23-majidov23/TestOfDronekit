from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import threading

# Global variable to control the loop
stop_flag = False

def arm_and_takeoff(vehicle, target_altitude):
    print(f"Arming motors and taking off to {target_altitude} meters...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Vehicle armed and ready!")

    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        if stop_flag:
            print("Stopping operation as requested.")
            vehicle.mode = VehicleMode("LAND")
            return
        time.sleep(1)

def monitor_user_input():
    global stop_flag
    while True:
        user_input = input("Enter 'stop' to stop the vehicle: ").strip().lower()
        if user_input == 'stop':
            stop_flag = True
            break

def move(vehicle, target_location):
    print(f"Moving to location: {target_location}")
    vehicle.simple_goto(target_location)

    while not stop_flag:
        current_location = vehicle.location.global_relative_frame
        distance = get_distance_metres(current_location, target_location)
        print(f" Distance to target: {distance} meters")

        if distance <= 1:
            print("Reached target location")
            vehicle.mode = VehicleMode("LOITER")
            break

        time.sleep(1)

    ask_next_action(vehicle)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the earth's poles.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return ((dlat**2) + (dlong**2))**0.5 * 1.113195e5

def ask_next_action(vehicle):
    while True:
        action = input("Enter 'stay' to stay in position, 'land' to land, or 'move' to move to a new target: ").strip().lower()
        if action == 'stay':
            print("Staying in position.")
            return
        elif action == 'land':
            vehicle.mode = VehicleMode("LAND")
            while vehicle.armed:
                print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
                time.sleep(1)
            print("Landed and disarmed.")
            disarm_or_move(vehicle)
            return
        elif action == 'move':
            target_lat = float(input("Enter target latitude: "))
            target_lon = float(input("Enter target longitude: "))
            target_alt = float(input("Enter target altitude: "))
            target_location = LocationGlobalRelative(target_lat, target_lon, target_alt)
            move(vehicle, target_location)
            return
        else:
            print("Invalid input. Please enter 'stay', 'land', or 'move'.")

def disarm_or_move(vehicle):
    while True:
        action = input("Enter 'disarm' to disarm motors or 'move' to move to a new target: ").strip().lower()
        if action == 'disarm':
            force_disarm(vehicle)
            return
        elif action == 'move':
            target_lat = float(input("Enter target latitude: "))
            target_lon = float(input("Enter target longitude: "))
            target_alt = float(input("Enter target altitude: "))
            target_location = LocationGlobalRelative(target_lat, target_lon, target_alt)
            arm_and_takeoff(vehicle, target_alt)
            move(vehicle, target_location)
            return
        else:
            print("Invalid input. Please enter 'disarm' or 'move'.")

def force_disarm(vehicle):
    try:
        print("Forcefully disarming motors...")
        vehicle.armed = False

        # Attempt to force disarm by setting armed to False repeatedly
        attempt_count = 0
        while vehicle.armed:
            vehicle.armed = False
            attempt_count += 1
            if attempt_count >= 3:  # Adjust the number of attempts as needed
                raise Exception("Failed to force disarm after multiple attempts")
            time.sleep(1)

        print("Vehicle forcefully disarmed.")
    
    except Exception as e:
        print(f"Error during force disarm: {e}")

def failsafe(vehicle):
    print("Entering failsafe mode...")
    vehicle.mode = VehicleMode("LOITER")
    time.sleep(1)
    print("Failsafe mode activated. Vehicle is loitering.")

if __name__ == "__main__":
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

        # Example: Monitor user input to stop or adjust commands
        user_input_thread = threading.Thread(target=monitor_user_input)
        user_input_thread.start()

        # Arm and takeoff example
        target_altitude = 1.6  # Set target altitude (in meters)
        arm_and_takeoff(vehicle, target_altitude)

        # Ask user if they want to move to a target
        ask_next_action(vehicle)

    except Exception as e:
        print(f"Error: {e}")
        failsafe(vehicle)

    finally:
        if 'vehicle' in locals():
            vehicle.close()
            print("Connection closed.")

    print("Completed.")

