from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import threading

stop_flag = False

def arm_and_ask_for_takeoff(vehicle, target_altitude):
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Vehicle armed and ready!")

    while True:
        action = input("Enter 'takeoff' to proceed with takeoff or 'abort' to disarm: ").strip().lower()
        if action == 'takeoff':
            takeoff(vehicle, target_altitude)
            break
        elif action == 'abort':
            vehicle.armed = False
            print("Takeoff aborted, motors disarmed.")
            break
        else:
            print("Invalid input. Please enter 'takeoff' or 'abort'.")

def takeoff(vehicle, target_altitude):
    print(f"Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def monitor_user_input():
    global stop_flag
    while True:
        user_input = input("Enter 'stop' to stop the vehicle: ").strip().lower()
        if user_input == 'stop':
            stop_flag = True
            break

def land_slowly(vehicle):
    print("Initiating slow landing...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
        time.sleep(1)
    print("Landed and disarmed.")

def ask_user_for_action(vehicle):
    while True:
        action = input("Enter 'stay' to hover in place or 'land' to land the vehicle: ").strip().lower()
        if action == 'stay':
            print("Vehicle will hover in place.")
            vehicle.mode = VehicleMode("LOITER")
            break
        elif action == 'land':
            print("Vehicle will land.")
            land_slowly(vehicle)
            break
        else:
            print("Invalid input. Please enter 'stay' or 'land'.")

if __name__ == "__main__":
    connection_string = 'COM5' 

    try:
        print(f"Connecting to vehicle on: {connection_string}")
        vehicle = connect(connection_string, baud=57600, wait_ready=True)
        print(" Type: %s" % vehicle._vehicle_type)
        print(" Armed: %s" % vehicle.armed)
        print(" System status: %s" % vehicle.system_status.state)
        target_altitude = 1.6  
        arm_and_ask_for_takeoff(vehicle, target_altitude)
        user_input_thread = threading.Thread(target=monitor_user_input)
        user_input_thread.start()

        while not stop_flag:
            time.sleep(1)

        ask_user_for_action(vehicle)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if 'vehicle' in locals():
            vehicle.close()
            print("Connection closed.")

    print("Completed.")

