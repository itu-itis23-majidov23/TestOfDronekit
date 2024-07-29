from dronekit import connect, VehicleMode
import time
import threading

stop_flag = False

def arm_and_ask_for_takeoff(vehicle, target_altitude):
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Vehicle armed and ready!")

    while True:
        action = input("Enter 'takeoff' to proceed with takeoff or 'disarm' to disarm: ").strip().lower()
        if action == 'takeoff':
            takeoff(vehicle, target_altitude)
            break
        elif action == 'disarm':
            vehicle.armed = False
            print("Takeoff aborted, motors disarmed.")
            break
        else:
            print("Invalid input. Please enter 'takeoff' or 'disarm'.")

def takeoff(vehicle, target_altitude):
    print(f"Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f} meters")
        print(f"Throttle: {vehicle.channels['3']:.2f}%")  # Assuming throttle is on channel 3
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def stay_stable(vehicle):
    duration = int(input("Enter how many seconds to stay stable: ").strip())
    start_time = time.time()
    while time.time() - start_time < duration:
        print(f"Staying stable... ({duration - int(time.time() - start_time)} seconds remaining)")
        print(f"Throttle: {vehicle.channels['3']:.2f}%")  # Assuming throttle is on channel 3
        time.sleep(1)

def land_slowly(vehicle):
    print("Initiating slow landing...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f} meters")
        print(f"Throttle: {vehicle.channels['3']:.2f}%")  # Assuming throttle is on channel 3
        time.sleep(1)
    
    print("Landed. Disarming motors...")
    vehicle.armed = False
    time.sleep(1)  # Ensure vehicle is fully disarmed before closing the connection
    print("Motors disarmed.")

def ask_user_for_action(vehicle):
    global stop_flag
    while True:
        action = input("Enter 'stay' to hover in place or 'land' to land the vehicle: ").strip().lower()
        if action == 'stay':
            stay_stable(vehicle)
            continue_staying = input("Enter 'stay' to continue staying or 'land' to land the vehicle: ").strip().lower()
            if continue_staying == 'stay':
                continue
            elif continue_staying == 'land':
                break
            else:
                print("Invalid input. Please enter 'stay' or 'land'.")
                continue
        elif action == 'land':
            print("Vehicle will land.")
            land_slowly(vehicle)
            stop_flag = True  # Set stop flag after landing and disarming
            break
        else:
            print("Invalid input. Please enter 'stay' or 'land'.")



if __name__ == "__main__":
    connection_string = 'COM6'  # Replace with your connection string

    try:
        print(f"Connecting to vehicle on: {connection_string}")
        vehicle = connect(connection_string, baud=57600, wait_ready=True)
        print("Type: %s" % vehicle._vehicle_type)
        print("Armed: %s" % vehicle.armed)
        print("System status: %s" % vehicle.system_status.state)

        target_altitude = 5 # Set your target altitude here
        arm_and_ask_for_takeoff(vehicle, target_altitude)


        while not stop_flag:
            ask_user_for_action(vehicle)

            if stop_flag:
                break  # Exit loop if stop flag is set after landing

            takeoff_again = input("Enter 'takeoff' to take off again or 'stop' to stop the loop: ").strip().lower()
            if takeoff_again == 'takeoff':
                arm_and_ask_for_takeoff(vehicle, target_altitude)
            elif takeoff_again == 'stop':
                stop_flag = True
            else:
                print("Invalid input. Please enter 'takeoff' or 'stop'.")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if 'vehicle' in locals():
            vehicle.close()
            print("Connection closed.")

    print("Completed.")
