from dronekit import connect, VehicleMode
import time


serial_ports = ['COM5', 'COM6']
drones = {}

def connect_drones(serial_ports):
    for i, port in enumerate(serial_ports):
        try:
            print(f"Connecting to drone {i+1} on {port}...")
            drones[i] = connect(port, baud=57600, timeout=200, wait_ready=True)
            print(f"Drone {i+1} connected")
        except Exception as e:
            print(f"Failed to connect to drone {i+1} on {port}: {e}")

def arm_and_takeoff(vehicle, target_altitude):
    print("Arming motors...")

    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)

    print("Taking off...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def land_and_disarm(vehicle):
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("Waiting for vehicle to land and disarm...")
        time.sleep(1)
    print("Disarmed.")

def arm_drones(drones):
    for i, vehicle in drones.items():
        try:
            print(f"Arming drone {i+1}...")
            while not vehicle.is_armable:
                print("Waiting for vehicle to become armable...")
                time.sleep(1)
            vehicle.mode = VehicleMode("GUIDED")
            vehicle.armed = True
            while not vehicle.armed:
                print("Waiting for vehicle to arm...")
                time.sleep(1)
            print(f"Drone {i+1} armed")
        except Exception as e:
            print(f"Failed to arm drone {i+1}: {e}")

def disarm_drones(drones):
    for i, vehicle in drones.items():
        print(f"Disarming drone {i+1}...")
        vehicle.armed = False
        while vehicle.armed:
            print(f"Waiting for drone {i+1} to disarm...")
            time.sleep(1)
    print("All drones disarmed")

def main():
    connect_drones(serial_ports)
    
    if not drones:
        print("No drones connected. Exiting.")
        return

    while True:
        action = input("Enter 'arm' to arm all drones, 'disconnect' to disconnect all drones: ").strip().lower()
        
        if action == 'disconnect':
            for i, vehicle in drones.items():
                vehicle.close()
            print("All drones disconnected")
            break
        
        elif action == 'arm':
            arm_drones(drones)
            break
        else:
            print("Invalid action. Please enter 'arm' or 'disconnect'.")

    while True:
        action = input("Enter 'disarm' to disarm all drones or 'takeoff' to take off: ").strip().lower()
        
        if action == 'disarm':
            disarm_drones(drones)
            break
        
        elif action == 'takeoff':
            target_altitude = float(input("Enter the target altitude for takeoff: "))
            
            # Arm and take off for each drone
            for i, vehicle in drones.items():
                try:
                    print(f"Taking off drone {i+1}...")
                    arm_and_takeoff(vehicle, target_altitude)
                except Exception as e:
                    print(f"Failed to take off drone {i+1}: {e}")
            
            while True:
                sub_action = input("Enter 'land' to land the drones, 'wait' to wait at current altitude: ").strip().lower()
                
                if sub_action == 'land':
                    for i, vehicle in drones.items():
                        try:
                            print(f"Landing drone {i+1}...")
                            land_and_disarm(vehicle)
                        except Exception as e:
                            print(f"Failed to land drone {i+1}: {e}")
                    
                    sub_action = input("Enter 'disarm' to disarm the drones, 'takeoff' to take off again: ").strip().lower()
                    if sub_action == 'disarm':
                        disarm_drones(drones)
                        break
                    elif sub_action == 'takeoff':
                        break  
                
                elif sub_action == 'wait':
                    wait_time = int(input("Enter wait time in seconds: "))
                    time.sleep(wait_time)
        else:
            print("Invalid action. Please enter 'disarm' or 'takeoff'.")

if __name__ == "__main__":
    main()
