from dronekit import connect, VehicleMode
import time
import string


drones = {}

def connect_drones():
    ports = str(input('Enter the port(s) name(s): '))
    ports = ports.replace(',', ' ').strip()
    serial_ports = ports.split()
    for i, port in enumerate(serial_ports):
        try:
            print(f"Connecting to drone {i+1} on {port}...")
            drones[i] = connect(port, baud=57600, timeout=1000, wait_ready=True)
            print(f"Drone {i+1} connected")
        except Exception as e:
            print(f"Failed to connect to drone {i+1} on {port}: {e}")

def takeoff(vehicle, target_altitude):
    """
    Command the drone to take off and wait until it reaches the target altitude.
    
    Args:
    vehicle (Vehicle): The vehicle instance to command.
    target_altitude (float): The altitude to reach in meters.
    """
    print("Taking off...")
    
    # Initiate takeoff
    vehicle.simple_takeoff(target_altitude)
    
    # Wait until the drone reaches the target altitude
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {altitude}")
        if altitude >= target_altitude * 0.95:  # 95% of target altitude
            print("Reached target altitude")
            break
        time.sleep(1)

def land(vehicle):
    """
    Command the drone to land and wait until it has landed.
    
    Args:
    vehicle (Vehicle): The vehicle instance to command.
    """
    print("Landing...")
    
    # Set the vehicle mode to LAND
    vehicle.mode = VehicleMode("LAND")
    
    # Wait until the vehicle is in LAND mode
    while vehicle.mode.name != "LAND":
        print("Waiting for landing mode...")
        time.sleep(1)
    
    # Wait until the vehicle is landed (altitude is approximately zero)
    while vehicle.location.global_relative_frame.alt > 0.1:
        print("Altitude:", vehicle.location.global_relative_frame.alt)
        time.sleep(1)
    
    print("Landed")

   

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
    connect_drones()
    
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
                    takeoff(vehicle, target_altitude)
                except Exception as e:
                    print(f"Failed to take off drone {i+1}: {e}")
            
            while True:
                sub_action = input("Enter 'land' to land the drones, 'wait' to wait at current altitude: ").strip().lower()
                
                if sub_action == 'land':
                    for i, vehicle in drones.items():
                        try:
                            print(f"Landing drone {i+1}...")
                            land(vehicle)
                        except Exception as e:
                            print(f"Failed to land drone {i+1}: {e}")
                    
                    sub_action = input("Enter 'disarm' to disarm the drones, 'takeoff' to take off again: ").strip().lower()
                    if sub_action == 'disarm':
                        disarm_drones(drones)
                        break
                    elif sub_action == 'takeoff':
                        target_altitude = float(input("Enter the target altitude for takeoff: "))
                        
                        # Arm and take off for each drone
                        for i, vehicle in drones.items():
                            try:
                                print(f"Taking off drone {i+1}...")
                                takeoff(vehicle, target_altitude)
                            except Exception as e:
                                print(f"Failed to take off drone {i+1}: {e}")                        
                
                elif sub_action == 'wait':
                    wait_time = int(input("Enter wait time in seconds: "))
                    time.sleep(wait_time)
        else:
            print("Invalid action. Please enter 'disarm' or 'takeoff'.")

if __name__ == "__main__":
    main()
