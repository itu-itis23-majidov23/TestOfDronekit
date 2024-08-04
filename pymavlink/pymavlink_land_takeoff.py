import time
import asyncio
import serial.tools.list_ports
from pymavlink import mavutil

# List available serial ports
def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    port_list = []

    print("Available Serial Ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port.device}")
        port_list.append(port.device)

    return port_list

# Create a connection to the drone
def create_connection(connection_str, baud=57600, timeout=30):
    try:
        master = mavutil.mavlink_connection(connection_str, baud=baud, timeout=timeout)
        heartbeat_received = wait_for_heartbeat(master, timeout)
        if heartbeat_received:
            print(f"Heartbeat received from {connection_str}")
            return master
        else:
            raise ConnectionError(f"No heartbeat received from {connection_str} within timeout period.")
    except Exception as e:
        raise ConnectionError(f"Failed to connect to {connection_str}: {e}")

# Wait for heartbeat from the drone
def wait_for_heartbeat(master, timeout=30):
    start_time = time.time()
    while time.time() - start_time < timeout:
        if master.recv_match(type='HEARTBEAT', blocking=False):
            return True
        time.sleep(1)
    return False

# Send arm command to the drone
def arm(master):
    print(f"Arming drone {master.target_system}...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print(f"Drone {master.target_system} armed")

# Send disarm command to the drone
def disarm(master):
    print(f"Disarming drone {master.target_system}...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print(f"Drone {master.target_system} disarmed")

# Set mode of the drone
def set_mode(master, mode):
    master.set_mode(mode)
    print(f"Drone {master.target_system} set to {mode} mode")

# Check if drone is armed
def is_armed(master):
    master.recv_match(type='HEARTBEAT', blocking=True)
    return master.motors_armed()

# Send takeoff command to the drone
def takeoff(master, altitude):
    print(f"Drone {master.target_system} taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )

# Send land command to the drone
def land(master):
    print(f"Drone {master.target_system} landing...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )

# Get the current altitude of the drone
def get_altitude(master):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        return msg.relative_alt / 1000.0  # Altitude in meters
    return None

async def perform_action(connections, action, altitude=None):
    if action == "takeoff":
        takeoff_tasks = [asyncio.to_thread(takeoff, conn, altitude) for conn in connections if conn]
        await asyncio.gather(*takeoff_tasks)
    elif action == "land":
        land_tasks = [asyncio.to_thread(land, conn) for conn in connections if conn]
        await asyncio.gather(*land_tasks)
    elif action == "arm":
        for conn in connections:
            if conn:
                arm(conn)
    elif action == "disarm":
        for conn in connections:
            if conn:
                disarm(conn)
    elif action == "set_mode":
        for conn in connections:
            if conn:
                set_mode(conn, altitude)  # here altitude is used as mode for simplicity

async def monitor_altitude(connections, target_altitude, wait_time):
    start_time = time.time()
    while time.time() - start_time < wait_time:
        for conn in connections:
            if conn:
                altitude = get_altitude(conn)
                print(f"Drone {conn.target_system} altitude: {altitude} meters")
                if altitude >= target_altitude:
                    return
        await asyncio.sleep(1)

async def main():
    try:
        # List available ports
        available_ports = list_serial_ports()

        # Ask user to select ports
        selected_ports = input("Select ports by index (comma-separated, e.g., 0,1): ").split(',')
        selected_ports = [available_ports[int(idx.strip())] for idx in selected_ports]

        # Establish drone connections
        connections = []
        for port in selected_ports:
            try:
                baud_rate = int(input(f"Enter baud rate for {port} (default 57600): ") or 57600)
                conn = create_connection(port, baud_rate)
                connections.append(conn)
            except ConnectionError as e:
                print(e)

        if not connections:
            print("No connections established. Exiting.")
            return

        # Set mode to GUIDED and arm the drones
        await perform_action(connections, "set_mode", "GUIDED")
        await perform_action(connections, "arm")

        # Ensure drones are armed before takeoff
        for conn in connections:
            while not is_armed(conn):
                print(f"Waiting for drone {conn.target_system} to arm...")
                time.sleep(1)

        # Ask user to takeoff or wait
        takeoff_choice = input("Do you want to takeoff the drones or wait? (takeoff/wait): ").strip().lower()
        if takeoff_choice == "takeoff":
            altitude = float(input("Enter the takeoff altitude (meters): "))
            await perform_action(connections, "takeoff", altitude)
            # Wait and monitor altitude
            await monitor_altitude(connections, altitude, wait_time=30)

        # Ask user to land or wait
        land_choice = input("Do you want to land the drones or wait? (land/wait): ").strip().lower()
        if land_choice == "land":
            await perform_action(connections, "land")
            # Ask user to disarm the drones
            disarm_choice = input("Do you want to disarm the drones? (yes/no): ").strip().lower()
            if disarm_choice == "yes":
                await perform_action(connections, "disarm")

        # Close connections
        for conn in connections:
            if conn:
                conn.close()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    asyncio.run(main())
