from pymavlink import mavutil
import time
import serial.tools.list_ports
import threading
from pymavlink_move import handle_move_action

drones = {}


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
    return available_ports


def set_mode(drone: mavutil.mavlink_connection, mode: str) -> None:
    """
    Set the flight mode of the drone.

    Args:
        drone (mavutil.mavlink_connection): The drone connection.
        mode (str): The flight mode to set (e.g., "GUIDED", "LOITER", "RTL").
    """
    # Get the mode ID
    if mode not in drone.mode_mapping():
        print(f"Unknown mode: {mode}")
        print(f"Available modes: {list(drone.mode_mapping().keys())}")
        return

    mode_id = drone.mode_mapping()[mode]

    # Set the mode
    drone.mav.set_mode_send(
        drone.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
    )

    # Wait for ACK command
    # MAVLink requires an ACK from the drone to confirm the mode change
    ack = None
    while not ack:
        ack = drone.recv_match(type="COMMAND_ACK", blocking=True)
        if ack:
            try:
                ack_result = ack.result
                if ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    if ack_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print(f"Mode change to {mode} accepted")
                    else:
                        print(f"Mode change to {mode} failed with result {ack_result}")
                    break
            except AttributeError as e:
                print(f"Error processing ACK message: {e}")
        else:
            print("No ACK received, retrying...")


def connect_drones() -> None:
    """
    Connect to drones on specified serial ports and set their mode to GUIDED.

    Args:
        None
    """
    available_ports = list_serial_ports()
    print("Available serial ports:", available_ports)
    ports = input("Enter the port name(s): ")
    ports = ports.replace(",", " ").strip()
    serial_ports = ports.split()

    for i, port in enumerate(serial_ports):
        try:
            print(f"Connecting to drone {i+1} on {port}...")
            drones[i] = mavutil.mavlink_connection(port, baud=57600, timeout=30)
            drones[i].wait_heartbeat()
            print(f"Drone {i+1} connected")

            # Set the mode to GUIDED
            set_mode(drones[i], "GUIDED")
        except Exception as e:
            print(f"Failed to connect to drone {i+1} on {port}: {e}")


def set_parameter(drone, param_id, param_value):
    """
    Set a parameter on the drone.

    Args:
        drone: MAVLink connection object.
        param_id: The parameter id (name) as string.
        param_value: The parameter value to set.

    Returns:
        None
    """
    param_id = param_id.encode("utf-8")
    drone.mav.param_set_send(
        drone.target_system,
        drone.target_component,
        param_id,
        float(param_value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    # Wait for the parameter to be set
    while True:
        ack = drone.recv_match(type="PARAM_VALUE", blocking=True)
        if ack and ack.param_id.decode("utf-8") == param_id.decode("utf-8"):
            print(f"Parameter {param_id.decode('utf-8')} set to {ack.param_value}")
            break


def arm_drones(drones):
    for i, drone in drones.items():
        try:
            # Set the disarm delay parameter before arming
            desired_disarm_delay = 30  # Adjust this value as needed
            set_parameter(drone, "DISARM_DELAY", desired_disarm_delay)
            print(f"Arming drone {i+1}...")

            drone.mav.command_long_send(
                drone.target_system,
                drone.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            drone.motors_armed_wait()
            print(f"Drone {i+1} armed")
        except Exception as e:
            print(f"Failed to arm drone {i+1}: {e}")


def disarm_drones(drones):
    for i, drone in drones.items():
        try:
            print(f"Disarming drone {i+1}...")
            drone.mav.command_long_send(
                drone.target_system,
                drone.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,  # 0 to disarm
                0,
                0,
                0,
                0,
                0,
                0,
            )
            drone.motors_disarmed_wait()
            print(f"Drone {i+1} disarmed")
        except Exception as e:
            print(f"Failed to disarm drone {i+1}: {e}")


def takeoff(drone, target_altitude):
    """
    Command the drone to take off and wait until it reaches the target altitude.

    Args:
    drone: The drone instance to command.
    target_altitude (float): The altitude to reach in meters.
    """
    print("Taking off...")
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        target_altitude,
    )

    # Wait until the drone reaches the target altitude
    while True:
        msg = drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        altitude = msg.relative_alt / 1000.0  # altitude in meters
        print(f"Altitude: {altitude}")
        if altitude >= target_altitude * 0.95:  # 95% of target altitude
            print("Reached target altitude")
            break
        time.sleep(0.5)


def execute_takeoff(target_altitude):
    threads = []
    for i, drone in drones.items():
        try:
            print(f"Taking off drone {i+1} to altitude {target_altitude}...")
            t = threading.Thread(target=takeoff, args=(drone, target_altitude))
            threads.append(t)
            t.start()
        except Exception as e:
            print(f"Failed to take off drone {i+1}: {e}")

    for t in threads:
        t.join()


def handle_post_takeoff_actions():
    while True:
        sub_action = (
            input(
                "Enter 'land' to land the drones, 'wait' to wait at current altitude: "
            )
            .strip()
            .lower()
        )
        if sub_action == "land":
            land_drones()
            handle_post_land_actions()
            break
        elif sub_action == "wait":
            wait_time = int(input("Enter wait time in seconds: "))
            time.sleep(wait_time)
        else:
            print("Invalid action. Please enter 'land' or 'wait'.")


def handle_post_land_actions():
    while True:
        sub_action = (
            input("Enter 'disarm' to disarm the drones, 'takeoff' to take off again: ")
            .strip()
            .lower()
        )
        if sub_action == "disarm":
            disarm_drones(drones)
            break
        elif sub_action == "takeoff":
            target_altitude = float(input("Enter the target altitude for takeoff: "))
            execute_takeoff(target_altitude)
            handle_post_takeoff_actions()
            break
        else:
            print("Invalid action. Please enter 'disarm' or 'takeoff'.")


def handle_post_move_actions():
    while True:
        sub_action = (
            input(
                "Enter 'land' to land the drones, 'wait' to wait at current altitude, or 'move' to move to a new location: "
            )
            .strip()
            .lower()
        )
        if sub_action == "land":
            land_drones()
            handle_post_land_actions()
            break
        elif sub_action == "wait":
            wait_time = int(input("Enter wait time in seconds: "))
            time.sleep(wait_time)
        elif sub_action == "move":
            handle_move_action(drones)
        else:
            print("Invalid action. Please enter 'land', 'wait', or 'move'.")


def land_drones():
    for i, drone in drones.items():
        try:
            print(f"Landing drone {i+1}...")
            land(drone)
        except Exception as e:
            print(f"Failed to land drone {i+1}: {e}")


def land(drone):
    """
    Command the drone to land and wait until it has landed.

    Args:
    drone: The drone instance to command.
    """
    print("Landing...")
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    # Wait until the vehicle is landed (altitude is approximately zero)
    while True:
        msg = drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        altitude = msg.relative_alt / 1000.0  # altitude in meters
        print("Altitude:", altitude)
        if altitude <= 0.1:
            print("Landed")
            break
        time.sleep(0.5)


def disconnect_drones():
    for i, drone in drones.items():
        drone.close()
    print("All drones disconnected")


def main():
    connect_drones()

    if not drones:
        print("No drones connected. Exiting.")
        return

    while True:
        action = (
            input(
                "Enter 'arm' to arm all drones, 'disconnect' to disconnect all drones: "
            )
            .strip()
            .lower()
        )

        if action == "disconnect":
            disconnect_drones()
            break
        elif action == "arm":
            arm_drones(drones)
            break
        else:
            print("Invalid action. Please enter 'arm' or 'disconnect'.")

    while True:
        action = (
            input(
                "Enter 'disarm' to disarm all drones, 'takeoff' to take off, or 'move' to move all drones: "
            )
            .strip()
            .lower()
        )

        if action == "disarm":
            disarm_drones(drones)
            break
        elif action == "takeoff":
            target_altitude = float(input("Enter the target altitude for takeoff: "))
            execute_takeoff(target_altitude)
            handle_post_takeoff_actions()
            break
        elif action == "move":
            handle_move_action(drones)
            handle_post_move_actions()
            break
        else:
            print("Invalid action. Please enter 'disarm', 'takeoff', or 'move'.")


if __name__ == "__main__":
    main()
