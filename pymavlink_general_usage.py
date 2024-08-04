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
        ack_type = ack.type if ack else None
        ack_result = ack.result if ack else None
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if ack_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Mode change to {mode} accepted")
            else:
                print(f"Mode change to {mode} failed with result {ack_result}")
            break


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


def arm_drones(drones):
    for i, drone in drones.items():
        try:
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
        print(f"Disarming drone {i+1}...")
        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        drone.motors_disarmed_wait()
        print(f"Drone {i+1} disarmed")
    print("All drones disarmed")


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
            for i, drone in drones.items():
                drone.close()
            print("All drones disconnected")
            break

        elif action == "arm":
            arm_drones(drones)
            break
        else:
            print("Invalid action. Please enter 'arm' or 'disconnect'.")

    while True:
        action = (
            input(
                "Enter 'disarm' to disarm all drones, 'takeoff' to take off and 'move' to move all drones: "
            )
            .strip()
            .lower()
        )

        if action == "disarm":
            disarm_drones(drones)
            break

        ### To be solved quickly in case of an error (Do not extract for now!)
        elif action == "move":
            handle_move_action(drones)

            while True:
                sub_action = (
                    input(
                        "Enter 'land' to land the drones, 'wait' to wait at current altitude or 'move' to move a new location: "
                    )
                    .strip()
                    .lower()
                )

                if sub_action == "land":
                    for i, drone in drones.items():
                        try:
                            print(f"Landing drone {i+1}...")
                            land(drone)
                        except Exception as e:
                            print(f"Failed to land drone {i+1}: {e}")

                    sub_action = (
                        input(
                            "Enter 'disarm' to disarm the drones, 'takeoff' to take off again: "
                        )
                        .strip()
                        .lower()
                    )
                    if sub_action == "disarm":
                        disarm_drones(drones)
                        break
                    elif sub_action == "takeoff":
                        target_altitude = float(
                            input("Enter the target altitude for takeoff: ")
                        )

                        # Create and start a new thread for each drone takeoff
                        threads = []
                        for i, drone in drones.items():
                            try:
                                print(f"Taking off drone {i+1}...")
                                t = threading.Thread(
                                    target=takeoff, args=(drone, target_altitude)
                                )
                                threads.append(t)
                                t.start()
                            except Exception as e:
                                print(f"Failed to take off drone {i+1}: {e}")

                        # Wait for all threads to complete
                        for t in threads:
                            t.join()

                elif sub_action == "wait":
                    wait_time = int(input("Enter wait time in seconds: "))
                    time.sleep(wait_time)

                elif sub_action == "move":
                    handle_move_action(drones)
                ### To be solved quickly in case of an error (Do not extract for now!)

        elif action == "takeoff":
            target_altitude = float(input("Enter the target altitude for takeoff: "))

            # Create and start a new thread for each drone takeoff
            threads = []
            for i, drone in drones.items():
                try:
                    print(f"Taking off drone {i+1}...")
                    t = threading.Thread(target=takeoff, args=(drone, target_altitude))
                    threads.append(t)
                    t.start()
                except Exception as e:
                    print(f"Failed to take off drone {i+1}: {e}")

            # Wait for all threads to complete
            for t in threads:
                t.join()

            while True:
                sub_action = (
                    input(
                        "Enter 'land' to land the drones, 'wait' to wait at current altitude: "
                    )
                    .strip()
                    .lower()
                )

                if sub_action == "land":
                    for i, drone in drones.items():
                        try:
                            print(f"Landing drone {i+1}...")
                            land(drone)
                        except Exception as e:
                            print(f"Failed to land drone {i+1}: {e}")

                    sub_action = (
                        input(
                            "Enter 'disarm' to disarm the drones, 'takeoff' to take off again: "
                        )
                        .strip()
                        .lower()
                    )
                    if sub_action == "disarm":
                        disarm_drones(drones)
                        break
                    elif sub_action == "takeoff":
                        target_altitude = float(
                            input("Enter the target altitude for takeoff: ")
                        )

                        # Create and start a new thread for each drone takeoff
                        threads = []
                        for i, drone in drones.items():
                            try:
                                print(f"Taking off drone {i+1}...")
                                t = threading.Thread(
                                    target=takeoff, args=(drone, target_altitude)
                                )
                                threads.append(t)
                                t.start()
                            except Exception as e:
                                print(f"Failed to take off drone {i+1}: {e}")

                        # Wait for all threads to complete
                        for t in threads:
                            t.join()

                elif sub_action == "wait":
                    wait_time = int(input("Enter wait time in seconds: "))
                    time.sleep(wait_time)
        else:
            print("Invalid action. Please enter 'disarm' or 'takeoff'.")


if __name__ == "__main__":
    main()
