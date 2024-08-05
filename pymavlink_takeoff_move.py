from pymavlink import mavutil
import time
import serial.tools.list_ports
import threading

from pymavlink_move import (
    handle_move_action,
    get_xyz_coordinates,
    calculate_average_coordinates,
)

from pyproj import Transformer, CRS
from pymavlink import mavutil
from typing import Tuple, List, Dict

drones = {}

wgs84 = CRS("EPSG:4326")  # WGS84
turef30 = CRS("EPSG:5254")  # TUREF30

xyz_to_lla_transformer = Transformer.from_crs(turef30, wgs84, always_xy=True)


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
            time.sleep(0.3)
            # Set the mode to GUIDED
            set_mode(drones[i], "GUIDED")
            print("Mode has been set to GUIDED")
        except Exception as e:
            raise (f"Failed to connect to drone {i+1} on {port}: {e}")
    xyz_coords = get_xyz_coordinates(drones)
    print(xyz_coords)
    avg_x, avg_y, avg_z = calculate_average_coordinates(xyz_coords)
    print(avg_x)

    return xyz_coords, avg_x, avg_y, avg_z


def disconnect_drones():
    for i, drone in drones.items():
        drone.close()
    print("All drones disconnected")


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
    drone.mav.param_set_send(
        drone.target_system,
        drone.target_component,
        param_id.encode("utf-8"),  # Encode the parameter ID to bytes
        float(param_value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    # Wait for the parameter to be set
    while True:
        ack = drone.recv_match(type="PARAM_VALUE", blocking=True)
        if ack and ack.param_id == param_id:
            print(f"Parameter {param_id} set to {ack.param_value}")
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


def xyz_to_lla(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Converts XYZ coordinates to latitude, longitude, and altitude.
    Uses TUREF30 and WGS84 coordinate systems for the conversion.

    Parameters
    ----------
        x (float):
            X coordinate in TUREF30 coordinate system
        y (float):
            Y coordinate in TUREF30 coordinate system
        z (float):
            Z coordinate in TUREF30 coordinate system

    Returns
    ----------
        Tuple[float, float, float]:
            Tuple containing the latitude, longitude, and altitude in WGS84 coordinate system
    """
    longitude, latitude, altitude = xyz_to_lla_transformer.transform(x, y, z)
    return latitude, longitude, altitude


def send_position_target_global_int(
    drone: mavutil.mavlink_connection,
    lat: float,
    lon: float,
    alt: float,
    vx: float = 0,
    vy: float = 0,
    vz: float = 0,
    yaw: float = 0,
    yaw_rate: float = 0,
) -> None:
    """
    Send a position target message to the drone using global coordinates.

    Args:
        drone (mavutil.mavlink_connection): The drone connection.
        lat (float): Latitude in degrees.
        lon (float): Longitude in degrees.
        alt (float): Altitude in meters.
        vx (float): X velocity in m/s.
        vy (float): Y velocity in m/s.
        vz (float): Z velocity in m/s.
        yaw (float): Yaw angle in radians.
        yaw_rate (float): Yaw rate in radians/second.
    """
    drone.mav.send(
        drone.mav.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0,
            0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            int(lat * 1e7),
            int(lon * 1e7),
            alt,  # lat, lon, alt
            vx,
            vy,
            vz,  # x, y, z velocity in m/s (not used)
            0,
            0,
            0,  # afx, afy, afz acceleration (not used)
            yaw,
            yaw_rate,  # yaw, yaw_rate (not used)
        )
    )


def move_drones(
    drone: mavutil.mavlink_connection, target_coordinates: Tuple[float, float, float]
) -> None:
    """
    Move a drone to the target coordinates.

    Args:
        drone (mavutil.mavlink_connection): The drone connection.
        target_coordinates (Tuple[float, float, float]): Target coordinates (x, y, z).
    """
    x, y, z = target_coordinates
    lat, lon, alt = xyz_to_lla(x, y, z)
    send_position_target_global_int(drone, lat, lon, alt)


def handle_move_action(
    drones: Dict[int, mavutil.mavlink_connection], xyz_coords, avg_x, avg_y, avg_z
) -> None:
    """
    Handle the move action for multiple drones.

    Args:
        drones (Dict[int, mavutil.mavlink_connection]): Dictionary of drone connections.
    """
    target_coordinates_input = tuple(
        map(
            float, input("Enter the target coordinates for move (X, Y, Z): ").split(",")
        )
    )

    print("Current coordinates of each drone:")
    for i, (x, y, z) in enumerate(xyz_coords, start=1):
        lat, lon, alt = xyz_to_lla(x, y, z)
        print(f"Drone {i}: Latitude={lat}, Longitude={lon}, Altitude={alt}")

    print(f"\nCalculated average coordinates (absolute): ({avg_x}, {avg_y}, {avg_z})")

    target_coordinates = (
        avg_x + target_coordinates_input[0],
        avg_y + target_coordinates_input[1],
        avg_z + target_coordinates_input[2],
    )

    print(f"\nTarget coordinates (relative to average): {target_coordinates_input}")
    print(f"Final target coordinates (absolute): {target_coordinates}")

    threads = []
    for i, drone in drones.items():
        try:
            print(f"Moving drone {i+1} to {target_coordinates}...")
            t = threading.Thread(target=move_drones, args=(drone, target_coordinates))
            threads.append(t)
            t.start()
        except Exception as e:
            print(f"Failed to move drone {i+1}: {e}")


def main():
    xyz_cords, avg_x, avg_y, avg_z = connect_drones()

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
            input("Enter 'disarm' to disarm all drones, 'takeoff' to take off: ")
            .strip()
            .lower()
        )

        if action == "disarm":
            disarm_drones(drones)
            break
        elif action == "takeoff":
            target_altitude = float(input("Enter the target altitude for takeoff: "))
            execute_takeoff(target_altitude)
            while True:
                sub_action = (
                    input(
                        "Enter 'land' to land the drones, 'wait' to wait at current altitude, 'move' to move the drones: "
                    )
                    .strip()
                    .lower()
                )
                if sub_action == "land":
                    land_drones()
                    break
                elif sub_action == "wait":
                    wait_time = int(input("Enter wait time in seconds: "))
                    time.sleep(wait_time)
                elif sub_action == "move":
                    handle_move_action(
                        drones,
                        xyz_coords=xyz_cords,
                        avg_x=avg_x,
                        avg_y=avg_y,
                        avg_z=avg_z,
                    )
                    land_drones()
                    disarm_drones(drones)
                    disconnect_drones()
                    break

                else:
                    print("Invalid action. Please enter 'land' or 'wait'.")
            break
        else:
            print("Invalid action. Please enter 'disarm', 'takeoff', or 'move'.")


if __name__ == "__main__":
    main()
