import threading
from pyproj import Transformer, CRS
from pymavlink import mavutil
from typing import Tuple, List, Dict

# Define the coordinate systems and transformers
wgs84 = CRS("EPSG:4326")  # WGS84
turef30 = CRS("EPSG:5254")  # TUREF30
lla_to_xyz_transformer = Transformer.from_crs(wgs84, turef30, always_xy=True)
xyz_to_lla_transformer = Transformer.from_crs(turef30, wgs84, always_xy=True)


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


# Conversion functions
def lla_to_xyz(
    latitude: float, longitude: float, altitude: float
) -> Tuple[float, float, float]:
    """
    Converts latitude, longitude, and altitude to XYZ coordinates.
    Uses WGS84 and TUREF30 coordinate systems for the conversion.

    Parameters
    ----------
        latitude (float):
            Latitude in degrees in WGS84 coordinate system
        longitude (float):
            Longitude in degrees in WGS84 coordinate system
        altitude (float):
            Altitude in meters in WGS84 coordinate system

    Returns
    ----------
        Tuple[float, float, float]:
            Tuple containing the X, Y, and Z coordinates in TUREF30 coordinate system
    """
    x, y, z = lla_to_xyz_transformer.transform(longitude, latitude, altitude)
    return x, y, z


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


def request_data_stream(drone, rate=1):
    drone.mav.request_data_stream_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate,
        1,
    )
    print("Requested data stream")


def get_drone_coordinates(
    drone: mavutil.mavlink_connection,
) -> Tuple[float, float, float]:
    """
    Get the current latitude, longitude, and altitude of the drone.

    Args:
        drone (mavutil.mavlink_connection): The drone connection.

    Returns:
        Tuple[float, float, float]: Latitude, longitude, and altitude.
    """
    request_data_stream(drone)
    msg = drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.relative_alt / 1000.0  # Convert from mm to meters
    return (lat, lon, alt)


def get_xyz_coordinates(
    drones: Dict[int, mavutil.mavlink_connection]
) -> List[Tuple[float, float, float]]:
    """
    Get the x, y, z coordinates for each drone.

    Args:
        drones (Dict[int, mavutil.mavlink_connection]): Dictionary of drone connections.

    Returns:
        List[Tuple[float, float, float]]: List of x, y, z coordinates for each drone.
    """
    xyz_coords = []
    for drone in drones.values():
        lat, lon, alt = get_drone_coordinates(drone)
        x, y, z = lla_to_xyz(lat, lon, alt)
        xyz_coords.append((x, y, z))
    return xyz_coords


def calculate_average_coordinates(
    xyz_coords: List[Tuple[float, float, float]]
) -> Tuple[float, float, float]:
    """
    Calculate the average of the x, y, z coordinates.

    Args:
        xyz_coords (List[Tuple[float, float, float]]): List of x, y, z coordinates.

    Returns:
        Tuple[float, float, float]: The average x, y, z coordinates.
    """
    avg_x = sum(coord[0] for coord in xyz_coords) / len(xyz_coords)
    avg_y = sum(coord[1] for coord in xyz_coords) / len(xyz_coords)
    avg_z = sum(coord[2] for coord in xyz_coords) / len(xyz_coords)
    return avg_x, avg_y, avg_z


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
