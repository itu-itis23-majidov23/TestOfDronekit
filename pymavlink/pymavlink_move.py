import threading
from pyproj import Transformer, CRS
from pymavlink import mavutil
from typing import Tuple, List

# Define the coordinate systems
wgs84 = CRS("EPSG:4326")  
turef30 = CRS("EPSG:5254")  

# Create transformers for coordinate conversion
lla_to_xyz_transformer = Transformer.from_crs(wgs84, turef30, always_xy=True)
xyz_to_lla_transformer = Transformer.from_crs(turef30, wgs84, always_xy=True)

# Conversion functions
def lla_to_xyz(latitude: float, longitude: float, altitude: float) -> Tuple[float, float, float]: 
    x, y, z = lla_to_xyz_transformer.transform(longitude, latitude, altitude)
    return x, y, z

def xyz_to_lla(x: float, y: float, z: float) -> Tuple[float, float, float]:
    longitude, latitude, altitude = xyz_to_lla_transformer.transform(x, y, z)
    return latitude, longitude, altitude

def get_drone_coordinates(drone: mavutil.mavlink_connection) -> Tuple[float, float, float]:
    """
    Retrieve the current GPS coordinates of the drone.

    Args:
        drone (mavutil.mavlink_connection): The drone connection.

    Returns:
        Tuple[float, float, float]: The latitude, longitude, and altitude of the drone.
    """
    # Request GPS position
    msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    latitude = msg.lat / 1e7
    longitude = msg.lon / 1e7
    altitude = msg.alt / 1e3
    return latitude, longitude, altitude

def get_xyz_coordinates(drones: List[mavutil.mavlink_connection]) -> List[Tuple[float, float, float]]:
    """
    Get the x, y, z coordinates for each drone.

    Args:
        drones (List[mavutil.mavlink_connection]): List of drone connections.

    Returns:
        List[Tuple[float, float, float]]: List of x, y, z coordinates for each drone.
    """
    xyz_coords = []
    for drone in drones:
        lat, lon, alt = get_drone_coordinates(drone)
        x, y, z = lla_to_xyz(lat, lon, alt)
        xyz_coords.append((x, y, z))
    return xyz_coords

def calculate_average_coordinates(xyz_coords: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
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

def move_drones(drone, target_coordinates):
    """
    Move a drone to the target coordinates.
    
    Args:
        drone (mavutil.mavlink_connection): The drone connection.
        target_coordinates (Tuple[float, float, float]): Target coordinates (x, y, z).
    """
    x, y, z = target_coordinates
    lat, lon, alt = xyz_to_lla(x, y, z)
    send_position_target_global_int(drone, lat, lon, alt)

def send_position_target_global_int(drone, lat, lon, alt, vx=0, vy=0, vz=0, yaw=0, yaw_rate=0):
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
    drone.mav.send(drone.mav.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        int(lat * 1e7), int(lon * 1e7), alt, # lat, lon, alt
        vx, vy, vz, # x, y, z velocity in m/s (not used)
        0, 0, 0, # afx, afy, afz acceleration (not used)
        yaw, yaw_rate # yaw, yaw_rate (not used)
    ))

def handle_move_action(drones):
    xyz_coords = get_xyz_coordinates(drones)
    avg_x, avg_y, avg_z = calculate_average_coordinates(xyz_coords)
    
    target_coordinates_input = tuple(map(float, input("Enter the target coordinates for move (X, Y, Z): ").split(',')))
    target_coordinates = (avg_x + target_coordinates_input[0], avg_y + target_coordinates_input[1], avg_z + target_coordinates_input[2])
    
    threads = []
    for i, drone in drones.items():
        try:
            print(f"Moving drone {i+1} to {target_coordinates}...")
            t = threading.Thread(target=move_drones, args=(drone, target_coordinates))
            threads.append(t)
            t.start()
        except Exception as e:
            print(f"Failed to move drone {i+1}: {e}")
