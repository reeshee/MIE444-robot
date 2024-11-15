import pickle
import numpy as np

def get_rplidar_scans(client_socket, scan_limit):
    """
    Requests a specific number of scans from the RPLIDAR server.
    
    Args:
        client_socket (socket.socket): The socket connected to the server.
        scan_limit (int): The number of scans to retrieve.
        
    Returns:
        list: A list of scan data, each entry containing angle-distance pairs.
    """
    scan_data_collection = []

    # Send the scan limit as a 4-byte integer
    client_socket.sendall(scan_limit.to_bytes(4, 'big'))

    try:
        for _ in range(scan_limit):
            # Receive the length of the incoming data
            data_length_bytes = client_socket.recv(4)
            if not data_length_bytes:
                break
            data_length = int.from_bytes(data_length_bytes, 'big')
            
            # Now, receive the actual data based on the length
            data = b''
            while len(data) < data_length:
                packet = client_socket.recv(data_length - len(data))
                if not packet:
                    break
                data += packet
            
            # Deserialize the data after fully receiving it
            scan_data = pickle.loads(data)
            scan_data_collection.append(scan_data)  # Collect each scan data
    except KeyboardInterrupt:
        print("Client stopped.")
    
    return scan_data_collection



TARGET_ANGLES = [0, 30, 90, 150, 180, 210, 270, 330]
ANGLE_TOLERANCE = 5  # Degrees tolerance to consider around the target angle
DEFAULT_DISTANCE = 90.375  # cm, the assumed distance when too close to the wall

def normalize_angle(angle):
    """Normalize an angle to be between 0 and 360 degrees."""
    return angle % 360

def get_target_angles(scans):
    """
    Extracts readings at specific target angles by averaging across multiple scans.
    
    Args:
        scans (list of list): Each element is a list of (angle, distance) tuples from a single LIDAR scan.

    Returns:
        dict: A dictionary of distances for each target angle.
    """
    angle_distances = {angle: [] for angle in TARGET_ANGLES}
    
    # Iterate over multiple scans
    for scan in scans:
        # Create a sorted list of angles and distances from the current scan
        scan = [(normalize_angle(angle), distance) for (angle, distance) in scan]  # Normalize angles
        scan.sort(key=lambda x: x[0])  # Ensure the scan is sorted by angle
        
        for target_angle in TARGET_ANGLES:
            # Find readings close to the target angle
            closest_points = []
            for angle, distance in scan:
                # Calculate the difference considering the circular nature of angles
                diff = min(abs(target_angle - angle), abs((target_angle - 360) - angle), abs((target_angle + 360) - angle))
                if diff <= ANGLE_TOLERANCE:
                    closest_points.append((angle, distance))
            
            # If we found valid points within the tolerance, pick the minimum distance
            if closest_points:
                min_distance = min([distance for angle, distance in closest_points if distance > 10])
                angle_distances[target_angle].append(min_distance)
            else:
                # No valid readings found, use default distance
                angle_distances[target_angle].append(DEFAULT_DISTANCE)
    
    # Calculate the average distance for each target angle across all scans
    averaged_distances = {}
    for target_angle, distances in angle_distances.items():
        if distances:
            averaged_distances[target_angle] = sum(distances) / len(distances)
        else:
            averaged_distances[target_angle] = DEFAULT_DISTANCE

    return averaged_distances