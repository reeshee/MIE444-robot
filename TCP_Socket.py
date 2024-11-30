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

# Generate 60 target angles every 6 degrees
TARGET_ANGLES = list(range(0, 360, 6))  # [0, 6, 12, ..., 354]
ANGLE_TOLERANCE = 3  # Degrees tolerance to consider around the target angle
# DEFAULT_DISTANCE = 93.75  # mm, the assumed distance when too close to the wall (Removed)

def normalize_angle(angle):
    """Normalize an angle to be between 0 and 360 degrees."""
    return angle % 360

def get_target_angles(scans):
    """
    Extracts readings at specific target angles by averaging across multiple scans.
    
    Args:
        scans (list of list): Each element is a list of (angle, distance) tuples from a single LIDAR scan.

    Returns:
        dict: A dictionary of averaged distances for each target angle. 
              If no valid readings are found for a target angle, its value is set to None.
    """
    angle_distances = {angle: [] for angle in TARGET_ANGLES}
    
    # Iterate over multiple scans
    for scan in scans:
        # Normalize and sort the scan data by angle
        normalized_scan = [(normalize_angle(angle), distance) for (angle, distance) in scan]
        normalized_scan.sort(key=lambda x: x[0])
        
        for target_angle in TARGET_ANGLES:
            # Find readings within ±3 degrees of the target angle
            lower_bound = normalize_angle(target_angle - ANGLE_TOLERANCE)
            upper_bound = normalize_angle(target_angle + ANGLE_TOLERANCE)
            
            closest_points = []
            for angle, distance in normalized_scan:
                # Handle angle wrap-around
                if lower_bound <= upper_bound:
                    in_sector = lower_bound <= angle <= upper_bound
                else:
                    in_sector = angle >= lower_bound or angle <= upper_bound
                
                if in_sector:
                    if distance is not None and distance > 10:  # Assuming 10 mm as minimum valid distance
                        closest_points.append(distance)
            
            if closest_points:
                min_distance = min(closest_points)
                angle_distances[target_angle].append(min_distance)
            else:
                # No valid readings found, assign None
                angle_distances[target_angle].append(None)
    
    # Calculate the average minimum distance for each target angle across all scans
    averaged_distances = {}
    for target_angle, distances in angle_distances.items():
        # Filter out None values
        valid_distances = [d for d in distances if d is not None]
        if valid_distances:
            averaged_distance = sum(valid_distances) / len(valid_distances)
            averaged_distances[target_angle] = averaged_distance
        else:
            # If all readings are None, assign None
            averaged_distances[target_angle] = None

    return averaged_distances

# Assuming 60 scans with each scan representing 6-degree increments
def get_all_target_distances(scans, scans_per_average=5):
    """
    Processes scans to compute averaged minimum distances for each target angle.
    
    Args:
        scans (list of list): List containing multiple scan data.
        scans_per_average (int): Number of scans to average over.
        
    Returns:
        dict: Averaged distances for each target angle. Values can be float or None.
    """
    # Ensure we have enough scans
    if len(scans) < scans_per_average:
        print(f"Not enough scans to average. Required: {scans_per_average}, Available: {len(scans)}")
        return {angle: None for angle in TARGET_ANGLES}
    
    # Only consider the last 'scans_per_average' scans for averaging
    recent_scans = scans[-scans_per_average:]
    target_distances = get_target_angles(recent_scans)
    
    return target_distances
