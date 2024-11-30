import socket
import pickle
from TCP_Socket import get_all_target_distances
import time
import math

raspberry_pi_ip = '172.20.10.3'     # Raspberry Pi's IP address when connected to Baqir's iPhone hotspot
# raspberry_pi_ip = '100.67.145.12'  # Raspberry Pi's IP address when connected to UofT wifi
port = 8888

def get_latest_scans(server_ip, port, scan_limit):
    """Connects to the server and retrieves the specified number of scans."""
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, port))
    try:
        # Send request type (1 for scan)
        client_socket.sendall((1).to_bytes(4, 'big'))
        
        # Send the scan limit as a 4-byte integer
        client_socket.sendall(scan_limit.to_bytes(4, 'big'))

        # Receive the length of the incoming data
        data_length_bytes = client_socket.recv(4)
        if not data_length_bytes:
            return None
        data_length = int.from_bytes(data_length_bytes, 'big')

        # Receive the actual data
        data = b''
        while len(data) < data_length:
            packet = client_socket.recv(data_length - len(data))
            if not packet:
                break
            data += packet

        # Deserialize the list of scans
        scans = pickle.loads(data)
        return scans
    finally:
        client_socket.close()



def scan_rplidar(scan_limit=60, scans_per_average=5):
    """
    Retrieves and processes RPLIDAR scans.
    
    Args:
        scan_limit (int): Total number of scans to retrieve.
        scans_per_average (int): Number of scans to average over for each target angle.
        
    Returns:
        list: Averaged distances for each of the 60 target angles.
    """
    print(f"Requesting {scan_limit} scans...")
    scans = get_latest_scans(raspberry_pi_ip, port, scan_limit)
    if scans:
        # Process the target angles from the retrieved scans
        target_scans = get_all_target_distances(scans, scans_per_average)
        # Convert the dictionary to a list sorted by target angle
        sorted_angles = sorted(target_scans.keys())
        rpl_scans = [target_scans[angle] for angle in sorted_angles]
        return rpl_scans
    else:
        print("Failed to get scan data.")
        # Return a list with default distances if scans failed
        return [93.75 for _ in range(60)]



def send_command(server_ip, port, command):
    """Sends a command to the Raspberry Pi and retrieves the ultrasonic data."""
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, port))
    try:
        # Send request type (2 for command)
        client_socket.sendall((2).to_bytes(4, 'big'))

        # Encode the command and send its length
        command_data = command.encode('utf-8')
        cmd_length = len(command_data)
        client_socket.sendall(cmd_length.to_bytes(4, 'big'))

        # Send the command data
        client_socket.sendall(command_data)

        # Receive the length of the response (4 bytes)
        response_length_data = client_socket.recv(4)
        if not response_length_data or len(response_length_data) < 4:
            raise ValueError("Incomplete response length received.")

        response_length = int.from_bytes(response_length_data, 'big')

        # Receive the actual response based on the length
        response_data = b''
        while len(response_data) < response_length:
            packet = client_socket.recv(response_length - len(response_data))
            if not packet:
                break
            response_data += packet

        if len(response_data) < response_length:
            raise ValueError("Incomplete response data received.")

        response = response_data.decode('utf-8')  # Decode the response

        # Return the response to the caller
        return response

    finally:
        client_socket.close()




###### Send a command to the Raspberry Pi ######
### "obs_moveBackward", "obs_rotateRight", "obs_rotateLeft", "obs_smallrotateRight", "obs_smallrotateLeft"

# send_command(raspberry_pi_ip, port, command="obs_moveForward")

# command = f'rotate_left: {abs(int(90))}\n'
# command = f'move_forward: {abs(int(12))}\n'
# send_command(raspberry_pi_ip, port, command)
# time.sleep(2)
# command = f'rotate_right: {abs(int(90))}\n'
# send_command(raspberry_pi_ip,port,command)


###### US Reading ######
# response = send_command(raspberry_pi_ip, port, command="obs_smallrotateRight")
# time.sleep(0.1)
# response = float(response)
# if response > 0:
#    print(response)

##### Read RPLiDAR Scans #####
# robot_readings = scan_rplidar()
# print(robot_readings)

# --------------------------------------------------

# pfdistances = []
# pfangles = []
# l = 0
# for scan in robot_readings:     # Grab robot distances and angles to compare with partilces in localization
#     if scan is not None:
#         pfdistances.append(scan/25.4)
#         pfangles.append(l*6)
#     l += 1

# pfdistances = pfdistances[0::3]
# pfangles = pfangles[0::3]

# m = 0
# for i in pfdistances:
#     print(f"{pfangles[m]}: {pfdistances[m]}")
#     m += 1

# print("DISTANCES",pfdistances)
# print("ANGLES",pfangles)

# --------------------------------------------------
# Initialize sector variables to None
# sensor_back = None
# sensor_backl = None
# sensor_left = None
# sensor_frontl = None
# sensor_front = None
# sensor_frontr = None
# sensor_right = None
# sensor_backr = None

# k = 0

# # Define sector boundaries in terms of scan indices

# SECTOR_DEFINITIONS = {
#     'sensor_front': list(range(0, 3)) + list(range(57, 60)),    # 0°-18° and 342°-360° (indices 0-3 and 56-59)
#     'sensor_frontr': list(range(3, 12)),                        # 18°-72°   (indices 3-12)
#     'sensor_right': list(range(12, 18)),                        # 72°-108°  (indices 12-18)
#     'sensor_backr': list(range(18, 27)),                        # 108°-162° (indices 18-27)
#     'sensor_back': list(range(27, 33)),                         # 162°-198° (indices 27-33)
#     'sensor_backl': list(range(33, 42)),                        # 198°-252° (indices 33-42)
#     'sensor_left': list(range(42, 48)),                         # 252°-288° (indices 42-48)
#     'sensor_frontl': list(range(48, 57))                        # 288°-360° (indices 48-59)
# }
    
# for scan in robot_readings:
#     if scan is not None:
#         # Determine which section the current scan belongs to
#         for sector, indices in SECTOR_DEFINITIONS.items():
#             if k in indices:
#                 # Assign the minimum distance for the section
#                 if sector == 'sensor_back':
#                     if sensor_back is None or scan < sensor_back:
#                         sensor_back = scan
#                         # ab = k*6
#                 elif sector == 'sensor_backl':
#                     if sensor_backl is None or scan < sensor_backl:
#                         sensor_backl = scan
#                         # abl = k*6
#                 elif sector == 'sensor_left':
#                     if sensor_left is None or scan < sensor_left:
#                         sensor_left = scan
#                         # al = k*6
#                 elif sector == 'sensor_frontl':
#                     if sensor_frontl is None or scan < sensor_frontl:
#                         sensor_frontl = scan
#                         # afl = k*6
#                 elif sector == 'sensor_front':
#                     if sensor_front is None or scan < sensor_front:
#                         sensor_front = scan
#                         # af = k*6
#                 elif sector == 'sensor_frontr':
#                     if sensor_frontr is None or scan < sensor_frontr:
#                         sensor_frontr = scan
#                         # afr = k*6
#                 elif sector == 'sensor_backr':
#                     if sensor_backr is None or scan < sensor_backr:
#                         sensor_backr = scan
#                         # abr = k*6
#                 elif sector == 'sensor_right':
#                     if sensor_right is None or scan < sensor_right:
#                         sensor_right = scan
#                         # ar = k*6
#                 break
#     k += 1
    
# if sensor_back is None:
#     sensor_back = 93.75
# if sensor_backl is None:
#     sensor_backl = 93.75
# if sensor_left is None:
#     sensor_left = 93.75
# if sensor_frontl is None:
#     sensor_frontl = 93.75
# if sensor_front is None:
#     sensor_front = 93.75
# if sensor_frontr is None:
#     sensor_frontr = 93.75
# if sensor_right is None:
#     sensor_right = 93.75
# if sensor_backr is None:
#     sensor_backr = 93.75

# print(sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl)
    
#-----------------------------------------------------------------------------------------------


# us_sensor = send_command(raspberry_pi_ip, port, command="obs_smallrotateRight")
# command = f'rotate_left: {abs(int(90))}\n'

# us_sensor = command = f'move_forward: {abs(int(1))}\n'
# send_command(raspberry_pi_ip, port, command)
# scan = float(us_sensor)
# print(scan)

# if scan < 7.0 and sensor_front - scan > 2.0:
#     print("Block Might Be Ahead")
#     rescan = float(send_command(raspberry_pi_ip, port, command="scan"))
#     lidar_rescan = scan_rplidar()[0]

#     if rescan < 3.0 and lidar_rescan - rescan > 2.0:
#         print("BLOCK!!!!!")
            
            
            
            
# us_sensor = send_command(raspberry_pi_ip, port, command="obs_moveForward")
# us_sensor = send_command(raspberry_pi_ip, port, command="obs_moveBackward")

# us_sensor = send_command(raspberry_pi_ip, port, "arm_lower")

# command = f'move_forward: {abs(int(10))}\n'

# command = f'move_forward: {abs(int(15))}\n'
# send_command(raspberry_pi_ip, port, command)

# time.sleep(0.5)

# command = f'rotate_left: {abs(int(170))}\n'
# send_command(raspberry_pi_ip, port, command)

# time.sleep(0.3)

# command = f'move_forward: {abs(int(38))}\n'
# send_command(raspberry_pi_ip, port, command)