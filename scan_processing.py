import socket
import pickle
from TCP_Socket import get_target_angles

raspberry_pi_ip = '172.20.10.3'     # Raspberry Pi's IP address when connected to Baqir's iPhone hotspot
#raspberry_pi_ip = '100.67.157.87'  # Raspberry Pi's IP address when connected to UofT wifi
port = 8888

def get_latest_scans(server_ip, port, scan_limit):
    """Connects to the server and retrieves the specified number of scans."""
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, port))
    try:
        # Send request type (1 for command)
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



def scan_rplidar(scan_limit=5):
    print(f"Requesting {scan_limit} scans...")
    scans = get_latest_scans(raspberry_pi_ip, port, scan_limit)
    if scans:
        # Process the target angles from the retrieved scans
        target_scans = get_target_angles(scans)
        print("Averaged Target Scans:", target_scans)
        rpl_scans = [*target_scans.values()]
        #print(rpl_scans)
    else:
        print("Failed to get scan data.")
    return rpl_scans



def send_command(server_ip, port, command):
    """Sends a command to the Raspberry Pi."""
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
    finally:
        client_socket.close()


# Send a command to the Raspberry Pi
#command = "w0:10"
#send_command(raspberry_pi_ip, port, command)

# Read RPLiDAR Scans
rpl = scan_rplidar()
#sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr = rpl    # Check robot sensors
print(rpl)
#print(sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr)