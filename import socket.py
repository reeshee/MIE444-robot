import socket
import threading
from rplidar import RPLidar
import pickle
import serial

def lidar_scanner(lidar, shared_data):
    """Continuously updates the shared_data dictionary with the latest scan."""
    for scan in lidar.iter_scans():
        data = [(angle, distance) for (_, angle, distance) in scan]
        shared_data['latest_scan'] = data

def client_handler(client_socket, shared_data, arduino_serial):
    """Handles client connections by processing requests."""
    try:
        # Receive the request type (4 bytes)
        request_type_data = client_socket.recv(4)
        if not request_type_data:
            return
        request_type = int.from_bytes(request_type_data, 'big')

        if request_type == 1:
            # Handle LIDAR scan requests (existing code)
            # Receive the scan limit from the client (4-byte integer)
            data = client_socket.recv(4)
            if not data:
                return
            scan_limit = int.from_bytes(data, 'big')

            # Collect the specified number of scans
            scans_to_send = []
            for _ in range(scan_limit):
                # Wait until a new scan is available
                while 'latest_scan' not in shared_data:
                    pass
                scans_to_send.append(shared_data['latest_scan'])

            # Serialize and send the scans
            serialized_data = pickle.dumps(scans_to_send)
            data_length = len(serialized_data)

            # Send the length and the data
            client_socket.sendall(data_length.to_bytes(4, 'big'))
            client_socket.sendall(serialized_data)

        elif request_type == 2:
            # Handle incoming command
            cmd_length_data = client_socket.recv(4)
            if not cmd_length_data:
                return
            cmd_length = int.from_bytes(cmd_length_data, 'big')

            # Receive the command string
            cmd_data = b''
            while len(cmd_data) < cmd_length:
                packet = client_socket.recv(cmd_length - len(cmd_data))
                if not packet:
                    break
                cmd_data += packet

            command = cmd_data.decode('utf-8').strip()

            # Send the command directly to the Arduino over serial
            arduino_serial.write((command + '\n').encode())  # Add newline to indicate end of command
            print(f"Sent command to Arduino: {command}")
        else:
            print("Unknown request type")
    finally:
        client_socket.close()

# Setup RPLidar
lidar = RPLidar('/dev/ttyUSB0')  # Adjust the port as needed

# Shared data dictionary to hold the latest scan
shared_data = {}

# Initialize serial connection to Arduino
try:
    arduino_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Adjust port and baud rate as needed
except serial.SerialException:
    print("Failed to connect to Arduino on /dev/ttyACM0. Check the port and try again.")
    exit(1)

# Start the LIDAR scanning thread
lidar_thread = threading.Thread(target=lidar_scanner, args=(lidar, shared_data))
lidar_thread.daemon = True
lidar_thread.start()

# Setup server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 8888))
server_socket.listen(5)
print("Server is listening on port 8888...")

try:
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Accepted connection from {addr}")
        # Start a new thread to handle the client
        handler_thread = threading.Thread(target=client_handler, args=(client_socket, shared_data, arduino_serial))
        handler_thread.start()
except KeyboardInterrupt:
    print("Stopping server...")
finally:
    lidar.stop()
    lidar.disconnect()
    server_socket.close()
    arduino_serial.close()
    print("Server stopped.")
