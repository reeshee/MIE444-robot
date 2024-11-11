'''
This file is part of SimMeR, an educational mechatronics robotics simulator.
Initial development funded by the University of Toronto MIE Department.
Copyright (C) 2023  Ian G. Bennett

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

# Basic client for sending and receiving data to SimMeR or a robot, for testing purposes
# Some code modified from examples on https://realpython.com/python-sockets/
# and https://www.geeksforgeeks.org/python-display-text-to-pygame-window/

# If using a bluetooth low-energy module (BT 4.0 or higher) such as the HM-10, the ble-serial
# package (https://github.com/Jakeler/ble-serial) is necessary to directly create a serial
# connection between a computer and the device. If using this package, the BAUDRATE constant
# should be left as the default 9600 bps.

import socket
import time
from datetime import datetime
import serial

# Wrapper functions
def transmit(data):
    '''Selects whether to use serial or tcp for transmitting.'''
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)

def receive():
    '''Selects whether to use serial or tcp for receiving.'''
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()

# TCP communication functions
def transmit_tcp(data):
    '''Send a command over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
        except TimeoutError:
            print('Tx socket timed out.')
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')

def receive_tcp():
    '''Receive a reply over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                # return the data received as well as the current time
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')

# Serial communication functions
def transmit_serial(data):
    '''Transmit a command over a serial connection.'''
    clear_serial()
    SER.write(data.encode('ascii'))

def receive_serial():
    '''Receive a reply over a serial connection.'''

    start_time = time.time()
    response_raw = ''
    while time.time() < start_time + TIMEOUT_SERIAL:
        if SER.in_waiting:
            response_char = SER.read().decode('ascii')
            if response_char == FRAMEEND:
                response_raw += response_char
                break
            else:
                response_raw += response_char

    print(f'Raw response was: {response_raw}')

    # If response received, return it
    if response_raw:
        return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]

def clear_serial(delay_time: float = 0):
    '''Wait some time (delay_time) and then clear the serial buffer.'''
    if SER.in_waiting:
        time.sleep(delay_time)
        print(f'Clearing Serial... Dumped: {SER.read(SER.in_waiting)}')

# Packetization and validation functions
def depacketize(data_raw: str):
    '''
    Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
    '''

    # Locate start and end framing characters
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    # Check that the start and end framing characters are present, then return commands as a list
    if (start >= 0 and end >= start):
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

        # Make sure this list is formatted in the expected manner
        for cmd_single in cmd_list:
            match len(cmd_single):
                case 0:
                    cmd_single.append('')
                    cmd_single.append('')
                case 1:
                    cmd_single.append('')
                case 2:
                    pass
                case _:
                    cmd_single = cmd_single[0:2]

        return cmd_list
    else:
        return [[False, '']]

def packetize(data: str):
    '''
    Take a message that is to be sent to the command script and packetize it with start and end framing.
    '''

    # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        return FRAMESTART + data + FRAMEEND

    return False

def response_string(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ''
    sgn = ''
    chk = ''
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = '='
            chk = 'âœ“'
        else:
            sgn = '!='
            chk = 'X'

        out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n')

    return out_string

def validate_responses(cmd_list: list, responses_list: list):
    '''
    Validate that the list of commands and received responses have the same command id's. Takes a
    list of commands and list of responses as inputs, and returns a list of true and false values
    indicating whether each id matches.
    '''
    valid = []
    for pair in zip(cmd_list, responses_list):
        if pair[1]:
            if pair[0] == pair[1][0]:
                valid.append(True)
            else:
                valid.append(False)
    return valid

def check_sensors():
    # Check an ultrasonic sensor 'u0'
    transmit(packetize('u0'))
    [responses, time_rx] = receive()
    print(f"Ultrasonic 0 reading: {response_string('u0',responses)}")
    sensor_front = float(responses[0][1])
    
    # Check an ultrasonic sensor 'u1'
    transmit(packetize('u1'))
    [responses, time_rx] = receive()
    print(f"Ultrasonic 1 reading: {response_string('u1',responses)}")
    sensor_right = float(responses[0][1])
    
    # Check an ultrasonic sensor 'u2'
    transmit(packetize('u2'))
    [responses, time_rx] = receive()
    print(f"Ultrasonic 2 reading: {response_string('u2',responses)}")
    sensor_left = float(responses[0][1])
    
    # Check an ultrasonic sensor 'u3'
    transmit(packetize('u3'))
    [responses, time_rx] = receive()
    print(f"Ultrasonic 3 reading: {response_string('u3',responses)}")
    sensor_back = float(responses[0][1])

    # Check an ultrasonic sensor 'u4'
    transmit(packetize('u4'))
    [responses, time_rx] = receive()
    print(f"Ultrasonic 4 reading: {response_string('u4',responses)}")
    sensor_frontl = float(responses[0][1])
    
    # Check an ultrasonic sensor 'u5'
    transmit(packetize('u5'))
    [responses, time_rx] = receive()
    print(f"Ultrasonic 5 reading: {response_string('u5',responses)}")
    sensor_frontr = float(responses[0][1])
    
    return sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr

############## Constant Definitions Begin ##############
### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM3'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = True



############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = 'SimMeR'
else:
    SOURCE = 'serial device ' + PORT_SERIAL
try:
    SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
except serial.SerialException:
    print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0





############## Main section for the open loop control algorithm ##############
LOOP_PAUSE_TIME = 0.1 # seconds
# Main loop
RUNNING = True # If true, run this. If false, skip it
CMD_LIST = ['w0:1.2', 'r0:-10', 'r0:10','w0:-0.5']
threshold = 10.5
diag_threshold = 9
#cl = True

while RUNNING:
    # Pause for a little while so as to not spam commands insanely fast
    time.sleep(LOOP_PAUSE_TIME)

    sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr = check_sensors()

    #while cl:
    if sensor_front > threshold and sensor_frontr > diag_threshold and sensor_frontl > diag_threshold:
        # Send a drive forward command
        transmit(packetize(CMD_LIST[0]))
        time.sleep(LOOP_PAUSE_TIME)
        [responses, time_rx] = receive()
        print(f"Drive command response: {response_string('w0:1.2',responses)}")
    elif sensor_left > sensor_right and sensor_left > sensor_front:
        # Send a turn left command
        transmit(packetize(CMD_LIST[1]))
        time.sleep(LOOP_PAUSE_TIME)
        transmit(packetize(CMD_LIST[0]))
        [responses, time_rx] = receive()
        print(f"Drive command response: {response_string('r0:-10',responses)}")
    elif sensor_left > sensor_right and sensor_left < sensor_front:
        # Send a drive forward command w left correction
        transmit(packetize(CMD_LIST[1]))
        time.sleep(LOOP_PAUSE_TIME)
        transmit(packetize(CMD_LIST[0]))
        [responses, time_rx] = receive()
        print(f"Drive command response: {response_string('w0:1',responses)}")
    elif sensor_right > sensor_left and sensor_right > sensor_front:
        # Send a turn right command
        transmit(packetize(CMD_LIST[2]))
        time.sleep(LOOP_PAUSE_TIME)
        transmit(packetize(CMD_LIST[0]))
        [responses, time_rx] = receive()
        print(f"Drive command response: {response_string('r0:10',responses)}")
    elif sensor_right > sensor_left and sensor_right < sensor_front:
        # Send a drive forward command w right correction
        transmit(packetize(CMD_LIST[2]))
        time.sleep(LOOP_PAUSE_TIME)
        transmit(packetize(CMD_LIST[0]))
        [responses, time_rx] = receive()
        print(f"Drive command response: {response_string('w0:1',responses)}")
    else:
        # Send a drive forward command
        transmit(packetize(CMD_LIST[0]))
        [responses, time_rx] = receive()
        print(f"Drive command response: {response_string('w0:1',responses)}")
        #break
    
    if sensor_front < threshold or sensor_frontl < diag_threshold or sensor_frontr < diag_threshold:
        #cl = False
        #rr = str(sensor_right*90/60)
        #rl = str(sensor_left*90/60*-1)
        if sensor_front < threshold and sensor_frontl < diag_threshold and sensor_frontr < diag_threshold and sensor_left < 6 and sensor_right < 6:
            while sensor_left < threshold or sensor_right < threshold:
                # Send a turn left command
                sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr = check_sensors()
                if sensor_right < 3:
                    time.sleep(LOOP_PAUSE_TIME)
                    transmit(packetize(CMD_LIST[2]))
                    [responses, time_rx] = receive()
                    print(f"Drive command response: {response_string('r0:10',responses)}")
                elif sensor_left < 3:
                    time.sleep(LOOP_PAUSE_TIME)
                    transmit(packetize(CMD_LIST[1]))
                    [responses, time_rx] = receive()
                    print(f"Drive command response: {response_string('r0:-10',responses)}")
                else:
                    time.sleep(LOOP_PAUSE_TIME)
                    transmit(packetize(CMD_LIST[3]))
                    [responses, time_rx] = receive()
                    print(f"Drive command response: {response_string('w0:-1',responses)}")
        if sensor_left > sensor_right:
            while sensor_front < threshold or sensor_frontr < diag_threshold or sensor_frontl < diag_threshold:
                
                sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr = check_sensors()
                
                if sensor_front < threshold - 2 or sensor_frontl < diag_threshold - 1.5 or sensor_frontr < diag_threshold - 1.5:
                    # Send reverse command
                    time.sleep(LOOP_PAUSE_TIME)
                    transmit(packetize(CMD_LIST[3]))
                    [responses, time_rx] = receive()
                    print(f"Drive command response: {response_string('w0:-1',responses)}")
                else:
                    # Send a turn left command
                    time.sleep(LOOP_PAUSE_TIME)
                    transmit(packetize(CMD_LIST[1]))
                    [responses, time_rx] = receive()
                    print(f"Drive command response: {response_string('r0:-10',responses)}")
        elif sensor_right > sensor_left:
            while sensor_front < threshold or sensor_frontr < diag_threshold or sensor_frontl < diag_threshold:
                # Send a turn right command
                sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr = check_sensors()
                if sensor_front < threshold - 2 or sensor_frontl < diag_threshold - 1.5 or sensor_frontr < diag_threshold - 1.5:
                    # Send reverse command
                    time.sleep(LOOP_PAUSE_TIME)
                    transmit(packetize(CMD_LIST[3]))
                    [responses, time_rx] = receive()
                    print(f"Drive command response: {response_string('w0:-1',responses)}")
                else:
                    # Send a turn left command
                    time.sleep(LOOP_PAUSE_TIME)
                    transmit(packetize(CMD_LIST[2]))
                    [responses, time_rx] = receive()
                    print(f"Drive command response: {response_string('r0:10',responses)}")
                
    else:
        # Send a drive forward command
        #cl = True
        transmit(packetize(CMD_LIST[0]))
        [responses, time_rx] = receive()
        print(f"Drive command response: {response_string('w0:1',responses)}")

# python3 scripts/test_client_sample.py