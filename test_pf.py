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

import pygame
import numpy as np
from PFLocalization import (
    initialize_particles, move_particles, update_particle_weights, 
    resample_particles, estimate_robot_position, calculate_ess
)
from maze import Maze
#from scan_processing import target_scans
import config as CONFIG
import socket
import time
from datetime import datetime
import serial
import math

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
TRANSMIT_PAUSE = 0.1 if SIMULATE else 0

############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = 'SimMeR'
else:
    SOURCE = 'serial device ' + PORT_SERIAL
if not SIMULATE:
    try:
        SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
    except serial.SerialException:
        print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')


# Initialize Pygame for Particle Visualization
pygame.init()
canvas_width = int(CONFIG.maze_dim_x * CONFIG.ppi + 2 * CONFIG.border_pixels)
canvas_height = int(CONFIG.maze_dim_y * CONFIG.ppi + 2 * CONFIG.border_pixels)
canvas = pygame.display.set_mode((canvas_width, canvas_height))
pygame.display.set_caption("Particle Filter Visualization")

# Initialize the font for rendering text (optional)
font = pygame.font.SysFont(None, 24)

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
            chk = '✓'
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
    #print(f"Ultrasonic 0 reading: {response_string('u0',responses)}")
    sensor_front = float(responses[0][1])

    # Check an ultrasonic sensor 'u1'
    transmit(packetize('u1'))
    [responses, time_rx] = receive()
    #print(f"Ultrasonic 1 reading: {response_string('u1',responses)}")
    sensor_right = float(responses[0][1])

    # Check an ultrasonic sensor 'u2'
    transmit(packetize('u2'))
    [responses, time_rx] = receive()
    #print(f"Ultrasonic 2 reading: {response_string('u2',responses)}")
    sensor_left = float(responses[0][1])

    # Check an ultrasonic sensor 'u3'
    transmit(packetize('u3'))
    [responses, time_rx] = receive()
    #print(f"Ultrasonic 3 reading: {response_string('u3',responses)}")
    sensor_back = float(responses[0][1])

    # Check an ultrasonic sensor 'u4'
    transmit(packetize('u4'))
    [responses, time_rx] = receive()
    #print(f"Ultrasonic 4 reading: {response_string('u4',responses)}")
    sensor_frontl = float(responses[0][1])

    # Check an ultrasonic sensor 'u5'
    transmit(packetize('u5'))
    [responses, time_rx] = receive()
    #print(f"Ultrasonic 5 reading: {response_string('u5',responses)}")
    sensor_frontr = float(responses[0][1])
    
    # Check an ultrasonic sensor 'u6'
    transmit(packetize('u6'))
    [responses, time_rx] = receive()
    #print(f"Ultrasonic 6 reading: {response_string('u6',responses)}")
    sensor_backl = float(responses[0][1])
    
    # Check an ultrasonic sensor 'u7'
    transmit(packetize('u7'))
    [responses, time_rx] = receive()
    #print(f"Ultrasonic 7 reading: {response_string('u7',responses)}")
    sensor_backr = float(responses[0][1])

    return sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr


# Initialize particles
maze = Maze()
maze.import_walls()


# Function to draw the maze

def draw_maze():
    maze.draw_walls(canvas)  # Implement this method in your Maze class
# Function to draw particles

def draw_particles_on_canvas(particles):
    for particle in particles:
        px = int(particle.x * CONFIG.ppi + CONFIG.border_pixels)
        py = int(particle.y * CONFIG.ppi + CONFIG.border_pixels)
        pygame.draw.circle(canvas, (0, 255, 0), (px, py), 2)  # Small green circles
# Function to handle Pygame events

def handle_pygame_events():
    global RUNNING  # Ensure RUNNING is correctly modified
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            RUNNING = False


def localization(NUM_PARTICLES=2000):
    
    ############## Main section for the open loop control algorithm ##############
    LOOP_PAUSE_TIME = 0.1 # seconds
    # Main loop
    RUNNING = True
    
    particles = initialize_particles(NUM_PARTICLES)   # Initialize particles

    CMD_LIST = ['w0:1.2', 'r0:-10', 'r0:10','w0:-0.5', 'r0:-18', 'r0:18']
    threshold = 7.5
    diag_threshold = 7.5
    NUM_STEPS = 555
    sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr = check_sensors()    # Check robot sensors
    iteration = 0
    RESAMPLE_INTERVAL = 10
    convergence_condition = 0
    ess = 0
    sigma = 12
    j = 0
    
    try:
        for i in range(NUM_STEPS):
            # Handle Pygame events
            handle_pygame_events()
            if not RUNNING:
                break

            # Pause to control command rate
            time.sleep(LOOP_PAUSE_TIME)
            iteration += 1

            # Handle movement commands based on sensor readings
            if sensor_front > threshold and sensor_frontr > diag_threshold and sensor_frontl > diag_threshold:
                # Send a drive forward command
                transmit(packetize(CMD_LIST[0]))
                time.sleep(LOOP_PAUSE_TIME)
                [responses, time_rx] = receive()
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=0)

            elif sensor_left > sensor_right and sensor_left > sensor_front:
                # Send a drive forward command with left correction
                transmit(packetize(CMD_LIST[1]))
                time.sleep(LOOP_PAUSE_TIME)
                transmit(packetize(CMD_LIST[0]))
                [responses, time_rx] = receive()
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=-10)

            elif sensor_left > sensor_right and sensor_left < sensor_front:
                # Send a drive forward command with left correction
                transmit(packetize(CMD_LIST[1]))
                time.sleep(LOOP_PAUSE_TIME)
                transmit(packetize(CMD_LIST[0]))
                [responses, time_rx] = receive()
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=-10)

            elif sensor_right > sensor_left and sensor_right > sensor_front:
                # Send a drive forward command with right correction
                transmit(packetize(CMD_LIST[2]))
                time.sleep(LOOP_PAUSE_TIME)
                transmit(packetize(CMD_LIST[0]))
                [responses, time_rx] = receive()
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=10)

            elif sensor_right > sensor_left and sensor_right < sensor_front:
                # Send a drive forward command with right correction
                transmit(packetize(CMD_LIST[2]))
                time.sleep(LOOP_PAUSE_TIME)
                transmit(packetize(CMD_LIST[0]))
                [responses, time_rx] = receive()
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=10)

            else:
                # Send a drive forward command
                transmit(packetize(CMD_LIST[0]))
                [responses, time_rx] = receive()
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=0)


            if sensor_front < threshold or sensor_frontl < diag_threshold or sensor_frontr < diag_threshold:
                if sensor_left > sensor_right:
                    while sensor_front < threshold or sensor_frontl < diag_threshold or sensor_frontr < diag_threshold:
                        sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr = check_sensors()    # Check robot sensors
                        if sensor_front < threshold or sensor_frontl < diag_threshold or sensor_frontr < diag_threshold:
                            # Send a turn left command
                            time.sleep(LOOP_PAUSE_TIME)
                            transmit(packetize(CMD_LIST[4]))
                            [responses, time_rx] = receive()
                            # Move particles
                            move_particles(particles, move_distance=0, rotation_change=-18)

                elif sensor_right > sensor_left:
                    while sensor_front < threshold or sensor_frontl < diag_threshold or sensor_frontr < diag_threshold:
                        sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr = check_sensors()    # Check robot sensors
                        if sensor_front < threshold or sensor_frontl < diag_threshold or sensor_frontr < diag_threshold:
                            # Send a turn right command
                            time.sleep(LOOP_PAUSE_TIME)
                            transmit(packetize(CMD_LIST[5]))
                            [responses, time_rx] = receive()
                            # Move particles
                            move_particles(particles, move_distance=0, rotation_change=18)

            # Check robot sensors
            sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr = check_sensors()    # Check robot sensors
            robot_readings = [sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl]   # Robot sensors list

            ESS_THRESHOLD = 0.5 * NUM_PARTICLES # Set threshold to 50% of total particles
            HIGH_ESS = 0.65
            
            #if ess > HIGH_ESS * NUM_PARTICLES:
            #    update_particle_weights(particles, robot_readings, sigma=4)  # Calculate particle weights
            #    print("YESSSIIIIRRRRRR")
            #    particles = resample_particles(particles)   # Regenerate particles with weight
            #    convergence_condition += 1
            #    RESAMPLE_INTERVAL = 1
            if iteration % RESAMPLE_INTERVAL == 0 and iteration > 0:
                j += 1
                update_particle_weights(particles, robot_readings, sigma)  # Calculate particle weights
                ess = calculate_ess(particles)
                print(f"ESS = {ess}  --------------- Convergence Condition = {convergence_condition}")
                if ess < ESS_THRESHOLD:
                    particles = resample_particles(particles)   # Regenerate particles with weight
                    if j == 5:
                        convergence_condition = True
                    elif j == 4:
                        sigma = 4
                    elif j == 3:
                        sigma = 8
                        RESAMPLE_INTERVAL = 5
                    
                    print(i)


            # Estimate robot position using top particles
            estimated_position = estimate_robot_position(particles)
            if estimated_position:
                ex, ey, etheta = estimated_position
                # Convert to screen coordinates
                ex_screen = int(ex * CONFIG.ppi + CONFIG.border_pixels)
                ey_screen = int(ey * CONFIG.ppi + CONFIG.border_pixels)
                pygame.draw.circle(canvas, (0, 0, 255), (ex_screen, ey_screen), 5)  # Blue circle for estimated position
            else:
                print("Cannot estimate position.")


            # Update the display
            canvas.fill(CONFIG.background_color) # Clear screen
            draw_maze()                          # Draw maze
            draw_particles_on_canvas(particles)           # Draw particles
            if estimated_position:
                pygame.draw.circle(canvas, (0, 0, 255), (ex_screen, ey_screen), 5)  # Blue circle
            pygame.display.flip()  # Update the full display

            if convergence_condition:
                print(f"Normal Coords = [{ex},{ey},{etheta}]")
                eposition = (ex, ey)
                eorientation = etheta

                # Save the top 50 particles
                top_50_particles = sorted(particles, key=lambda p: p.weight, reverse=True)[:50]
                print("Top 50 particles saved for reinitialization.")
                break
            
            

    except KeyboardInterrupt:
        print("Interrupted by user.")

    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    finally:
        #pygame.quit()
        print("Localization complete.")
        
localization()
