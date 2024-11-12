import heapq
import subprocess
import pygame
import numpy as np
from PFLocalization import (
    initialize_particles, move_particles, update_particle_weights, 
    resample_particles, estimate_robot_position, calculate_ess
)
from maze import Maze
#from scan_processing import scan_rplidar
import config as CONFIG
import socket
import time
from datetime import datetime
import serial
import time
import math


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
    threshold = 7.7
    diag_threshold = 7.7
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
                #if ess < ESS_THRESHOLD:
                particles = resample_particles(particles)   # Regenerate particles with weight
                if j == 6:
                    convergence_condition = True
                elif j == 4:
                    sigma = 3
                    particles = sorted(particles, key=lambda p: p.weight, reverse=True)[:500] 
                elif j == 3:
                    sigma = 5
                    particles = sorted(particles, key=lambda p: p.weight, reverse=True)[:1000]
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


############## Navigation Code #############
### includes helper functions where the parent function is navigate to
### will be used twice, one to get to loading zone, and once again to reach one of the drop off zones
### needs to be immersed with simmer code

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* Search Algorithm
def a_star_search(maze, start, goal):
    print("we here")
    rows, cols = len(maze), len(maze[0]) # iterate through all possible direction nodes
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Define the directions of the nodes to consider
    open_set = [] # set empty array open_set to store location of these nodes
    heapq.heappush(open_set, (0, start)) # defines the first node as the start position form the localization
    path_history = {} # stores the previous steps
    # Next g_score and f_score are defined which are crucial for the A* search algorithm to work successfully
    # algorithm uses g_score to see if a node is lower cost compared to the saved, if it is, g_score is reupdated
    # f_score is used to determine the priority of the nodes in the open set, the lower
    # the f_score, the higher the priority of the node in the open set
    g_score = {start: 0} # defines the initial point 0 as the start point 
    f_score = {start: heuristic(start, goal)} # represents estimated total cost if a specific node is chosen
    #max_iterations = 1000
    iteration_count = 0

    while open_set: # and iteration_count < max_iterations:
        iteration_count += 1
        current_cost, current = heapq.heappop(open_set)

        #print(f"Iteration: {iteration_count}, Current node: {current}, Cost: {current_cost}")

        if current == goal:
            path = []
            while current in path_history:
                path.append(current)
                current = path_history[current]
            path.reverse()
            #print(f"Path found: {path}")
            return (path)

        for dx, dy in directions:
            neighbor = (int(current[0]) + dx, int(current[1]) + dy)
            #print(f"Evaluating neighbor: {neighbor}")

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and maze[neighbor[0]][neighbor[1]] != 0:
                if not is_valid_position_for_rover(maze, neighbor): 
                    #print(f"Neighbor {neighbor} is not valid due to clearance.")
                    continue

                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    path_history[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    #print(f"Neighbor {neighbor} added to open set with f_score: {f_score[neighbor]}, heuristic: {heuristic(neighbor, goal)}")
    print (iteration_count)
    print("No path found.")
    return float('inf')


# Helper function to determine if the position is valid for the rover given its size and required clearance
def is_valid_position_for_rover(maze, position):
    rows, cols = len(maze), len(maze[0])
    cx, cy = position

    # Iterate over a square area around the position to ensure no walls are within 5 inches
    for dx in range(-4, 4):
        for dy in range(-4, 4):
            nx, ny = cx + dx, cy + dy
            # Check if the position is out of bounds
            if not (0 <= nx < rows and 0 <= ny < cols):
                continue  # Out of bounds cells are ignored
            # Check if the position is too close to a wall
            if maze[nx][ny] == 0:
                return False  # Position is too close to a wall or obstacle
    
    return True

def navigate_to(maze, position, est_position, est_orientation, top_50_particles, max_retries=3, current_retry=0, orientation=0):
    
    #est_position, est_orientation, top_50_particles = localization()### localization function
    
    sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr = check_sensors()    # Check robot sensors
    #scans_rpl = scan_rplidar(5)    # Check robot sensors
    #sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr = scans_rpl
    print(est_position, est_orientation)
    # if current_retry >= max_retries:
    #     print("Maximum retries reached. Navigation aborted.")
    #     return current_position  # Abort navigation after too many retries
    
    # Use A* to find the best path to the target loading zone
    path_points = a_star_search(maze, est_position, position)
    #adjust_rover_orientation(est_orientation, top_50_particles) # sets orientation to 0 with respect to East
    
    
    if position is None:
        print("No valid loading zone to navigate to.")
        return est_position  # No valid path found, return current position

    print(f"Navigating from {est_position} to Target Zone {position} with path points: {path_points}")
    
    print(position)

    while abs(est_position[0] - position[0]) >= 0.3 or abs(est_position[1] - position[1]) >= 0.3:
    # Iterate through each path point using the helper function
        for point in path_points:
            est_position, est_orientation, boolean = move_to_waypoint_with_localization(est_position, point, maze, est_orientation, top_50_particles)

        # Check if the current position is False, indicating an out-of-tolerance situation
            if boolean is False:
                # Out of tolerance boundary, recalculating path
                print("Out of tolerance boundary, recalculating path.")
                adjust_rover_orientation(orientation,top_50_particles)
                orientation = 0
                path_points = a_star_search(maze, est_position, position)
                print(path_points)
                break

    return est_position, (f"found final position{est_position}")


def move_to_waypoint_with_localization(current_position, waypoint, maze, orientation, top_50_particles):
    CMD_LIST = ['r0:50', 'r0:-50', 'w0:1']
    
    # Update the display
    canvas.fill(CONFIG.background_color) # Clear screen
    draw_maze()                          # Draw maze
    draw_particles_on_canvas(top_50_particles)           # Draw particles
    
    tolerance = 1  # Tolerance in units (e.g., inches)

    # Calculate the direction vector to the waypoint
    dx = waypoint[0] - current_position[0]
    dy = waypoint[1] - current_position[1]
    rotation_changer = 0
    # Calculate the angle to the waypoint from the current position (in degrees)
    target_angle = math.degrees(math.atan2(dy, dx)) % 360

    # Calculate the minimal angle to turn
    angle_difference = (target_angle - orientation + 360) % 360
    if angle_difference > 180:
        angle_difference -= 360  # Choose the shortest rotation

    # Determine turning direction and execute turn
    if angle_difference > 45:
        transmit(packetize(CMD_LIST[0]))
        time.sleep(LOOP_PAUSE_TIME)
        [responses, time_rx] = receive()
        print(f"Command: rotate_right by {angle_difference} degrees")
        rotation_changer = 90
        # Update orientation
        orientation = (orientation + rotation_changer) % 360
    elif angle_difference < -45:
        transmit(packetize(CMD_LIST[1]))
        time.sleep(LOOP_PAUSE_TIME)
        [responses, time_rx] = receive()
        print(f"Command: rotate_left by {-angle_difference} degrees")
        rotation_changer = -90
        # Update orientation
        orientation = (orientation + rotation_changer) % 360
    else:
        print("No rotation needed.")

    # Wait for rotation to complete (adjust time as needed)
    time.sleep(0.4)

    # Move forward towards the waypoint
    transmit(packetize(CMD_LIST[2]))
    time.sleep(LOOP_PAUSE_TIME)
    [responses, time_rx] = receive()
    #ser.write(b'move_forward\n')
    print("Command: move_forward")

    # Wait for movement to complete (adjust time as needed)
    time.sleep(0.2)

    # Simulate the rover moving to the waypoint
    #LOCALIZATION LOCALIZATION LOCALIZTION
    move_particles(top_50_particles, move_distance=1, rotation_change = rotation_changer)
    
    #scans_rpl = scan_rplidar(5)    # Check robot sensors
    #sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr = scans_rpl
    sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr = check_sensors()    # Check robot sensors
    robot_readings = [sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl]   # Robot sensors list
    #print("First",top_50_particles)
    
    update_particle_weights(top_50_particles, robot_readings)
    #print("update",top_50_particles)
    resample_particles(top_50_particles)
    #print("resample",top_50_particles)
    time.sleep(0.4)
    estimated_position = estimate_robot_position(top_50_particles)
    
    # Convert to screen coordinates
    if estimated_position:
        ex, ey, etheta = estimated_position
        ex_screen = int(ex * CONFIG.ppi + CONFIG.border_pixels)
        ey_screen = int(ey * CONFIG.ppi + CONFIG.border_pixels)
        pygame.draw.circle(canvas, (0, 0, 255), (ex_screen, ey_screen), 5)  # Blue circle for estimated position
    
    if current_position:
        pygame.draw.circle(canvas, (0, 0, 255), (ex_screen, ey_screen), 5)  # Blue circle for estimated position
    pygame.display.flip()  # Update the full display
    
    e_position = (ex, ey)
    eorientation = etheta

    # Check if within tolerance
    error_x = abs(e_position[0] - waypoint[0])
    error_y = abs(eposition[1] - waypoint[1])

    if error_x <= tolerance and error_y <= tolerance:
        print(f"Rover reached the waypoint {waypoint} within tolerance.")
        current_position = waypoint  # Update position
    else:
        print(f"Rover did not reach the waypoint {waypoint} within tolerance.")
        current_position = e_position
        orientation = eorientation
        return current_position, orientation, False

    return current_position, orientation, True

def adjust_rover_orientation(lidar_angle, top_50_particles):     #REDOOOOO!!!!!!__________________________________________________

    # Round the LIDAR angle to the nearest integer
    lidar_angle_int = int(round(lidar_angle))
    print(lidar_angle_int)
    # Convert LIDAR angle (north=0°) to rover coordinate system (east=0°)
    # Mapping: code_angle = (450 - lidar_angle) % 360
    code_angle = (lidar_angle_int) % 360
    print(code_angle)
    # Desired orientation is 0 degrees (east)
    desired_orientation = 0

    # Calculate minimal angle difference to get to desired orientation
    angle_to_rotate = code_angle
    print(angle_to_rotate)


    # Determine rotation direction and prepare command
    if angle_to_rotate > desired_orientation or angle_to_rotate > desired_orientation:
        # Send reverse command
        time.sleep(LOOP_PAUSE_TIME)
        transmit(packetize(f'r0:{angle_to_rotate}'))
        [responses, time_rx] = receive()
        # Move particles
        move_particles(top_50_particles, move_distance=0, rotation_change=angle_to_rotate)
        print(f"rotated by{angle_to_rotate}")
        time.sleep(LOOP_PAUSE_TIME)
        #command = f'adjust{abs(int(angle_to_rotate))}\n'  r0:anglerotate
        #ser.write(command.encode())
        #print(f"Sending command: {command.strip()}")
    else:
        print("No rotation needed.")
        # Optionally send a command or do nothing

    # Wait for rotation to complete (adjust time as needed based on your rover's rotation speed)
    time.sleep(1.2)

    return angle_to_rotate  # Return the angle for confirmation or further processing



######### MAIN and Parent LOOP ########

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

############## Main section for the open loop control algorithm ##############
RUNNING = True
LOOP_PAUSE_TIME = 0.1 # seconds

eposition, eorientation, top_50_particles = localization(NUM_PARTICLES=2000)
#print(eposition, eorientation, top_50_particles)
#ser = serial.Serial('COM7', 9600, timeout=1)


# Actual maze configuration as provided earlier
walls = [
    [3, 3, 1, 1, 0, 2, 0, 2],
    [3, 3, 0, 1, 1, 1, 1, 1],
    [1, 0, 2, 0, 0, 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 0, 2]
]

# Adjusting wall matrix to represent cells of 1-inch each
# Each 12-inch cell becomes a 12x12 grid of 1-inch cells
expanded_maze = []
for row in walls:
    expanded_row = []
    for cell in row:
        expanded_row.extend([cell] * 12)
    for _ in range(12):
        expanded_maze.append(expanded_row)

# Convert expanded wall matrix to a traversable maze representation (1: traversable, 0: wall)
maze2 = [[1 if cell != 0 else 0 for cell in row] for row in expanded_maze]

loading_zones = [
    (8.0, 66.0),  # Loading bay 1 is at the 5th square, row 1
    (8.0,90.0),   # Loading bay 2 is at 7th square, row 1
    (32.0, 30.0), # Loading bay 3 is at 3rd square, row 3
    (40.0, 90,0)  # Loading bay 4 is at 7th square row 4
]
target_loading_zone = 2
x_e = 6.0
y_e = 6.0
position_1 = (x_e,y_e)
position_2 = loading_zones [target_loading_zone]

load_zone = navigate_to(maze2, position_1, eposition, eorientation, top_50_particles)
final_position = navigate_to(maze2, position_2, eposition, eorientation, top_50_particles) ### Navigate to the drop-off
#ser.close()
print(f"Final Position: {final_position}")