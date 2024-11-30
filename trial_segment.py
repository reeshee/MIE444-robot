import heapq
import subprocess
import pygame
import numpy as np
from PFLocalization import (
    initialize_particles, move_particles, update_particle_weights, 
    resample_particles, estimate_robot_position, reinitialize_particles
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
    #print(responses)
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
PORT_SERIAL = 'COM7'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = True
TRANSMIT_PAUSE = 0.3 if SIMULATE else 0

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




def localization(NUM_PARTICLES=100):
    
    ############## Main section for the open loop control algorithm ##############
    LOOP_PAUSE_TIME = 0.05 # seconds
    # Main loop
    RUNNING = True
    
    particles = initialize_particles(NUM_PARTICLES)   # Initialize particles

    CMD_LIST = ['w0:1.2', 'r0:-10', 'r0:10','w0:-0.5', 'r0:-18', 'r0:18']
    threshold = 7.7
    diag_threshold = 7.7
    NUM_STEPS = 555
    sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr = check_sensors()    # Check robot sensors
    iteration = 0
    RESAMPLE_INTERVAL = 1
    convergence_condition = False
    sigma = 20
    j = 0
    
    try:
        for i in range(NUM_STEPS):
            # Handle Pygame events
            handle_pygame_events()
            if not RUNNING:
                break

            # Pause to control command rate
            time.sleep(LOOP_PAUSE_TIME)
            

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
            iteration += 1
            
            #if ess > HIGH_ESS * NUM_PARTICLES:
            #    update_particle_weights(particles, robot_readings, sigma=4)  # Calculate particle weights
            #    print("YESSSIIIIRRRRRR")
            #    particles = resample_particles(particles)   # Regenerate particles with weight
            #    convergence_condition += 1
            #    RESAMPLE_INTERVAL = 1
            if iteration == 1:
                update_particle_weights(particles, robot_readings, sigma)  # Calculate particle weights
                particles = resample_particles(particles)   # Regenerate particles with weight
            
            if iteration % RESAMPLE_INTERVAL == 0 and iteration > 0:
                j += 1
                update_particle_weights(particles, robot_readings, sigma)  # Calculate particle weights
                #ess = calculate_ess(particles)
                #print(f"ESS = {ess}  --------------- Convergence Condition = {convergence_condition}")
                #if ess < ESS_THRESHOLD:
                particles = resample_particles(particles)   # Regenerate particles with weight
                # if j == 4:
                #     convergence_condition = True
                # elif j == 3:
                #     RESAMPLE_INTERVAL = 2
                # elif j == 2:
                #     sigma = 3
                # elif j == 1:
                #     sigma = 5
                #     particles = sorted(particles, key=lambda p: p.weight, reverse=True)[:500]
                #     RESAMPLE_INTERVAL = 5
                    
                if j == 3:
                    convergence_condition = True
                elif j == 2:
                    sigma = 5
                elif j == 1:
                    sigma = 2
                    
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

                canvas.fill(CONFIG.background_color)
                draw_maze()
                
                # Save the top 50 particles
                #top_50_particles = sorted(particles, key=lambda p: p.weight, reverse=True)[:50]
                #print(top_50_particles)
                #print("Top 50 particles saved for reinitialization.")
                return eposition, eorientation
            

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
    print(f"a star search being implemented, finding path from {start} to {goal}")
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
    return "inf"


# Helper function to determine if the position is valid for the rover given its size and required clearance
def is_valid_position_for_rover(maze, position):
    rows, cols = len(maze), len(maze[0])
    cx, cy = position

    # Iterate over a square area around the position to ensure no walls are within 4 inches
    for dx in range(-2, 2):
        for dy in range(-2, 2):
            
            if cx < 2 or cx >= (rows - 2) or cy < 2 or cy >= (cols - 2):
                #print(f"is valid position failed for({cx},{cy})")
                return False  # Too close to the maze border

            nx, ny = cx + dx, cy + dy
            # Check if the position is out of bounds
            if not (0 <= nx < rows and 0 <= ny < cols):
                #print(f"is valid position Failed for({nx},{ny}) position out of bounds")
                continue  # Out of bounds cells are ignored
            # Check if the position is too close to a wall
            if maze[nx][ny] == 0:
                #print(f"is valid position Failed for({nx},{ny}) too close to wall")
                return False  # Position is too close to a wall or obstacle
    
    return True

## Segment_path ##

def segment_path(path_points):
    """
    Converts a list of waypoints into segments of straight lines.
    Each segment contains waypoints that are in the same direction.
    """
    print(path_points)
    if not path_points or path_points == 'inf':
        return []

    segments = []
    start_point = path_points[0]
    prev_point = path_points[0]
    current_direction = None

    for point in path_points[1:]:
        # Calculate direction from prev_point to current point
        dx = point[0] - prev_point[0]
        dy = point[1] - prev_point[1]
        direction = (dx, dy)

        if current_direction is None:
            current_direction = direction
        elif direction != current_direction:
            # Direction changed, finalize current segment
            segments.append({
                'start': start_point,
                'end': prev_point,
                'direction': current_direction
            })
            # Start a new segment
            start_point = prev_point
            current_direction = direction

        prev_point = point

    # Add the last segment
    segments.append({
        'start': start_point,
        'end': prev_point,
        'direction': current_direction
    })

    return segments

def update_particle_orientation(position, rotation_change, move_distance, top_50_particles):
    ##### particles travel the wrong direction in 2nd iteration
    position = (position [1], position[0])
    print(f"Updating particle orientation: position {position}, rotation change {rotation_change}, move distance {move_distance}")
    move_particles(top_50_particles, move_distance=move_distance, rotation_change=rotation_change)
    sensor_front, sensor_right, sensor_left, sensor_back, sensor_frontl, sensor_frontr, sensor_backl, sensor_backr = check_sensors()
    robot_readings = [sensor_front, sensor_frontr, sensor_right, sensor_backr,
                      sensor_back, sensor_backl, sensor_left, sensor_frontl]
    print(f"after move particles: position {position}, rotation change {rotation_change}, move distance {move_distance}")
    # Update particle weights and resample
    update_particle_weights(top_50_particles, robot_readings, sigma=8)
    resample_particles(top_50_particles)
    time.sleep(0.4)
    print(f"Particle orientation updated. Estimated position: {estimate_robot_position(top_50_particles)}")
    temp1, temp2, orientation = estimate_robot_position(top_50_particles)
    orientation += 90
    estimated_position = (temp1, temp2)
    return estimated_position, orientation, top_50_particles
    
def obtain_average (position, rotation_change, move_distance, top_50_particles):
    # Simulate the rover moving forward in the particle filter
    estimated_position, orientation, top_50_particles = update_particle_orientation(position, rotation_change, move_distance, top_50_particles)
    avg_x, avg_y, etheta = estimated_position[0], estimated_position [1], orientation
    estimated_position = avg_x, avg_y
    # Convert to screen coordinates and display estimated position
    if estimated_position:
        ex, ey = estimated_position
        ex_screen = int(ex * CONFIG.ppi + CONFIG.border_pixels)
        ey_screen = int(ey * CONFIG.ppi + CONFIG.border_pixels)
        pygame.draw.circle(canvas, (0, 0, 255), (ex_screen, ey_screen), 5)  # Blue circle for estimated position

    pygame.display.flip()  # Update the full display

    # Fix the orientation of localization to orientation to the navigation maze
    eposition = (ey, ex)
    eorientation = etheta
    print(f"obtaining average from particles:position, orientation {eposition, eorientation}")
    return eposition, eorientation, top_50_particles
    
def update_display(top_50_particles, path_segments):
    canvas.fill(CONFIG.background_color)  # Clear screen
    draw_maze()                           # Draw maze
    draw_particles_on_canvas(top_50_particles)  # Draw particles

    # Draw the path segments as yellow circles
    if path_segments:
        for segment in path_segments:
            # Draw the start and end points of the segment as yellow circles
            sy, sx = segment['start']
            ey, ex = segment['end']
            # Convert to screen coordinates
            sx_screen = int(sx * CONFIG.ppi + CONFIG.border_pixels)
            sy_screen = int(sy * CONFIG.ppi + CONFIG.border_pixels)
            ex_screen = int(ex * CONFIG.ppi + CONFIG.border_pixels)
            ey_screen = int(ey * CONFIG.ppi + CONFIG.border_pixels)
            pygame.draw.circle(canvas, (255, 255, 0), (sx_screen, sy_screen), 5)  # Yellow circle at start
            pygame.draw.circle(canvas, (255, 255, 0), (ex_screen, ey_screen), 5)  # Yellow circle at end

    pygame.display.flip()  # Update the full display

    
def navigate_to(maze, position, est_position, est_orientation, top_50_particles, max_retries=3, current_retry=0):
    # test statement, print current position and orientation
    print(f"running navigate_to function, initial values are {est_position}, {est_orientation}")

    # Use A* to find the best path to the target loading zone
    path_points = a_star_search(maze, est_position, position) # --> feed into this the maze defined in main, current position and orientation
    # if invalid loading zone/drop-off zone, no path returned
    if position is None:
        print("No valid loading zone to navigate to.")
        return est_position  # No valid path found, return current position
    
    ### Define navigation path and return all into terminal
    print(f"Navigating from {est_position} to Target Zone {position} with path points: {path_points}")
    
    path_segments = segment_path(path_points)
    print(f"Path segments: {path_segments}")
    
        # While the rover is not within the target tolerance
    while abs(est_position[0] - position[0]) >= 0.3 or abs(est_position[1] - position[1]) >= 0.3:
        # Iterate through each path segment
        est_orientation = orientation_fixer(est_orientation)
        for segment in path_segments:
            segment_length = math.hypot(segment['end'][0] - segment['start'][0], segment['end'][1] - segment['start'][1])
            print(f"the path segments is{path_segments}, the direction is{segment["direction"]},")
            
            # # Move to the waypoint
 
            # Move forward the segment length
            est_position, est_orientation, success = move_to_waypoint_with_localization(est_position, segment_length, segment, est_orientation, top_50_particles)
            # est_position, est_orientation, top_50_particles = obtain_average(est_position, 0, segment_length, top_50_particles)
            update_display(top_50_particles, path_segments)

            path_points = a_star_search(maze, est_position, position)
            path_segments = segment_path(path_points)
            # If movement was unsuccessful
            if not success:
            # Check rover position in maze boundaries
                if not (0 <= est_position[0] < 48 and 0 <= est_position[1] < 96):
                    print("Start position is out of maze bounds.")
                    print(f"What the HEEEEL {est_position}")
                    # if nto return the incorrect position and finish execution
                    return est_position
                # Recalculate path from current position
                est_orientation = orientation_fixer(est_orientation)
                print("Out of tolerance boundary, recalculating path.")
                est_position, est_orientation, rotation_change = adjust_rover_orientation(est_position, segment['direction'], est_orientation)
                print(f"Rover Position Updated by adjust rover position: position: {est_position}, orientation: {est_orientation}")       
                est_position, est_orientation, top_50_particles = obtain_average(est_position, 0, 0, top_50_particles)
                update_display(top_50_particles, path_segments)
                print(f"Rover Position Updated by localization. position: {est_position}, orientation: {position}")
                est_orientation = orientation_fixer(est_orientation)

                break  # Exit the for loop to start over with new path
                        # update particle position
            est_position, est_orientation, rotation_change = adjust_rover_orientation(est_position, segment['direction'], est_orientation)
            print(f"Rover Position Updated by adjust rover position: position: {est_position}, orientation: {est_orientation}")          
            est_position, est_orientation, top_50_particles = obtain_average(est_position, rotation_change, segment_length, top_50_particles)
            update_particle_orientation(position, rotation_change, 0, top_50_particles)
            update_display(top_50_particles, path_segments)
            print(f"Rover Position Updated by localization. position: {est_position}, orientation: {position}")

    return est_position, (f"found final position {est_position}")  
        
def orientation_fixer(angle):
    """
    Corrects the input angle to the nearest multiple of 90 degrees within the range of 0 to 360 degrees.
    Also returns the minimal difference between the original angle and the corrected angle.

    Parameters:
        angle (float): The input angle in degrees.

    Returns:
        tuple:
            corrected_angle (float): The angle adjusted to the nearest multiple of 90 degrees within [0, 360).
            angle_difference (float): The minimal difference between the input angle and the corrected angle, in degrees.
    """
    # Normalize the angle to the range [0, 360)
    normalized_angle = angle % 360

    # Find the nearest multiple of 90 degrees
    nearest_multiple = round(normalized_angle / 90) * 90

    # Ensure the corrected angle is within [0, 360)
    corrected_angle = nearest_multiple % 360

    # Calculate the minimal angle difference
    angle_difference = normalized_angle - corrected_angle
    # Adjust the difference to be within [-180, 180)
    if angle_difference >= 180:
        angle_difference -= 360
    elif angle_difference < -180:
        angle_difference += 360

    time.sleep(LOOP_PAUSE_TIME=0.5)

    # Since we want the difference from the original input angle
    # Use the difference between the original angle and where it was snapped to
    if angle_difference != 0:
        transmit(packetize(f'r0:{angle_difference}'))
        [responses, time_rx] = receive()
        print(f"angle fixer employed to correct rotation from {angle} to {corrected_angle}")
    else:
        print("angle_difference is equal to 0 so we skip")
    return corrected_angle




def move_to_waypoint_with_localization(position, segment_length, segment, orientation, top_50_particles):
    orientation = orientation_fixer(orientation)
    CMD_MOVE_FORWARD = f'w0:{segment_length}'  # Move forward command
    print(f"Moving to waypoint: position {position}, orientation {orientation}, moving forward {segment_length} units")
    # Move forward towards the segment
    transmit(packetize(CMD_MOVE_FORWARD))
    [responses, time_rx] = receive()
    print(f"Command: move_forward {segment_length} units")
    time.sleep(1)
    # Update the display
    update_display(top_50_particles, path_segments=None)


    tolerance = 1  # Tolerance in units (e.g., inches)c
    # Since rotation is handled in adjust_rover_orientation, we remove rotation logic here
    est_position, est_orientation, top_50_particles = obtain_average(position, orientation, segment_length, top_50_particles)
    print(f"THIS IS THE VALUE{est_position}")
    # Check if within tolerance
    print(f"This is calculated final position {segment}, versus actual {est_position}")
    error_x = abs(est_position[0] - segment['end'][0])
    error_y = abs(est_position[1] - segment['end'][1])

    if error_x <= tolerance and error_y <= tolerance:
        print(f"Rover reached the point {segment} within tolerance.")
        current_position = segment['end']  # Update position
        success = True
    else:
        print(f"Rover did not reach the waypoint {segment} within tolerance.")
        current_position = est_position
        print(current_position)
        orientation = est_orientation
        success = False
    print(f"Move_to_waypoint_function COMPLETED: position - {current_position}, orientation - {orientation}, is success? - {success}")
    return current_position, orientation, success

def adjust_rover_orientation(current_position, direction_vector, current_orientation):
    print(f"Adjusting rover orientation from {current_orientation}, direction vector is {direction_vector}")
    dx, dy = direction_vector
    # Calculate the angle to the direction vector from the current position (in degrees)
    target_angle = math.degrees(math.atan2(dy, dx)) % 360

    # Calculate the minimal angle to turn
    angle_difference = (target_angle - current_orientation + 360) % 360
    if angle_difference > 180:
        angle_difference -= 360  # Choose the shortest rotation

    # Determine rotation direction and execute turn
    rotation_change = 0
    if abs(angle_difference) > 35:  # Tolerance of 35 degrees
        if angle_difference > 0:
            # Rotate right
            transmit(packetize(f'r0:{abs(angle_difference)}'))
            rotation_change = abs(angle_difference)
            current_orientation = (current_orientation + rotation_change) % 360
            print(f"Command: rotate_right by {rotation_change} degrees")
            [responses, time_rx] = receive()
        else:
            # Rotate left
            transmit(packetize(f'r0:-{abs(angle_difference)}'))
            rotation_change = -abs(angle_difference)
            current_orientation = (current_orientation + rotation_change) % 360
            print(f"Command: rotate_left by {abs(rotation_change)} degrees")
            [responses, time_rx] = receive()
    else:
        print(f"No rotation needed. Current orientation: {current_orientation}")
    print(f"Adjust rover orientation completed. New orientation: {current_orientation}, position: {current_position}")
    return current_position, current_orientation, rotation_change
    


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

eposition, eorientation = localization()
particles = reinitialize_particles(eposition, eorientation)
draw_particles_on_canvas(particles)

ex, ey = eposition
etheta = eorientation
# Convert to screen coordinates
ex_screen = int(ex * CONFIG.ppi + CONFIG.border_pixels)
ey_screen = int(ey * CONFIG.ppi + CONFIG.border_pixels)
pygame.draw.circle(canvas, (0, 0, 255), (ex_screen, ey_screen), 5)  # Blue circle for estimated position
pygame.draw.circle(canvas, (0, 0, 255), (ex_screen, ey_screen), 5)  # Blue circle
pygame.display.flip()  # Update the full display

# est_position = eposition[1], eposition[0]
# print(est_position)
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
load_zone = navigate_to(maze2, position_1, eposition, eorientation, particles)
final_position = navigate_to(maze2, position_2, eposition, eorientation, particles) ### Navigate to the drop-off
#ser.close()
print(f"Final Position: {final_position}")