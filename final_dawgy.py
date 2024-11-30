import heapq
import subprocess
import pygame
import numpy as np
from PFLocalization import (
    initialize_particles, move_particles, update_particle_weights, 
    resample_particles, estimate_robot_position, calculate_ess, reinitialize_particles
)
from maze import Maze
from scan_processing import scan_rplidar, send_command
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


rpi_ip = '172.20.10.3'      # Raspberry Pi's IP address when connected to Baqir's iPhone hotspot
#rpi_ip = '100.67.145.12'    # Raspberry Pi's IP address when connected to UofT wifi (IP MAY CHANGE, USE 'ip a' ON RPI TERMINAL TO FIND NEW UOFT IP)
port = 8888
    
# Initialize the font for rendering text (optional)
font = pygame.font.SysFont(None, 24)


def section_scans(robot_readings):
    sensor_back = None
    sensor_backl = None
    sensor_left = None
    sensor_frontl = None
    sensor_front = None
    sensor_frontr = None
    sensor_right = None
    sensor_backr = None

    k = 0

    # Define sector boundaries in terms of scan indices

    SECTOR_DEFINITIONS = {
        'sensor_front': list(range(0, 3)) + list(range(57, 60)),    # 0°-18° and 342°-360° (indices 0-3 and 56-59)
        'sensor_frontr': list(range(3, 12)),                        # 18°-72°   (indices 3-12)
        'sensor_right': list(range(12, 18)),                        # 72°-108°  (indices 12-18)
        'sensor_backr': list(range(18, 27)),                        # 108°-162° (indices 18-27)
        'sensor_back': list(range(27, 33)),                         # 162°-198° (indices 27-33)
        'sensor_backl': list(range(33, 42)),                        # 198°-252° (indices 33-42)
        'sensor_left': list(range(42, 48)),                         # 252°-288° (indices 42-48)
        'sensor_frontl': list(range(48, 57))                        # 288°-360° (indices 48-59)
    }
        
    for scan in robot_readings:
        if scan is not None:
            # Determine which section the current scan belongs to
            for sector, indices in SECTOR_DEFINITIONS.items():
                if k in indices:
                    # Assign the minimum distance for the section
                    if sector == 'sensor_back':
                        if sensor_back is None or scan < sensor_back:
                            sensor_back = scan
                            # ab = k*6
                    elif sector == 'sensor_backl':
                        if sensor_backl is None or scan < sensor_backl:
                            sensor_backl = scan
                            # abl = k*6
                    elif sector == 'sensor_left':
                        if sensor_left is None or scan < sensor_left:
                            sensor_left = scan
                            # al = k*6
                    elif sector == 'sensor_frontl':
                        if sensor_frontl is None or scan < sensor_frontl:
                            sensor_frontl = scan
                            # afl = k*6
                    elif sector == 'sensor_front':
                        if sensor_front is None or scan < sensor_front:
                            sensor_front = scan
                            # af = k*6
                    elif sector == 'sensor_frontr':
                        if sensor_frontr is None or scan < sensor_frontr:
                            sensor_frontr = scan
                            # afr = k*6
                    elif sector == 'sensor_backr':
                        if sensor_backr is None or scan < sensor_backr:
                            sensor_backr = scan
                            # abr = k*6
                    elif sector == 'sensor_right':
                        if sensor_right is None or scan < sensor_right:
                            sensor_right = scan
                            # ar = k*6
                    break
        k += 1
    
    if sensor_back is None:
        sensor_back = 93.75
    if sensor_backl is None:
        sensor_backl = 93.75
    if sensor_left is None:
        sensor_left = 93.75
    if sensor_frontl is None:
        sensor_frontl = 93.75
    if sensor_front is None:
        sensor_front = 93.75
    if sensor_frontr is None:
        sensor_frontr = 93.75
    if sensor_right is None:
        sensor_right = 93.75
    if sensor_backr is None:
        sensor_backr = 93.75
    
    return sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl


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

def localization(NUM_PARTICLES=1000):
    
    ############## Main section for the open loop control algorithm ##############
    LOOP_PAUSE_TIME = 0.4 # seconds
    # Main loop
    RUNNING = True
    
    rpi_ip = '172.20.10.3'      # Raspberry Pi's IP address when connected to Baqir's iPhone hotspot
    #rpi_ip = '100.67.145.12'    # Raspberry Pi's IP address when connected to UofT wifi (IP MAY CHANGE, USE 'ip a' ON RPI TERMINAL TO FIND NEW UOFT IP)
    port = 8888
    
    particles = initialize_particles(NUM_PARTICLES)   # Initialize particles

    #CMD_LIST = ['w0:1.2', 'r0:-10', 'r0:10','w0:-0.5', 'r0:-18', 'r0:18']
    threshold = 7.5*25.4
    diag_threshold = 5.3*25.4
    NUM_STEPS = 786
    RESAMPLE_INTERVAL = 4
    iteration = 0
    convergence_condition = False
    sigma = 20
    j = 0
    
    robot_readings = scan_rplidar()
    sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl = section_scans(robot_readings)  # Check robot sensors
    print("1 : ", sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl)
    
    try:
        for i in range(NUM_STEPS):
            print("Loop starts")
            # Handle Pygame events
            handle_pygame_events()
            if not RUNNING:
                break

            # Pause to control command rate
            time.sleep(LOOP_PAUSE_TIME)
            
            if sensor_front > threshold+1.8 and sensor_frontr > diag_threshold+0.4 and sensor_frontl > diag_threshold+0.4:
                print("big move forward")
                # Send a drive forward command
                send_command(rpi_ip, port, "obs_bigmoveForward")
                # Move particles
                move_particles(particles, move_distance=2.5, rotation_change=0)
            
            # Handle movement commands based on sensor readings
            elif sensor_front > threshold and sensor_frontr > diag_threshold and sensor_frontl > diag_threshold:
                print("move forward")
                # Send a drive forward command
                send_command(rpi_ip, port, "obs_moveForward")
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=0)

            elif sensor_left > sensor_right and sensor_left > sensor_front:
                print("rotate_left small and move forward")
                # Send a drive forward command with left correction
                send_command(rpi_ip, port, "obs_smallrotateLeft")
                time.sleep(LOOP_PAUSE_TIME)
                send_command(rpi_ip, port, "obs_moveForward")
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=-10)

            elif sensor_left > sensor_right and sensor_left < sensor_front:
                print("rotate_left small and move forward")
                # Send a drive forward command with left correction
                send_command(rpi_ip, port, "obs_smallrotateLeft")
                time.sleep(LOOP_PAUSE_TIME)
                send_command(rpi_ip, port, "obs_moveForward")
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=-10)

            elif sensor_right > sensor_left and sensor_right > sensor_front:
                print("rotate_right small and move forward")
                # Send a drive forward command with right correction
                send_command(rpi_ip, port, "obs_smallrotateRight")
                time.sleep(LOOP_PAUSE_TIME)
                send_command(rpi_ip, port, "obs_moveForward")
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=10)

            elif sensor_right > sensor_left and sensor_right < sensor_front:
                print("rotate_right small and move forward")
                # Send a drive forward command with right correction
                send_command(rpi_ip, port, "obs_smallrotateRight")
                time.sleep(LOOP_PAUSE_TIME)
                send_command(rpi_ip, port, "obs_moveForward")
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=10)

            else:
                print("move forward 2")
                # Send a drive forward command
                send_command(rpi_ip, port, "obs_moveForward")
                # Move particles
                move_particles(particles, move_distance=1.2, rotation_change=0)

            time.sleep(LOOP_PAUSE_TIME)
            
            if sensor_front < threshold or sensor_frontl < diag_threshold or sensor_frontr < diag_threshold:
                if sensor_frontl > sensor_frontr:
                    opposite = False
                    while sensor_front < threshold or sensor_frontl < diag_threshold or sensor_frontr < diag_threshold:
                        robot_readings = scan_rplidar()
                        sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl = section_scans(robot_readings)  # Check robot sensors
                        print("2 : ", sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl)
                        if sensor_front < threshold or sensor_frontl < diag_threshold or sensor_frontr < diag_threshold:
                            if sensor_right > 3*12*25.4:
                                opposite = True
                            if opposite is True:
                                # Send a turn left command
                                send_command(rpi_ip, port, "obs_rotateRight")
                                time.sleep(LOOP_PAUSE_TIME)
                                # Move particles
                                move_particles(particles, move_distance=0, rotation_change=18)
                            else:
                                # Send a turn left command
                                send_command(rpi_ip, port, "obs_rotateLeft")
                                time.sleep(LOOP_PAUSE_TIME)
                                # Move particles
                                move_particles(particles, move_distance=0, rotation_change=-18)

                elif sensor_frontr > sensor_frontl:
                    opposite = False
                    while sensor_front < threshold or sensor_frontl < diag_threshold or sensor_frontr < diag_threshold:
                        robot_readings = scan_rplidar()
                        sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl = section_scans(robot_readings)  # Check robot sensors
                        print("3 : ", sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl)
                        if sensor_front < threshold or sensor_frontl < diag_threshold or sensor_frontr < diag_threshold:
                            if sensor_left > 3*12*25.4:
                                opposite = False
                            if opposite is True:
                                # Send a turn left command
                                send_command(rpi_ip, port, "obs_rotateLeft")
                                time.sleep(LOOP_PAUSE_TIME)
                                # Move particles
                                move_particles(particles, move_distance=0, rotation_change=-18)
                            else:
                                # Send a turn left command
                                send_command(rpi_ip, port, "obs_rotateRight")
                                time.sleep(LOOP_PAUSE_TIME)
                                # Move particles
                                move_particles(particles, move_distance=0, rotation_change=18)

            # Check robot sensors for localization
            robot_readings = scan_rplidar()
            sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl = section_scans(robot_readings)  # Check robot sensors
            print("4 : ", sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl)
            
            pfdistances = []
            pfangles = []
            l = 0
            for scan in robot_readings:     # Grab robot distances and angles to compare with partilces in localization
                if scan is not None:
                    pfdistances.append(scan/25.4)
                    pfangles.append(l*6)
                l += 1

            pfdistances = pfdistances[0::3]     # Don't need all 60 readings for localization, takes way too long to process
            pfangles = pfangles[0::3]

            # print("DISTANCES",pfdistances)
            # print("ANGLES",pfangles)
            
            if iteration == 0:
                update_particle_weights(particles, pfdistances, pfangles, sigma)  # Calculate particle weights
                particles = resample_particles(particles)   # Regenerate particles with weight
            if iteration % RESAMPLE_INTERVAL == 0 and iteration > 0:
                j += 1
                update_particle_weights(particles, pfdistances, pfangles, sigma)  # Calculate particle weights
                particles = resample_particles(particles)   # Regenerate particles with weight
                # if j == 4:
                #     convergence_condition = True
                if j == 3:
                    convergence_condition = True
                elif j == 2:
                    sigma = 5
                    particles = sorted(particles, key=lambda p: p.weight, reverse=True)[:250]
                    RESAMPLE_INTERVAL = 2
                elif j == 1:
                    sigma = 8
                    particles = sorted(particles, key=lambda p: p.weight, reverse=True)[:500]
                    
                    
                    print("Loop Iteration: ", i)
            iteration += 1
            # Update the display
            canvas.fill(CONFIG.background_color)    # Clear screen
            draw_maze()                             # Draw maze
            draw_particles_on_canvas(particles)     # Draw particles

            # Estimate robot position using top particles
            estimated_position = estimate_robot_position(particles)
            if estimated_position:
                ex, ey, etheta = estimated_position
                # Convert to screen coordinates
                ex_screen = int(ex * CONFIG.ppi + CONFIG.border_pixels)
                ey_screen = int(ey * CONFIG.ppi + CONFIG.border_pixels)
                
                # Desired radius in your units (e.g., inches)
                radius_inches = 3.69
                # Convert radius to pixels
                radius_pixels = int(radius_inches * CONFIG.ppi)
                
                # Draw the circle outline with specified width
                pygame.draw.circle(canvas, (0, 0, 255), (ex_screen, ey_screen), radius_pixels, width=1)
                
                # Calculate the endpoint of the line to represent orientation
                # Convert orientation angle from degrees to radians
                theta_radians = math.radians(etheta)
                
                # Calculate the endpoint coordinates
                x2 = ex_screen + radius_pixels * math.cos(theta_radians)
                y2 = ey_screen + radius_pixels * math.sin(theta_radians)  # Subtract because Pygame's Y-axis increases downward
                
                # Draw the line from the center to the edge of the circle
                pygame.draw.line(canvas, (0, 0, 255), (ex_screen, ey_screen), (x2, y2), width=2)  # Width can be adjusted as needed
                
                # Update the display
                pygame.display.flip()
            else:
                print("Cannot estimate position.")
                           
            
            if convergence_condition:
                print(f"We localized: Normal Coords = [{ex},{ey},{etheta}]")
                eposition = (ex, ey)
                eorientation = etheta

                # Save the top 50 particles
                top_50_particles = sorted(particles, key=lambda p: p.weight, reverse=True)[:50]
                print("Top 50 particles saved for reinitialization.")
                manual_control = False
                if manual_control is False:
                    time.sleep(LOOP_PAUSE_TIME)
                    return eposition, eorientation, top_50_particles
                
                # Clear the canvas to prepare for localization mode
                canvas.fill(CONFIG.background_color)

                # Enter a loop to listen for keyboard inputs for manual control
                
                while manual_control:
                    # Handle Pygame events
                    event = pygame.event.wait()
                    
                    if event.type == pygame.QUIT:
                            # Quit manual control mode and close Pygame window
                            manual_control = False
                            RUNNING = False
                            pygame.quit()
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_w:
                            us_sensor = send_command(rpi_ip, port, "obs_moveForward")
                            time.sleep(LOOP_PAUSE_TIME)
                            move_particles(particles, move_distance=1.2, rotation_change=0)
                        elif event.key == pygame.K_a:
                            us_sensor = send_command(rpi_ip, port, "obs_rotateLeft")
                            time.sleep(LOOP_PAUSE_TIME)
                            move_particles(particles, move_distance=0, rotation_change=-18)
                        elif event.key == pygame.K_d:
                            us_sensor = send_command(rpi_ip, port, "obs_rotateRight")
                            time.sleep(LOOP_PAUSE_TIME)
                            move_particles(particles, move_distance=0, rotation_change=18)
                        elif event.key == pygame.K_s:
                            us_sensor = send_command(rpi_ip, port, "obs_moveBackward")
                            time.sleep(LOOP_PAUSE_TIME)
                            move_particles(particles, move_distance=-0.5, rotation_change=0)
                        elif event.key == pygame.K_t:
                            us_sensor = send_command(rpi_ip, port, "obs_moveForward")
                            #time.sleep(LOOP_PAUSE_TIME)
                            us_sensor = send_command(rpi_ip, port, "obs_moveForward")
                            #time.sleep(LOOP_PAUSE_TIME)
                            us_sensor = send_command(rpi_ip, port, "obs_moveForward")
                            #time.sleep(LOOP_PAUSE_TIME)
                            move_particles(particles, move_distance=3.6, rotation_change=0)
                        elif event.key == pygame.K_f:
                            us_sensor = send_command(rpi_ip, port, "obs_rotateLeft")
                            #time.sleep(LOOP_PAUSE_TIME)
                            us_sensor = send_command(rpi_ip, port, "obs_rotateLeft")
                            #time.sleep(LOOP_PAUSE_TIME)
                            us_sensor = send_command(rpi_ip, port, "obs_rotateLeft")
                            #time.sleep(LOOP_PAUSE_TIME)
                            move_particles(particles, move_distance=0, rotation_change=-54)
                        elif event.key == pygame.K_h:
                            us_sensor = send_command(rpi_ip, port, "obs_rotateRight")
                            #time.sleep(LOOP_PAUSE_TIME)
                            us_sensor = send_command(rpi_ip, port, "obs_rotateRight")
                            #time.sleep(LOOP_PAUSE_TIME)
                            us_sensor = send_command(rpi_ip, port, "obs_rotateRight")
                            #time.sleep(LOOP_PAUSE_TIME)
                            move_particles(particles, move_distance=0, rotation_change=54)
                        
                        robot_readings = scan_rplidar()
                        sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl = section_scans(robot_readings)  # Check robot sensors
                    
                        pfdistances = []
                        pfangles = []
                        l = 0
                        for scan in robot_readings:     # Grab robot distances and angles to compare with partilces in localization
                            if scan is not None:
                                pfdistances.append(scan/25.4)
                                pfangles.append(l*6)
                            l += 1

                        pfdistances = pfdistances[0::3]     # Don't need all 60 readings for localization, takes way too long to process
                        pfangles = pfangles[0::3]

                        # print("DISTANCES",pfdistances)
                        # print("ANGLES",pfangles)
                        
                        iteration += 1
                    
                        update_particle_weights(particles, pfdistances, pfangles, sigma)  # Calculate particle weights
                        particles = resample_particles(particles)   # Regenerate particles with weight
                        
                        # Redraw the canvas to update the robot's estimated position and particle positions
                        canvas.fill(CONFIG.background_color)  # Clear screen
                        draw_maze()                           # Draw maze
                        draw_particles_on_canvas(particles)   # Draw particles
                        
                        estimated_position = estimate_robot_position(particles)
                        if estimated_position:
                            ex, ey, etheta = estimated_position
                            # Convert to screen coordinates
                            ex_screen = int(ex * CONFIG.ppi + CONFIG.border_pixels)
                            ey_screen = int(ey * CONFIG.ppi + CONFIG.border_pixels)
                            
                            # Desired radius in your units (e.g., inches)
                            radius_inches = 3.69
                            # Convert radius to pixels
                            radius_pixels = int(radius_inches * CONFIG.ppi)
                            
                            # Draw the circle outline with specified width
                            pygame.draw.circle(canvas, (0, 0, 255), (ex_screen, ey_screen), radius_pixels, width=1)
                            
                            # Calculate the endpoint of the line to represent orientation
                            # Convert orientation angle from degrees to radians
                            theta_radians = math.radians(etheta)
                            
                            # Calculate the endpoint coordinates
                            x2 = ex_screen + radius_pixels * math.cos(theta_radians)
                            y2 = ey_screen + radius_pixels * math.sin(theta_radians)  # Subtract because Pygame's Y-axis increases downward
                            
                            # Draw the line from the center to the edge of the circle
                            pygame.draw.line(canvas, (0, 0, 255), (ex_screen, ey_screen), (x2, y2), width=2)  # Width can be adjusted as needed
                            
                            # Update the display
                            pygame.display.flip()
                        else:
                            print("Cannot estimate position.")
                        
                        try:
                            us_sensor = float(us_sensor)
                        except ValueError:
                            # Handle the error
                            # For example, skip this iteration, assign a default value, or log the error
                            print("Invalid ultrasonic sensor reading received.")
                            continue  # Skip to the next iteration or handle as needed
                        
                        us_sensor = float(us_sensor)
                        print(us_sensor)
                        if sensor_back is not None:
                            if us_sensor < 8.0 and sensor_back > (8.0+3.5+2):
                                print("block might be in front")
                                if us_sensor < 4.0:
                                    print("BLOCK FOUND!!!!!!")
                                    print("Traveling to drop off zone")
                            
                    # Update Pygame display
                    pygame.display.flip()
                
                #return (eposition, eorientation)
            
    except KeyboardInterrupt:
        print("Interrupted by user.")

    finally:
        #pygame.quit()
        print("Localization complete.")

############## Navigation Code #############
### includes helper functions where the parent function is navigate_to
### will be used twice, one to get to loading zone, and once again to reach one of the drop off zones
### needs to be immersed with simmer code

### Function very straightforward, uses Manhattan distance
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

### A* Search Algorithm
def a_star_search(maze, start, goal):
    print(f"A* search being implemented, finding path from {start} to {goal}")
    rows, cols = len(maze), len(maze[0])  # Grid dimensions
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Possible moves
    open_set = []
    heapq.heappush(open_set, (0, start))
    path_history = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    iteration_count = 0

    while open_set:
        iteration_count += 1
        current_cost, current = heapq.heappop(open_set)

        if current == goal:
            path = reconstruct_path(path_history, current)
            print(f"Path found: {path}")
            return path

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and maze[neighbor[0]][neighbor[1]] != 0:
                if not is_valid_position_for_rover(maze, neighbor):
                    continue

                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    path_history[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    print("No path found.")
    return "inf"

def reconstruct_path(path_history, current):
    path = [current]
    while current in path_history:
        current = path_history[current]
        path.append(current)
    path.reverse()
    return path

# Helper function to determine if the position is valid for the rover given its size and required clearance
def is_valid_position_for_rover(maze, position):
    rows, cols = len(maze), len(maze[0])
    cx, cy = position

    # Iterate over a square area around the position to ensure no walls are within 6 units
    for dx in range(-6, 6):  # Include 6
        for dy in range(-6, 6):
            nx, ny = cx + dx, cy + dy
            # Check if the position is out of bounds
            if not (0 <= nx < rows and 0 <= ny < cols):
                continue  # Out of bounds cells are ignored
            # Check if the position is too close to a wall
            if maze[nx][ny] == 0:
                return False  # Position is too close to a wall or obstacle

    return True

def find_closest_valid_point(maze, start):
    from collections import deque
    rows, cols = len(maze), len(maze[0])
    visited = set()
    queue = deque()
    queue.append((start, [start]))  # Include path to current point
    visited.add(start)
    while queue:
        current, path = queue.popleft()
        if is_valid_position_for_rover(maze, current):
            return current, path  # Return valid point and path to it
        x, y = current
        # Explore neighbors
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (x + dx, y + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))
                    visited.add(neighbor)
    # If no valid point is found
    return None, []

def find_closest_valid_point_along_path(maze, path_points, est_position):
    closest_valid_point = None
    min_distance = float('inf')
    for point in path_points:
        if is_valid_position_for_rover(maze, point):
            distance = math.hypot(point[0] - est_position[0], point[1] - est_position[1])
            if distance < min_distance:
                min_distance = distance
                closest_valid_point = point
    return closest_valid_point

## Segment_path ##
def segment_path(path_points, max_segment_length=5):
    """
    Converts a list of waypoints into segments of straight lines.
    Each segment contains waypoints that are in the same direction
    and has a maximum length of max_segment_length points.
    """
    if not path_points:
        return []

    segments = []
    start_point = path_points[0]
    prev_point = path_points[0]
    current_direction = None
    segment_length = 1  # Initialize segment length

    for point in path_points[1:]:
        # Calculate direction from prev_point to current point
        dx = point[0] - prev_point[0]
        dy = point[1] - prev_point[1]
        direction = (dx, dy)

        # Check if direction has changed
        if current_direction is None:
            current_direction = direction
            segment_length = 1
        elif direction != current_direction or segment_length >= max_segment_length:
            # Direction changed or max segment length reached, finalize current segment
            segments.append({
                'start': start_point,
                'end': prev_point,
                'direction': current_direction
            })
            # Start a new segment
            start_point = prev_point
            current_direction = direction
            segment_length = 1
        else:
            segment_length += 1

        prev_point = point

    # Add the last segment
    segments.append({
        'start': start_point,
        'end': prev_point,
        'direction': current_direction
    })

    return segments


def update_particle_orientation(position, rotation_change, move_distance, top_50_particles):
    # Step 1: Move Particles with the correct rotation change and move distance applied
    move_particles(top_50_particles, move_distance=move_distance, rotation_change=rotation_change)
    robot_readings = scan_rplidar()
    
    pfdistances = []
    pfangles = []
    l = 0
    for scan in robot_readings:     # Grab robot distances and angles to compare with partilces in localization
        if scan is not None:
            pfdistances.append(scan/25.4)
            pfangles.append(l*6)
        l += 1

    pfdistances = pfdistances[0::3]     # Don't need all 60 readings for localization, takes way too long to process
    pfangles = pfangles[0::3]
    
    # print("DISTANCES",pfdistances)
    # print("ANGLES",pfangles)
    
    # Step 2: Update the particles
    update_particle_weights(top_50_particles, pfdistances, pfangles, sigma=8)
    top_50_particles = resample_particles(top_50_particles)
    time.sleep(0.2)

    # Step 3: obtain position and orientation of rover
    temp1, temp2, orientation = estimate_robot_position(top_50_particles)
    
    estimated_position = (temp1, temp2)
    
    return estimated_position, orientation, top_50_particles
        
# The average will be shown as a blue dot
def obtain_average(position, rotation_change, move_distance, top_50_particles):
    # Step 1: apply update particle orientation to obtain necessary values
    estimated_position, orientation, top_50_particles = update_particle_orientation(position, rotation_change, move_distance, top_50_particles)
    avg_x, avg_y, etheta = estimated_position[0], estimated_position[1], orientation
    
    # Step 2: Convert to screen coordinates and display estimated position
    estimated_position = estimate_robot_position(top_50_particles)
    if estimated_position:
        ex, ey, etheta = estimated_position
        # Convert to screen coordinates
        ex_screen = int(ex * CONFIG.ppi + CONFIG.border_pixels)
        ey_screen = int(ey * CONFIG.ppi + CONFIG.border_pixels)
        
        # Desired radius in your units (e.g., inches)
        radius_inches = 3.69
        # Convert radius to pixels
        radius_pixels = int(radius_inches * CONFIG.ppi)
        
        # Draw the circle outline with specified width
        pygame.draw.circle(canvas, (0, 0, 255), (ex_screen, ey_screen), radius_pixels, width=1)
        
        # Calculate the endpoint of the line to represent orientation
        # Convert orientation angle from degrees to radians
        theta_radians = math.radians(etheta)
        
        # Calculate the endpoint coordinates
        x2 = ex_screen + radius_pixels * math.cos(theta_radians)
        y2 = ey_screen + radius_pixels * math.sin(theta_radians)  # Subtract because Pygame's Y-axis increases downward
        
        # Draw the line from the center to the edge of the circle
        pygame.draw.line(canvas, (0, 0, 255), (ex_screen, ey_screen), (x2, y2), width=2)  # Width can be adjusted as needed
        
        # Update the display
        pygame.display.flip()
    else:
        print("Cannot estimate position.")

    pygame.display.flip()  # Update the full display
    
    # Step 3: prepare variables to return and return them
    eposition = (ex, ey)
    eorientation = etheta
    #eorientation = orientation_fixer(eorientation)
    print(f"Fixed orientation {eorientation}")
    
    return eposition, eorientation, top_50_particles

# The display must be updated after every action on the simmer console
def update_display(top_50_particles, estimated_position, path_segments,):
    canvas.fill(CONFIG.background_color)  # Clear screen
    draw_maze()                           # Draw maze
    draw_particles_on_canvas(top_50_particles)  # Draw particles

    # Draw the path segments as yellow circles
    if path_segments:
        for segment in path_segments:
            # Draw the start and end points of the segment as yellow circles
            sx, sy = segment['start']  # These are correct
            ex, ey = segment['end']
            # Convert to screen coordinates
            sx_screen = int(sx * CONFIG.ppi + CONFIG.border_pixels)
            sy_screen = int(sy * CONFIG.ppi + CONFIG.border_pixels)
            ex_screen = int(ex * CONFIG.ppi + CONFIG.border_pixels)
            ey_screen = int(ey * CONFIG.ppi + CONFIG.border_pixels)
            pygame.draw.circle(canvas, (255, 255, 0), (sx_screen, sy_screen), 5)  # Yellow circle at start
            pygame.draw.circle(canvas, (255, 255, 0), (ex_screen, ey_screen), 5)  # Yellow circle at end
    estimated_position = estimate_robot_position(top_50_particles)
    if estimated_position:
        ex, ey, etheta = estimated_position
        # Convert to screen coordinates
        ex_screen = int(ex * CONFIG.ppi + CONFIG.border_pixels)
        ey_screen = int(ey * CONFIG.ppi + CONFIG.border_pixels)
        
        # Desired radius in your units (e.g., inches)
        radius_inches = 3.69
        # Convert radius to pixels
        radius_pixels = int(radius_inches * CONFIG.ppi)
        
        # Draw the circle outline with specified width
        pygame.draw.circle(canvas, (0, 0, 255), (ex_screen, ey_screen), radius_pixels, width=1)
        
        # Calculate the endpoint of the line to represent orientation
        # Convert orientation angle from degrees to radians
        theta_radians = math.radians(etheta)
        
        # Calculate the endpoint coordinates
        x2 = ex_screen + radius_pixels * math.cos(theta_radians)
        y2 = ey_screen + radius_pixels * math.sin(theta_radians)  # Subtract because Pygame's Y-axis increases downward
        
        # Draw the line from the center to the edge of the circle
        pygame.draw.line(canvas, (0, 0, 255), (ex_screen, ey_screen), (x2, y2), width=2)  # Width can be adjusted as needed
        
        # Update the display
        pygame.display.flip()
    else:
        print("Cannot estimate position.")

    pygame.display.flip()  # Update the full display
    pygame.display.flip()  # Update the full display

### Parent Function ###

def navigate_to(maze, goal_positions_list, est_position, est_orientation, top_50_particles, max_retries=3, current_retry=0):
    # goal_positions_list is a list of possible goal positions
    best_path = None
    best_path_length = float('inf')
    best_goal_position = None
    best_valid_start_point = None

    for goal_pos in goal_positions_list:
        goal_grid = (int(goal_pos[0]), int(goal_pos[1]))

        # Find the closest valid point to the estimated position
        valid_start_point, path_to_valid_start = find_closest_valid_point(maze, (int(est_position[0]), int(est_position[1])))
        if valid_start_point is None:
            print(f"No valid starting point found near estimated position {est_position}.")
            continue  # Skip to next goal_pos

        print(f"Adjusted start position from {est_position} to closest valid point: {valid_start_point}")

        # Use A* to find the best path from valid starting point to goal
        path_from_valid_start = a_star_search(maze, valid_start_point, goal_grid)
        if not path_from_valid_start or path_from_valid_start == "inf":
            print(f"No path found from {valid_start_point} to the goal {goal_grid}.")
            continue  # Skip to next goal_pos

        # Combine the paths
        full_path = path_to_valid_start + path_from_valid_start[1:]  # Exclude duplicate valid_start_point

        path_length = len(full_path)
        print(f"Combined path length from {est_position} to {goal_grid} is {path_length}.")

        if path_length < best_path_length:
            best_path_length = path_length
            best_path = full_path
            best_goal_position = goal_grid
            best_valid_start_point = valid_start_point

    if best_path is None:
        print("No valid paths found to any of the goal positions.")
        return est_position  # Return current position or handle error appropriately

    # Proceed with the best path
    path_segments = segment_path(best_path)
    print(f"Selected path from {est_position} to goal {best_goal_position} with path length {best_path_length}")

    # Now proceed with navigation using est_position, est_orientation, path_segments, top_50_particles

    # While the rover is not within the target tolerance
    while abs(est_position[0] - best_goal_position[0]) >= 0.3 or abs(est_position[1] - best_goal_position[1]) >= 0.3:
        for segment in path_segments:
            # Adjust the Rover before moving Forward
            print(f"Adjust rover is inputted est position: {est_position}, segment direction {segment['direction']}, and orientation: {est_orientation}")
            est_position, est_orientation, rotation_change = adjust_rover_orientation(est_position, segment['direction'], est_orientation)
            print(f"Rover Position Updated by adjust rover position: position: {est_position}, orientation: {est_orientation}") 
            
            # Update particles and display
            est_position, est_orientation, top_50_particles = obtain_average(est_position, rotation_change, 0, top_50_particles)
            update_display(top_50_particles, est_position, path_segments)
            # Calculate the distance to the next segment point
            segment_length = math.hypot(segment['end'][0] - segment['start'][0], segment['end'][1] - segment['start'][1])
            print(f"Segment length: {segment_length}")
            
            # Move towards the next segment
            est_position, est_orientation, success = move_to_waypoint_with_localization(est_position, segment_length, segment, est_orientation, top_50_particles)
            
            # If move to waypoint is unsuccessful
            if not success:
                # Check rover position in maze boundaries
                if not (0 <= est_position[0] < len(maze) and 0 <= est_position[1] < len(maze[0])):
                    print("Rover position is out of maze bounds.")
                    return est_position
                print("Out of tolerance boundary, recalculating path.")
                # Recalculate path with updated est_position
                return navigate_to(maze, goal_positions_list, est_position, est_orientation, top_50_particles)
            else:
                print("Iteration within success limit, next iteration initiated")
    return est_position, f"Found final position {est_position}"


def orientation_fixer(current_orientation):
    """
    Corrects the current orientation to the nearest multiple of 90 degrees within the range of 0 to 360 degrees.
    Implements rotations in steps of maximum 45 degrees until the required angle change is completed.
    """
    print(f"Starting orientation fixer. Current orientation: {current_orientation}")
    
    # Normalize the angle to the range [0, 360)
    normalized_angle = current_orientation % 360

    # Find the nearest multiple of 90 degrees
    nearest_multiple = round(normalized_angle / 90) * 90

    # Ensure the corrected angle is within [0, 360)
    corrected_angle = nearest_multiple % 360

    # Calculate the minimal angle difference
    angle_difference = (corrected_angle - normalized_angle + 360) % 360
    if angle_difference > 180:
        angle_difference -= 360  # Choose the shortest rotation

    rotation_change = 0
    max_turn_angle = 45  # Maximum turn angle in degrees
    tolerance = 0        # Since we want to align exactly to nearest 90 degrees

    # Loop until the angle difference is within the tolerance
    while abs(angle_difference) > tolerance:
        # Determine the rotation step
        step_angle = min(max_turn_angle, abs(angle_difference))
        if angle_difference > 0:
            # Rotate right
            command = f'rotate_right: {int(step_angle)}\n'
            send_command(rpi_ip, port, command)
            current_orientation = (current_orientation + step_angle) % 360
            rotation_change += step_angle
            print(f"Command: rotate right by {step_angle} degrees")
        else:
            # Rotate left
            command = f'rotate_left: {int(step_angle)}\n'
            send_command(rpi_ip, port, command)
            current_orientation = (current_orientation - step_angle) % 360
            rotation_change -= step_angle
            print(f"Command: rotate left by {step_angle} degrees")
        
        # Recalculate the angle difference after the rotation
        normalized_angle = current_orientation % 360
        angle_difference = (corrected_angle - normalized_angle + 360) % 360
        if angle_difference > 180:
            angle_difference -= 360  # Choose the shortest rotation

    print(f"Angle fixer completed. Corrected orientation: {current_orientation}")
    return current_orientation


### Moves forward by segment: input is position | Segment length | segment (access dictionary) | orientation | top_50_particles
def move_to_waypoint_with_localization(position, segment_length, segment, orientation, top_50_particles):
    # fix orientation to nearest 90 degrees before movement
    
    # move forward by segment_length and transmit
    # CMD_MOVE_FORWARD = f'w0:{segment_length}'
    # Move forward towards the segment
    
    command = f'move_forward: {abs(int(segment_length))}\n'
    send_command(rpi_ip, port, command)
    # transmit(packetize(CMD_MOVE_FORWARD))
    # [responses, time_rx] = receive()
    time.sleep(0.2)
    
    print(f"Moving to waypoint: position {position}, orientation {orientation}, moving forward {segment_length} units")

    # Obtain the new particle estimate
    est_position, est_orientation, top_50_particles = obtain_average(position, 0, segment_length, top_50_particles)
    update_display(top_50_particles, est_position, path_segments=None)

    tolerance = 1  # Tolerance is inches
    
    # Error Check: calculate error in both directions
    error_x = abs(est_position[0] - segment['end'][0])
    error_y = abs(est_position[1] - segment['end'][1])

    if error_x <= tolerance and error_y <= tolerance:
        current_position = segment['end']  # Update position
        success = True
        print(f"Rover reached the point {segment['end']} within tolerance, success: {success}")
    else:
        current_position = est_position
        orientation = est_orientation
        success = False
        print(f"Rover did not reach the waypoint {segment['end']} within tolerance: est_position {est_position}, orientation: {orientation}, success {success}")
    return current_position, orientation, success

### Rough turn to face direction of travel
def adjust_rover_orientation(current_position, direction_vector, current_orientation):
    print(f"Adjusting rover orientation from {current_orientation}, direction vector is {direction_vector}")
    
    # Break direction vector into x and y components
    dx, dy = direction_vector 
    
    # Calculate the target angle from the current position (in degrees)
    target_angle = math.degrees(math.atan2(dy, dx)) % 360

    # Calculate the minimal angle to turn
    angle_difference = (target_angle - current_orientation + 360) % 360
    if angle_difference > 180:
        angle_difference -= 360  # Choose the shortest rotation

    rotation_change = 0
    max_turn_angle = 45  # Maximum turn angle in degrees
    tolerance = 15       # Tolerance in degrees

    # Loop until the angle difference is within the tolerance
    while abs(angle_difference) > tolerance:
        # Determine the rotation step
        step_angle = min(max_turn_angle, abs(angle_difference))
        if angle_difference > 0:
            # Rotate right
            command = f'rotate_right: {int(step_angle)}\n'
            send_command(rpi_ip, port, command)
            current_orientation = (current_orientation + step_angle) % 360
            rotation_change += step_angle
            print(f"Command: rotate right by {step_angle} degrees")
        else:
            # Rotate left
            command = f'rotate_left: {int(step_angle)}\n'
            send_command(rpi_ip, port, command)
            current_orientation = (current_orientation - step_angle) % 360
            rotation_change -= step_angle
            print(f"Command: rotate left by {step_angle} degrees")
        
        # Recalculate the angle difference after the rotation
        angle_difference = (target_angle - current_orientation + 360) % 360
        if angle_difference > 180:
            angle_difference -= 360  # Choose the shortest rotation

    print(f"No further rotation needed. Current orientation: {current_orientation}")
    return current_position, current_orientation, rotation_change
 

######### MAIN and Parent LOOP ########

############## Constant Definitions Begin ##############
### Network Setup ###
# HOST = '127.0.0.1'      # The server's hostname or IP address
# PORT_TX = 61200         # The port used by the *CLIENT* to receive
# PORT_RX = 61201         # The port used by the *CLIENT* to send data

# ### Serial Setup ###
# BAUDRATE = 9600         # Baudrate in bps
# PORT_SERIAL = 'COM3'    # COM port identification
# TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

# ### Packet Framing values ###
# FRAMESTART = '['
# FRAMEEND = ']'
# CMD_DELIMITER = ','

# ### Set whether to use TCP (SimMeR) or serial (Arduino) ###
# SIMULATE = True
# TRANSMIT_PAUSE = 0.1 if SIMULATE else 0

# ############### Initialize ##############
# ### Source to display
# if SIMULATE:
#     SOURCE = 'SimMeR'
# else:
#     SOURCE = 'serial device ' + PORT_SERIAL

# if not SIMULATE:
#     try:
#         SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
#     except serial.SerialException:
#         print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')

############## Main section for the open loop control algorithm ##############
RUNNING = True
LOOP_PAUSE_TIME = 0.1 # seconds

#eposition, eorientation, top_50_particles = localization()

# Actual maze configuration as provided earlier
walls = [
    [3, 3, 1, 1, 0, 2, 0, 2],
    [3, 3, 0, 1, 1, 1, 1, 1],
    [1, 0, 2, 0, 0, 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 0, 2]
]

# Transpose the walls to match the orientation
walls = [list(row) for row in zip(*walls)]  

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
    (66.0, 8.0),  # Loading bay 1 is at the 5th square, row 1
    (90.0, 8.0),   # Loading bay 2 is at 7th square, row 1
    (30.0, 32.0), # Loading bay 3 is at 3rd square, row 3
    (90.0, 40.0)  # Loading bay 4 is at 7th square row 4
]
target_loading_zone = 2
#x_e = 6.0
#y_e = 6.0

# insert desired position for target zone
#position_1 = (x_e, y_e)
position_1 =  [(24.0,6.0), (6.0,24.0)]
# This will be changed
position_2 = loading_zones[target_loading_zone]

top_50_particles = reinitialize_particles(eposition, eorientation, num_particles=200, position_variance=1, angle_variance=5)
#load_zone = navigate_to(maze2, position_1, eposition, eorientation, top_50_particles)
final_position = navigate_to(maze2, position_2, (24.0,6.0), (1,0), top_50_particles)  # Navigate to the drop-off