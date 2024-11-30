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
from run_dawgy import
import time
import time
import math

# Initialize Pygame for Particle Visualization
pygame.init()
canvas_width = int(CONFIG.maze_dim_x * CONFIG.ppi + 2 * CONFIG.border_pixels)
canvas_height = int(CONFIG.maze_dim_y * CONFIG.ppi + 2 * CONFIG.border_pixels)
canvas = pygame.display.set_mode((canvas_width, canvas_height))
pygame.display.set_caption("Particle Filter Visualization")

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



rpi_ip = '172.20.10.3'
port = '8888'

LOOP_PAUSE_TIME = 0.2
manual_control = True
iteration = 0
sigma = 12


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
            move_particles(particles, move_distance=3.6, rotation_change=0)
        elif event.key == pygame.K_f:
            us_sensor = send_command(rpi_ip, port, "obs_rotateLeft")
            move_particles(particles, move_distance=0, rotation_change=-54)
        elif event.key == pygame.K_h:
            us_sensor = send_command(rpi_ip, port, "obs_rotateRight")
            move_particles(particles, move_distance=0, rotation_change=54)
        elif event.key == pygame.K_l:
            us_sensor = send_command(rpi_ip, port, "arm_lower")
        elif event.key == pygame.K_u:
            us_sensor = send_command(rpi_ip, port, "arm_upper")
        
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
        
        # us_sensor = float(us_sensor)
        # print(us_sensor)
        # if sensor_back is not None:
        #     if us_sensor < 8.0 and sensor_back > (8.0+3.5+2):
        #         print("block might be in front")
        #         if us_sensor < 4.0:
        #             print("BLOCK FOUND!!!!!!")
        #             print("Traveling to drop off zone")
        
        
        us_sensor = send_command(rpi_ip, port, command="obs_moveForward")
        scan = float(us_sensor)
        print(scan)
        # if sensor_front is not None:
        #     if us_sensor < 8.0 and sensor_front > (8.0+3.5+2):
        #         print("block might be in front")
        #         if us_sensor < 5.0:
        #             print("BLOCK FOUND!!!!!!")
        #             print("Traveling to drop off zone")
                    
        if scan < 7.0 and sensor_front - scan > 2.0:
            print("Block Might Be Ahead")
            rescan = float(send_command(rpi_ip, port, command="scan"))
            lidar_rescan = scan_rplidar()[0]

            if rescan < 4.0 and lidar_rescan - rescan > 2.0:
                print("BLOCK!!!!!")
            
    # Update Pygame display
    pygame.display.flip()