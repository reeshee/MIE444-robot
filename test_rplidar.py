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
    resample_particles, estimate_robot_position
)
from maze import Maze
from scan_processing import scan_rplidar, send_command
import config as CONFIG
import time
from datetime import datetime
import math


# Initialize Pygame for Particle Visualization
pygame.init()
canvas_width = int(CONFIG.maze_dim_x * CONFIG.ppi + 2 * CONFIG.border_pixels)
canvas_height = int(CONFIG.maze_dim_y * CONFIG.ppi + 2 * CONFIG.border_pixels)
canvas = pygame.display.set_mode((canvas_width, canvas_height))
pygame.display.set_caption("Particle Filter Visualization")

# Initialize the font for rendering text (optional)
font = pygame.font.SysFont(None, 24)


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

# Define all 32 blocks with center-based coordinates
block11 = (3, 9, 3, 9)
block12 = (3, 9, 15, 21)
block13 = (3, 9, 27, 33)
block14 = (3, 9, 39, 45)

block21 = (15, 21, 3, 9)
block22 = (15, 21, 15, 21)
block23 = (15, 21, 27, 33)
block24 = (15, 21, 39, 45)

block31 = (27, 33, 3, 9)
block32 = (27, 33, 15, 21)
block33 = (27, 33, 27, 33)
block34 = (27, 33, 39, 45)

block41 = (39, 45, 3, 9)
block42 = (39, 45, 15, 21)
block43 = (39, 45, 27, 33)
block44 = (39, 45, 39, 45)

block51 = (51, 57, 3, 9)
block52 = (51, 57, 15, 21)
block53 = (51, 57, 27, 33)  # West
block54 = (51, 57, 39, 45)

block61 = (63, 69, 3, 9)
block62 = (63, 69, 15, 21)
block63 = (63, 69, 27, 33)
block64 = (63, 69, 39, 45)

block71 = (75, 81, 3, 9)
block72 = (75, 81, 15, 21)
block73 = (75, 81, 27, 33)
block74 = (75, 81, 39, 45)

block81 = (87, 93, 3, 9)
block82 = (87, 93, 15, 21)
block83 = (87, 93, 27, 33)
block84 = (87, 93, 39, 45)    # East towards the loading zone

# Define direction biases for each block
block_directions = {
    'block11': 0,
    'block12': 0,
    'block13': 270,
    'block14': 270,

    'block21': 0,
    'block22': 0,
    'block23': 0,
    'block24': 180,

    'block31': 180,
    'block32': 0,
    'block33': 90,
    'block34': 180,

    'block41': 180,
    'block42': 270,
    'block43': 0,
    'block44': 180,

    'block51': 0,
    'block52': 180,
    'block53': 0,  # West
    'block54': 180,

    'block61': 90,
    'block62': 180,
    'block63': 90,
    'block64': 180,

    'block71': 0,
    'block72': 180,
    'block73': 0,
    'block74': 0,

    'block81': 90,
    'block82': 180,
    'block83': 270,
    'block84': 270,    # East towards the loading zone
}

# Function to determine current block using if-elif statements
def get_current_block_if_elif(ex, ey):
    """
    Determines the current block based on the robot's estimated position using if-elif statements.
    
    Parameters:
        ex (float): Estimated X position in inches.
        ey (float): Estimated Y position in inches.
    
    Returns:
        str: The identifier of the current block (e.g., 'block11').
                Returns None if the position is not within any block.
    """
    # Check block11
    if 3 <= ex < 9 and 3 <= ey < 9:
        return 'block11'
    elif 3 <= ex < 9 and 15 <= ey < 21:
        return 'block12'
    elif 3 <= ex < 9 and 27 <= ey < 33:
        return 'block13'
    elif 3 <= ex < 9 and 39 <= ey < 45:
        return 'block14'
    
    # Check block21
    elif 15 <= ex < 21 and 3 <= ey < 9:
        return 'block21'
    elif 15 <= ex < 21 and 15 <= ey < 21:
        return 'block22'
    elif 15 <= ex < 21 and 27 <= ey < 33:
        return 'block23'
    elif 15 <= ex < 21 and 39 <= ey < 45:
        return 'block24'
    
    # Check block31
    elif 27 <= ex < 33 and 3 <= ey < 9:
        return 'block31'
    elif 27 <= ex < 33 and 15 <= ey < 21:
        return 'block32'
    elif 27 <= ex < 33 and 27 <= ey < 33:
        return 'block33'
    elif 27 <= ex < 33 and 39 <= ey < 45:
        return 'block34'
    
    # Check block41
    elif 39 <= ex < 45 and 3 <= ey < 9:
        return 'block41'
    elif 39 <= ex < 45 and 15 <= ey < 21:
        return 'block42'
    elif 39 <= ex < 45 and 27 <= ey < 33:
        return 'block43'
    elif 39 <= ex < 45 and 39 <= ey < 45:
        return 'block44'
    
    # Check block51
    elif 51 <= ex < 57 and 3 <= ey < 9:
        return 'block51'
    elif 51 <= ex < 57 and 15 <= ey < 21:
        return 'block52'
    elif 51 <= ex < 57 and 27 <= ey < 33:
        return 'block53'
    elif 51 <= ex < 57 and 39 <= ey < 45:
        return 'block54'
    
    # Check block61
    elif 63 <= ex < 69 and 3 <= ey < 9:
        return 'block61'
    elif 63 <= ex < 69 and 15 <= ey < 21:
        return 'block62'
    elif 63 <= ex < 69 and 27 <= ey < 33:
        return 'block63'
    elif 63 <= ex < 69 and 39 <= ey < 45:
        return 'block64'
    
    # Check block71
    elif 75 <= ex < 81 and 3 <= ey < 9:
        return 'block71'
    elif 75 <= ex < 81 and 15 <= ey < 21:
        return 'block72'
    elif 75 <= ex < 81 and 27 <= ey < 33:
        return 'block73'
    elif 75 <= ex < 81 and 39 <= ey < 45:
        return 'block74'
    
    # Check block81
    elif 87 <= ex < 93 and 3 <= ey < 9:
        return 'block81'
    elif 87 <= ex < 93 and 15 <= ey < 21:
        return 'block82'
    elif 87 <= ex < 93 and 27 <= ey < 33:
        return 'block83'
    elif 87 <= ex < 93 and 39 <= ey < 45:
        return 'block84'
    
    else:
        return None  # Position is not within any defined block

# Function to handle block-based reorientation
def handle_block_reorientation(ex, ey, etheta, particles):
    """
    Checks if the robot is within a block's center area and reorients if necessary.
    
    Parameters:
        ex (float): Estimated X position in inches.
        ey (float): Estimated Y position in inches.
        etheta (float): Current orientation angle in degrees.
    
    Returns:
        tuple: (new_ex, new_ey, new_etheta)
    """
    current_block = get_current_block_if_elif(ex, ey)
    
    rpi_ip = '172.20.10.3'
    port = 8888
    
    if current_block:
        desired_direction = block_directions.get(current_block, etheta)
        
        if desired_direction != etheta:
            rotation_difference = (desired_direction - etheta + 360) % 360
            if rotation_difference > 180:
                rotation_difference -= 360  # Choose the shortest rotation direction
            
            # Rotate the robot
            if rotation_difference != 0:
                
                #rotate_command = f"r0:{rotation_difference}"
                print(f"Reorienting in {current_block}: Rotating by {rotation_difference} degrees to {desired_direction} degrees.")
                # transmit(packetize(rotate_command))
                # [responses, time_rx] = receive()
                command = f'rotate_right: {abs(int(rotation_difference))}\n'
                send_command(rpi_ip,port,command)
                
                # Update particles based on rotation
                move_particles(particles, move_distance=0, rotation_change=rotation_difference)
                time.sleep(3)
                # Update the robot's orientation
                etheta = (etheta + rotation_difference) % 360
        
        # After reorientation, you can optionally move forward or perform other actions
        # For now, we'll just return the updated orientation
        return ex, ey, etheta
    
    else:
        # Not in any block's center area; continue obstacle avoidance
        return ex, ey, etheta

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


def localization(NUM_PARTICLES=1000):
    
    ############## Main section for the open loop control algorithm ##############
    LOOP_PAUSE_TIME = 0.3 # seconds
    # Main loop
    RUNNING = True
    
    rpi_ip = '172.20.10.3'      # Raspberry Pi's IP address when connected to Baqir's iPhone hotspot
    #rpi_ip = '100.67.145.12'    # Raspberry Pi's IP address when connected to UofT wifi (IP MAY CHANGE, USE 'ip a' ON RPI TERMINAL TO FIND NEW UOFT IP)
    port = 8888
    
    particles = initialize_particles(NUM_PARTICLES)   # Initialize particles

    threshold = 7.5*25.4
    diag_threshold = 5.3*25.4
    NUM_STEPS = 786
    RESAMPLE_INTERVAL = 4
    iteration = 0
    convergence_condition = False
    sigma = 20
    j = 0
    manual_control = False
    divider = 3
    
    robot_readings = scan_rplidar()
    sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl = section_scans(robot_readings)  # Check robot sensors
    print("1 : ", sensor_front, sensor_frontr, sensor_right, sensor_backr, sensor_back, sensor_backl, sensor_left, sensor_frontl)

    try:
        for i in range(NUM_STEPS):
            print("Loop starts")
            #print(f"robot_readings{robot_readings}")
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
                            if sensor_right > 10*12*25.4:
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
                            if sensor_left > 10*12*25.4:
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
            for scan in robot_readings:         # Grab robot distances and angles to compare with partilces in localization
                if scan is not None:
                    pfdistances.append(scan/25.4)
                    pfangles.append(l*6)
                l += 1

            pfdistances = pfdistances[0::divider]     # Don't need all 60 readings for localization, takes way too long to process
            pfangles = pfangles[0::divider]
            
            
            if iteration == 0:
                update_particle_weights(particles, pfdistances, pfangles, sigma)  # Calculate particle weights
                particles = resample_particles(particles)   # Regenerate particles with weight
            if iteration % RESAMPLE_INTERVAL == 0 and iteration > 0:
                j += 1
                update_particle_weights(particles, pfdistances, pfangles, sigma)  # Calculate particle weights
                particles = resample_particles(particles)   # Regenerate particles with weight
                if j == 4:
                    particles = sorted(particles, key=lambda p: p.weight, reverse=True)[:50]
                    convergence_condition = True
                # elif j == 3:
                    
                elif j == 2:
                    sigma = 5
                    particles = sorted(particles, key=lambda p: p.weight, reverse=True)[:500]
                    RESAMPLE_INTERVAL = 2
                elif j == 1:
                    sigma = 8
                    #particles = sorted(particles, key=lambda p: p.weight, reverse=True)[:500]
                    
                    
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
                divider = 5
                time.sleep(LOOP_PAUSE_TIME)
                #break
                # Determine current block based on final position
                current_block = get_current_block_if_elif(eposition[0], eposition[1])
                
                # Reorient based on the current block
                handle_block_reorientation(eposition[0], eposition[1], eorientation, particles)
                
                # Update display after navigation
                canvas.fill(CONFIG.background_color)  # Clear screen
                draw_maze()                           # Draw maze
                draw_particles_on_canvas(particles)   # Draw particles
                
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
                    
                    theta_radians = math.radians(eorientation)
                    x2 = ex_screen + radius_pixels * math.cos(theta_radians)
                    y2 = ey_screen - radius_pixels * math.sin(theta_radians)
                    pygame.draw.line(canvas, (0, 0, 255), (ex_screen, ey_screen), (x2, y2), width=2)
                    
                    pygame.display.flip()
                
                
                print("CURRENT BLOCK!!!!!!: ",current_block)
                # Add a condition to exit the path finding loop, e.g., reaching the loading zone
                if current_block in ['block11', 'block21', 'block31', 'block12', 'block22', 'block13']:
                    print(f"Reached a loading zone block: {current_block}.")
                    PATH_FINDING = False
                    manual_control = True
                
                time.sleep(LOOP_PAUSE_TIME)
                
                
                # Clear the canvas to prepare for localization mode
                canvas.fill(CONFIG.background_color)

                
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
                            move_particles(particles, move_distance=-1.0, rotation_change=0)
                        elif event.key == pygame.K_p:
                            command = f'rotate_left: {abs(int(360))}\n'
                            send_command(rpi_ip, port, command)
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

                        pfdistances = pfdistances[0::5]     # Don't need all 60 readings for localization, takes way too long to process
                        pfangles = pfangles[0::5]

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
                        
                        
                        # us_sensor = send_command(rpi_ip, port, command="obs_moveForward")
                        scan = float(us_sensor)
                        print(scan)
                                    
                        if scan < 7.0 and sensor_front - scan > 2.0:
                            print("Block Might Be Ahead")
                            rescan = float(send_command(rpi_ip, port, command="scan"))
                            lidar_rescan = scan_rplidar()[0]

                            if rescan < 4.0 and lidar_rescan - rescan > 2.0:
                                print("BLOCK!!!!!")
                            
                    # Update Pygame display
                    pygame.display.flip()
                
                #return (eposition, eorientation)
            
            
            

    except KeyboardInterrupt:
        print("Interrupted by user.")

    #except Exception as e:
     #   print(f"An unexpected error occurred: {e}")

    finally:
        #pygame.quit()
        print("Localization complete.")


localization()