import pygame
import numpy as np
from PFLocalization import (
    initialize_particles, move_particles, update_particle_weights, resample_particles, estimate_robot_position, calculate_ess
)
from maze import Maze
import config as CONFIG

# Initialize Pygame and set up the display
pygame.init()
canvas_width = int(CONFIG.maze_dim_x * CONFIG.ppi + 2 * CONFIG.border_pixels)
canvas_height = int(CONFIG.maze_dim_y * CONFIG.ppi + 2 * CONFIG.border_pixels)
canvas = pygame.display.set_mode((canvas_width, canvas_height))
pygame.display.set_caption("Particle Filter Localization Test")

# Initialize the font for rendering particle labels
font = pygame.font.SysFont(None, 15)

# Initialize the maze and robot parameters
maze = Maze()
maze.import_walls()

# Define the true robot position (ground truth) within the maze bounds
robot_position = np.array([89.78, 27.66])  # Starting position in inches
robot_theta = 0  # Facing 0 degrees

# Initialize particles within the maze bounds
num_particles = 300
particles = initialize_particles(num_particles)

# Movement parameters for robot and particles
move_distance = 1       # Robot movement distance per step (in inches)
rotation_change = 45    # Rotation change for robot per step (in degrees)

# Main test loop for localization
running = True
iteration = 0
RESAMPLE_ITERATION = 3
while running and iteration < 100:  # Run for 100 iterations or until closed
    # Handle quit events to properly close the Pygame window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_x:
            # Move the robot in a simulated path
            robot_theta = (robot_theta + rotation_change) % 360  # Update robot orientation
            robot_position[0] += move_distance * np.cos(np.radians(robot_theta))
            robot_position[1] -= move_distance * np.sin(np.radians(robot_theta))
            

            # Ensure the robot stays within the maze bounds
            robot_position = np.clip(robot_position, [0, 0], [CONFIG.maze_dim_x, CONFIG.maze_dim_y])

            # Move particles to follow the robot's movement pattern
            move_particles(particles, move_distance, rotation_change)
            
            # Simulate sensor readings for the robot with noise
            sensor_noise_std = 1.0  # Standard deviation of sensor noise
            robot_readings = [
                maze.get_distance_to_wall(robot_position[0], robot_position[1], (robot_theta + angle) % 360) +
                np.random.normal(0, sensor_noise_std)
                for angle in [0, 30, 90, 150, 180, 210, 270, 330]
                ]

            #print(robot_readings)
            
            ess_threshold = 0.7 * num_particles # Set threshold to 50% of total particles
            # Update particle weights based on the robot's sensor readings
            update_particle_weights(particles, robot_readings, sigma=10.0)
            
            ess = calculate_ess(particles)
            if iteration % RESAMPLE_ITERATION == 0 and iteration > 0:
                if ess < ess_threshold:
                    particles = resample_particles(particles)
                else:
                    pass    # Do not resample yet
               
            # Resample particles based on their weights
            #particles = resample_particles(particles)
            
            # Estimate robot position using top particles
            estimated_position = estimate_robot_position(particles)
            if estimated_position:
                ex, ey = estimated_position
                ex = int(ex * CONFIG.ppi + CONFIG.border_pixels)
                ey = int((CONFIG.maze_dim_y - ey) * CONFIG.ppi + CONFIG.border_pixels)
                pygame.draw.circle(canvas, (0, 0, 255), (ex, ey), 5)
            else:
                print("Cannot estimate position.")


            # Draw the maze, particles, and robot for visualization
            canvas.fill(CONFIG.background_color)
            maze.draw_walls(canvas)

            # Draw the particles as small green circles with their index
            for i, particle in enumerate(particles):
                px = int(particle.x * CONFIG.ppi + CONFIG.border_pixels)
                py = int(particle.y * CONFIG.ppi + CONFIG.border_pixels)
                pygame.draw.circle(canvas, (0, 255, 0), (px, py), 3)
                # Render and place particle index label
                #label = font.render(str(i + 1), True, (255, 255, 255))
                #canvas.blit(label, (px + 4, py - 4))

            # Draw the robot as a larger red circle
            rx = int(robot_position[0] * CONFIG.ppi + CONFIG.border_pixels)
            ry = int(robot_position[1] * CONFIG.ppi + CONFIG.border_pixels)
            pygame.draw.circle(canvas, (255, 0, 0), (rx, ry), 5)

            # Draw sensor lines from the robot to visualize readings
            for angle, distance in zip([0, 30, 90, 150, 180, 210, 270, 330], robot_readings):
                end_x = robot_position[0] + distance * np.cos(np.radians(robot_theta + angle))
                end_y = robot_position[1] + distance * np.sin(np.radians(robot_theta + angle))
                screen_end_x = int(end_x * CONFIG.ppi + CONFIG.border_pixels)
                screen_end_y = int(end_y * CONFIG.ppi + CONFIG.border_pixels)
                pygame.draw.line(canvas, (255, 0, 0), (rx, ry), (screen_end_x, screen_end_y), 1)

            # Calculate weighted average position
            sum_weights = sum(particle.weight for particle in particles)
            if sum_weights > 0:
                average_x = sum(particle.x * particle.weight for particle in particles) / sum_weights
                average_y = sum(particle.y * particle.weight for particle in particles) / sum_weights

                # Draw the estimated position as a blue circle
                ex = int(average_x * CONFIG.ppi + CONFIG.border_pixels)
                ey = int(average_y * CONFIG.ppi + CONFIG.border_pixels)
                pygame.draw.circle(canvas, (0, 0, 255), (ex, ey), 5)
            else:
                print("Sum of weights is zero, cannot compute weighted average.")

            
            # Display the updated frame
            pygame.display.flip()

            # Increment the iteration count
            iteration += 1

# Clean up and exit
pygame.quit()
