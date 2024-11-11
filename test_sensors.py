# test_particle_wall_sensing_adjusted_debug.py

import numpy as np
import pygame
from maze import Maze
from PFLocalization import Particle, get_particle_readings
import config as CONFIG

# Initialize the maze and set the resolution to fit it precisely
maze = Maze()
maze.import_walls()
maze_width = CONFIG.maze_dim_x * CONFIG.ppi + 2 * CONFIG.border_pixels
maze_height = CONFIG.maze_dim_y * CONFIG.ppi + 2 * CONFIG.border_pixels

# Set up Pygame display
pygame.init()
canvas = pygame.display.set_mode((int(maze_width), int(maze_height)))
pygame.display.set_caption("Particle Wall Sensing Test (Debug)")

# Define particles with random positions within the maze dimensions
#np.random.seed(0)  # Seed for reproducibility
test_particles = [
    Particle(np.random.uniform(0, CONFIG.maze_dim_x), np.random.uniform(0, CONFIG.maze_dim_y), np.random.uniform(0, 360))
    for _ in range(5)
]

# Angles to check (0° is forward for each particle)
angles = [0, 30, 90, 180, 270, 330]

# Run sensing test and print results
for i, particle in enumerate(test_particles):
    readings = get_particle_readings(particle, angles)
    print(f"\nParticle {i+1} at ({particle.x:.2f}, {particle.y:.2f}) with orientation θ={particle.theta:.2f}°")
    print(f"  Sensing distances at angles {angles}:")
    
    for angle, reading in zip(angles, readings):
        print(f"    Angle {angle}°: Distance to wall = {reading:.2f}")

# Visualization setup
canvas.fill(CONFIG.background_color)
maze.draw_floor(canvas)
maze.draw_walls(canvas)

# Draw each particle and its sensing lines
for particle in test_particles:
    # Convert particle position to screen coordinates
    screen_x = int(particle.x * CONFIG.ppi + CONFIG.border_pixels)
    screen_y = int(particle.y * CONFIG.ppi + CONFIG.border_pixels)
    
    # Draw particle as a large green dot
    pygame.draw.circle(canvas, (0, 255, 0), (screen_x, screen_y), 10)
    
    # Draw sensing lines for each angle
    for angle, reading in zip(angles, get_particle_readings(particle, angles)):
        # Print distance to help in debugging
        print(f"Particle at ({particle.x:.2f}, {particle.y:.2f}), angle {angle}° -> Wall distance: {reading:.2f}")
        
        # Adjust scaling to visualize the exact reading distance
        dx = int(reading * CONFIG.ppi * np.cos(np.radians(particle.theta + angle)))
        dy = int(reading * CONFIG.ppi * np.sin(np.radians(particle.theta + angle)))
        end_x = screen_x + dx
        end_y = screen_y + dy
        pygame.draw.line(canvas, (255, 0, 0), (screen_x, screen_y), (end_x, end_y), 2)

pygame.display.flip()

# Keep the window open until the user closes it
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

pygame.quit()
