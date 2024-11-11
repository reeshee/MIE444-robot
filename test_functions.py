## test_functions.py
#import numpy as np
#from PFLocalization import initialize_particles
#import config as CONFIG
#
#def test_initialize_particles():
#    # Define number of particles to initialize
#    num_particles = 10
#    particles = initialize_particles(num_particles)
#
#    # Check if we get the correct number of particles
#    assert len(particles) == num_particles, f"Expected {num_particles} particles, got {len(particles)}"
#
#    # Check that each particle's position is within maze bounds
#    for particle in particles:
#        assert 0 <= particle.x <= CONFIG.maze_dim_x, f"Particle x out of bounds: {particle.x}"
#        assert 0 <= particle.y <= CONFIG.maze_dim_y, f"Particle y out of bounds: {particle.y}"
#        assert 0 <= particle.theta < 360, f"Particle theta out of bounds: {particle.theta}"
#
#    print("test_initialize_particles passed successfully.")
#
## Run the test
#if __name__ == "__main__":
#    test_initialize_particles()


#### test_visualization.py
#test_move_particles.py
import pygame
import numpy as np
from PFLocalization import initialize_particles, move_particles, draw_particles
from maze import Maze
import config as CONFIG

# Initialize Pygame and set up the canvas
pygame.init()
canvas = pygame.display.set_mode([
   CONFIG.maze_dim_x * CONFIG.ppi + 2 * CONFIG.border_pixels,
   CONFIG.maze_dim_y * CONFIG.ppi + 2 * CONFIG.border_pixels
])
pygame.display.set_caption("Particle Movement Test")

# Initialize the maze and draw it
maze = Maze()
maze.import_walls()
maze.generate_floor()

def draw_maze(canvas):
   """Draw the maze walls and floor on the canvas."""
   canvas.fill(CONFIG.background_color)
   maze.draw_floor(canvas)
   maze.draw_walls(canvas)

def main():
   # Generate particles
   num_particles = 500
   particles = initialize_particles(num_particles)

   # Parameters for particle movement
   move_distance = 1.0  # Distance each particle moves in each step
   rotation_change = 5  # Rotation change for each particle in degrees

   # Main Pygame loop for visualization
   running = True
   while running:
       # Handle quit events
       for event in pygame.event.get():
           if event.type == pygame.QUIT:
               running = False

       # Draw maze background
       draw_maze(canvas)

       # Move particles and draw them
       move_particles(particles, move_distance, rotation_change, canvas)

       # Update the display
       pygame.display.flip()

       # Control the speed of the simulation
       pygame.time.delay(100)  # Adjust the delay to control movement speed

   # Quit Pygame when done
   pygame.quit()

if __name__ == "__main__":
   main()


#import numpy as np
#import pygame
#from maze import Maze
#import config as CONFIG
#from PFLocalization import Particle, initialize_particles, move_particles, update_particle_weights, resample_particles, get_particle_readings, draw_particles
#
#def custom_update_particle_weights(particles, readings, epsilon=1e-6):
#    # Call the original update_particle_weights without epsilon
#    update_particle_weights(particles, readings)
#    
#    # Apply epsilon to each weight to avoid zero weights
#    for particle in particles:
#        particle.weight = max(particle.weight, epsilon)
#
#    # Normalize weights
#    total_weight = sum(p.weight for p in particles)
#    for p in particles:
#        p.weight /= total_weight
#
#def test_particle_convergence():
#    # Setup Pygame canvas for visualization
#    pygame.init()
#    canvas = pygame.display.set_mode((1920, 1080))
#    
#    # Initialize the maze (assuming maze is needed for wall distances)
#    maze = Maze()
#    maze.import_walls()
#
#    # Define the true position of the robot
#    true_position = (500, 300, 90)  # Example coordinates and orientation (x, y, theta)
#
#    # Generate synthetic sensor readings for the true position
#    angles = [0, 30, 90, 180, 270, 330]
#    synthetic_readings = get_particle_readings(Particle(*true_position), angles)
#
#    # Initialize particles across the maze randomly
#    num_particles = 1000
#    particles = initialize_particles(num_particles)
#
#    # Run a loop to observe convergence
#    for i in range(5):  # Run for 5 iterations, adjust as needed
#        # Move particles (you might not need this step if focusing only on convergence)
#        move_distance = 0  # Set to 0 if we only want to update weights based on fixed readings
#        rotation_change = 0
#        move_particles(particles, move_distance, rotation_change, canvas)
#
#        # Update particle weights using the custom function
#        custom_update_particle_weights(particles, synthetic_readings)
#
#        # Resample particles to focus on areas with higher weights
#        particles = resample_particles(particles)
#
#        # Draw particles for visualization
#        draw_particles(particles, canvas)
#
#        # Print particle positions and weights for debugging
#        for p in particles[:5]:  # Print first 5 particles for brevity
#            print(f"Particle at ({p.x:.2f}, {p.y:.2f}) with weight {p.weight:.4f}")
#
#        # Pause briefly to visualize each step
#        pygame.time.delay(500)
#
#    # End the test
#    pygame.quit()
#
#if __name__ == "__main__":
#    test_particle_convergence()
#