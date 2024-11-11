import numpy as np
import pygame
from maze import Maze
import config as CONFIG

# Initialize the maze
maze = Maze()
maze.import_walls()

class Particle:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = 1.0



def initialize_particles(num_particles):
    particles = []  # Initialize an empty list to store particles
    # (x_min, x_max, y_min, y_max) for each wall segment in the maze
    wall_ranges = [
        (8, 24, 20, 40),    # (12, 24, 24, 36)
        (20, 40, 8, 28),    # (24, 36, 12, 24)
        (36, 64, 20, 40),   # (36, 60, 24, 36)
        (44, 64, 0, 16),    # (48, 60, 0, 12)
        (72, 88, 0, 16),    # (72, 84, 0, 12)
        (68, 88, 20, 52),   # (72, 84, 24, 48)
        (0, 84, 0, 4),      # Top Wall
        (92, 96, 12, 48),    # Right Wall
        (0, 96, 44, 48),    # Bottom Wall
        (0, 4, 0, 48)       # Left Wall
    ]
    # Function to check if a point is inside any wall boundary
    def is_in_wall(x, y):
        for x_min, x_max, y_min, y_max in wall_ranges:
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return True
        return False

    while len(particles) < num_particles:
        # Generate random x, y, and theta values for a new particle
        x = np.random.uniform(0, CONFIG.maze_dim_x)
        y = np.random.uniform(0, CONFIG.maze_dim_y)
        theta = np.random.uniform(0, 360)

        # Only add the particle if it's not inside any wall boundary
        if not is_in_wall(x, y):
            particles.append(Particle(x, y, theta))

    return particles  # Return the list of particles



def move_particles(particles, move_distance, rotation_change):
    for particle in particles:
        # Movement noise parameters
        distance_noise_std = 0.5  # Adjust as needed
        rotation_noise_std = 5.0  # Adjust as needed

        # Add noise to movement
        noisy_distance = move_distance + np.random.normal(0, distance_noise_std)
        noisy_rotation = rotation_change + np.random.normal(0, rotation_noise_std)

        # Move the particle
        particle.x += noisy_distance * np.cos(np.radians(particle.theta))
        particle.y += noisy_distance * np.sin(np.radians(particle.theta))
        particle.theta = (particle.theta + noisy_rotation) % 360



def get_particle_readings(particles, angles=[0, 30, 90, 150, 180, 210, 270, 330]):
    """Simulate sensor-like readings by measuring distances to walls at specific angles for all particles."""
    num_particles = len(particles)
    num_angles = len(angles)

    # Preallocate readings array (num_particles, num_angles)
    readings = np.zeros((num_particles, num_angles))

    for i, particle in enumerate(particles):
        if not (0 <= particle.x <= CONFIG.maze_dim_x and 0 <= particle.y <= CONFIG.maze_dim_y):
            # If out of bounds, set readings to max distance
            readings[i, :] = 1000
        else:
            # Calculate readings for each angle
            for j, angle in enumerate(angles):
                direction = (particle.theta + angle) % 360
                distance = maze.get_distance_to_wall(particle.x, particle.y, direction)
                readings[i, j] = distance

    return readings




def update_particle_weights(particles, robot_readings, sigma=10):
    """Updates particle weights based on similarity to robot readings using a Gaussian model.
    
    Args:
        particles (list of Particle): The particles to update.
        robot_readings (list of float): The sensor readings from the robot.
        sigma (float): Standard deviation for Gaussian, representing expected sensor noise.
    """
    num_particles = len(particles)
    sigma_squared = sigma ** 2
    gaussian_denom = 2 * sigma_squared

    # Get particle readings for all particles
    particle_readings = get_particle_readings(particles, angles=[0, 30, 90, 150, 180, 210, 270, 330])

    # Convert robot readings to a NumPy array for vector operations
    robot_readings = np.array(robot_readings)

    # Calculate the differences between particle readings and robot readings
    differences = particle_readings - robot_readings  # Shape: (num_particles, num_readings)

    # Calculate the likelihoods using a Gaussian model
    likelihoods = np.exp(-differences ** 2 / gaussian_denom)  # Apply Gaussian model element-wise

    # Compute the final weight for each particle by multiplying over the angles
    weights = np.prod(likelihoods, axis=1)

    # Normalize the weights so they sum to 1
    total_weight = np.sum(weights)
    if total_weight > 0:
        weights /= total_weight
    else:
        # Edge case where all weights are zero
        weights.fill(1.0 / num_particles)

    # Assign back the computed weights to particles
    for i, particle in enumerate(particles):
        particle.weight = weights[i]

    # Print statements for debugging
    #print("Computed particle weights:", weights)



def resample_particles(particles, min_variance=0.1, max_variance=2.0):
    """Resamples particles, retaining high-weight particles and replacing low-weight ones with new particles in high-probability regions."""
    
    # Calculate effective sample size (ESS)
    ess = calculate_ess(particles)
    N = len(particles)
    variance = max_variance * (ess / N)
    variance = max(min_variance, variance)

    # Normalize particle weights
    weights = np.array([p.weight for p in particles])
    weights /= np.sum(weights)

    # Identify high weight particles and retain them
    high_weight_particles = [p for p in particles if p.weight > (1.0 / N)]
    num_high_weight = len(high_weight_particles)
    num_to_resample = N - num_high_weight

    # Systematic resampling for low weight particles
    positions = (np.arange(num_to_resample) + np.random.uniform(0, 1)) / num_to_resample
    indexes = np.searchsorted(np.cumsum(weights), positions)

    new_particles = high_weight_particles[:]
    for idx in indexes:
        particle = particles[idx]
        # Apply variance scaling to add diversity
        new_x = particle.x + np.random.normal(0, variance)
        new_y = particle.y + np.random.normal(0, variance)
        new_theta = (particle.theta + np.random.normal(0, variance * 10)) % 360  # Adjust scaling for angle

        new_particles.append(Particle(new_x, new_y, new_theta))

    return new_particles



def draw_particles(particles, canvas):
    """Draws each particle on the Pygame canvas after erasing previous particles."""
    background_color = CONFIG.background_color  # Assuming background color is in config

    for particle in particles:
        # Erase previous particle position by drawing over it with the background color
        screen_x = int(particle.x * CONFIG.ppi + CONFIG.border_pixels)
        screen_y = int(particle.y * CONFIG.ppi + CONFIG.border_pixels)
        pygame.draw.circle(canvas, background_color, (screen_x, screen_y), 3)  # Erase with background color

        # Draw the updated particle
        color = (0, 255, 0) if particle.weight > 0.5 else (0, 128, 0)
        #radius = max(1, int(3 * particle.weight))
        radius = 3
        pygame.draw.circle(canvas, color, (screen_x, screen_y), radius)
        
    pygame.display.flip()



def calculate_ess(particles):
    weights = np.array([p.weight for p in particles])
    return 1.0 / np.sum(weights ** 2)



# def estimate_robot_position(particles, weight_threshold=0.4):
#     # Select particles above the specified weight threshold
#     high_weight_particles = [p for p in particles if p.weight > weight_threshold]

#     # If no particles are above the threshold, return None
#     if len(high_weight_particles) == 0:
#         return None

#     # Calculate the weighted average of the high-weight particles
#     sum_weights = sum(p.weight for p in high_weight_particles)
#     average_x = sum(p.x * p.weight for p in high_weight_particles) / sum_weights
#     average_y = sum(p.y * p.weight for p in high_weight_particles) / sum_weights

#     return average_x, average_y
import numpy as np

def estimate_robot_position(particles, weight_threshold=0.5):
    # Select particles above the specified weight threshold

    if len(particles) == 50:
        weight_threshold = 0
        high_weight_particles = [p for p in particles if p.weight > weight_threshold]

    else:
        high_weight_particles = [p for p in particles if p.weight > weight_threshold]
    
    
    # If no particles are above the threshold, return None
    if len(high_weight_particles) == 0:
        return None

    # Calculate the weighted average of the high-weight particles
    sum_weights = sum(p.weight for p in high_weight_particles)
    average_x = sum(p.x * p.weight for p in high_weight_particles) / sum_weights
    average_y = sum(p.y * p.weight for p in high_weight_particles) / sum_weights

    # Calculate the weighted average orientation (circular mean)
    sin_sum = sum(np.sin(np.radians(p.theta)) * p.weight for p in high_weight_particles)
    cos_sum = sum(np.cos(np.radians(p.theta)) * p.weight for p in high_weight_particles)
    average_theta = np.degrees(np.arctan2(sin_sum, cos_sum)) % 360

    #print("THERE",average_x, average_y, average_theta)
    return average_x, average_y, average_theta


# def reinitialize_particles_with_top(top_particles, num_particles, position_variance=2.0, angle_variance=10.0):
#     """
#     Reinitialize particles using top particles, adding some variance for diversity.
    
#     Args:
#         top_particles (list of Particle): The top particles to base reinitialization on.
#         num_particles (int): Total number of particles to use for reinitialization.
#         position_variance (float): Standard deviation for adding variance to position.
#         angle_variance (float): Standard deviation for adding variance to orientation.
        
#     Returns:
#         list of Particle: A list of reinitialized particles.
#     """
#     reinitialized_particles = []

#     # Add the top particles with some variance
#     for particle in top_particles:
#         for _ in range(num_particles // len(top_particles)):
#             new_x = particle.x + np.random.normal(0, position_variance)
#             new_y = particle.y + np.random.normal(0, position_variance)
#             new_theta = (particle.theta + np.random.normal(0, angle_variance)) % 360
#             reinitialized_particles.append(Particle(new_x, new_y, new_theta))

#     # If needed, add extra particles to reach num_particles
#     #while len(reinitialized_particles) < num_particles:
#     #    base_particle = top_particles[np.random.randint(0, len(top_particles))]
#     #    new_x = base_particle.x + np.random.normal(0, position_variance)
#     #    new_y = base_particle.y + np.random.normal(0, position_variance)
#     #    new_theta = (base_particle.theta + np.random.normal(0, angle_variance)) % 360
#     #    reinitialized_particles.append(Particle(new_x, new_y, new_theta))

#     return reinitialized_particles[:num_particles]

