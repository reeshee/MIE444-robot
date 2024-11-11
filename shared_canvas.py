# shared_canvas.py
import pygame
import config as CONFIG

# Check if Pygame is already initialized
if not pygame.get_init():
    pygame.init()

# Use the existing display if available
if not pygame.display.get_surface():
    canvas = pygame.display.set_mode([
        CONFIG.maze_dim_x * CONFIG.ppi + 2 * CONFIG.border_pixels,
        CONFIG.maze_dim_y * CONFIG.ppi + 2 * CONFIG.border_pixels
    ])
    pygame.display.set_caption("SimMeR Canvas")
else:
    canvas = pygame.display.get_surface()
