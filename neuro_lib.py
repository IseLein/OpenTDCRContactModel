
import pygame
import numpy as np
from utils import mod_cr
from utils.node_definition import Node
from utils.taskspace import TaskspaceCircle
from utils.obstacle_definition import Circle

pygame.init()

# Screen dimensions
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Continuum Robot Simulation")

# Colors
white = (255, 255, 255)
black = (0, 0, 0)
blue = (0, 0, 255)
red = (255, 0, 0)

# Scale factor for converting simulation units to pixels
scale_factor = 200

# Robot definition (replace with your actual robot parameters)
robot_radius = 0.006
num_disks = 10
robot = mod_cr.Robot(robot_radius, num_disks)

# Initial node
initial_length = 0.1
initial_tendon = 0.1
robot_node = Node(robot, initial_length, initial_tendon)
robot_node.set_init_guess(np.zeros(num_disks))  # Initial guess (all zeros)

# Taskspace with a single obstacle
taskspace = TaskspaceCircle()
obstacle = Circle(0.02, [0.2, 0, 0.2])  # Radius, x, y, z (y is ignored in 2D)
taskspace.set_obstacles(obstacle, 1, 0)

def get_robot_points_pygame(node, screen_width, screen_height, scale_factor):
    """Convert robot coordinates to Pygame coordinates."""
    if not hasattr(node, 'coordinates') or node.coordinates is None:
        return []

    points = []
    for i in range(node.nd):
        x = node.coordinates[0, i]
        z = node.coordinates[2, i]
        pygame_x = int(x * scale_factor + screen_width / 4)
        pygame_y = int(screen_height - (z * scale_factor + screen_height / 4))
        points.append((pygame_x, pygame_y))

    return points

def draw_obstacles(screen, obstacles, scale_factor, screen_width, screen_height):
    """Draw obstacles on the Pygame screen."""
    for obstacle in obstacles:
        x, _, z = obstacle.p  # Extract x and z coordinates from obstacle center
        radius = obstacle.ab[0]  # Extract radius from obstacle definition

        # Convert simulation coordinates to Pygame coordinates
        pygame_x = int(x * scale_factor + screen_width / 4)
        pygame_y = int(screen_height - (z * scale_factor + screen_height / 4))
        pygame.draw.circle(screen, red, (pygame_x, pygame_y), int(radius * scale_factor))

# Pygame main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Run forward kinematic model
    success = robot_node.run_forward_model(taskspace, True, "KINEMATIC_CPP")

    # Clear the screen
    screen.fill(white)

    # Draw obstacles
    draw_obstacles(screen, taskspace.obstacle, scale_factor, screen_width, screen_height)

    # Draw robot
    if success:
        robot_points = get_robot_points_pygame(robot_node, screen_width, screen_height, scale_factor)
        if robot_points:
            pygame.draw.lines(screen, blue, False, robot_points, 2)

    # Update the display
    pygame.display.flip()

pygame.quit()
