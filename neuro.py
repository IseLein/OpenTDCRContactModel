
import pygame
import sys
import numpy as np
from utils import mod_cr
from utils.node_definition import Node
from utils.taskspace import TaskspaceCircle
from utils.obstacle_definition import Circle

# Scale factor for converting simulation units to pixels
scale_factor = 200
screen_width, screen_height = 800, 600

def sim_to_pixel(x, y):
    return int(x * scale_factor + screen_width / 2), int(screen_height - (y * scale_factor))

def pixel_to_sim(x, y):
    return (x - screen_width / 2) / scale_factor, (screen_height - y) / scale_factor

def get_robot_points_pygame(node):
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


def main():
    # Initialize Pygame
    pygame.init()

    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Robot Aided Surgery")

    # Grid settings
    grid_size = 20  # Size of the grid squares
    grid_color = (230, 230, 230)  # Light grey color for the grid lines

    # Load an image (make sure the image is in the same folder or provide the path)
    image = pygame.image.load('mri.jpg')
    image_width, image_height = image.get_width(), image.get_height()
    image = pygame.transform.scale(image, (image_width // 5, image_height // 5))
    image_rect = image.get_rect(center=(screen_width // 2, screen_height // 2))

    # Define the font for displaying coordinates
    font = pygame.font.Font(None, 36)

    # Button settings
    button_width, button_height = 150, 50
    button_color = (0, 200, 0)  # Green button color
    button_rect = pygame.Rect(screen_width - button_width - 10, 10, button_width, button_height)

    alert_message = ""  # Variable to store alert message

    # robot stuff
    obstacles = []
    goal = []
    state = 'goal'

    # -- opencr stuff --

    # Robot definition (replace with your actual robot parameters)
    robot_radius = 0.006
    num_disks = 5
    robot = mod_cr.Robot(robot_radius, num_disks)
    initial_length = 0.1
    initial_tendon = 0.1
    robot_node = Node(robot, initial_length, initial_tendon)
    robot_node.set_init_guess([0.0, 0.0, 0.0, 0.01, 0.01])

    taskspace = TaskspaceCircle()

    # Main game loop
    running = True
    while running:
        screen.fill((255, 255, 255))  # Fill the screen with white
        # Draw the grid
        for x in range(0, screen_width, grid_size):  # Vertical lines
            pygame.draw.line(screen, grid_color, (x, 0), (x, screen_height), 1)
        for y in range(0, screen_height, grid_size):  # Horizontal lines
            pygame.draw.line(screen, grid_color, (0, y), (screen_width, y), 1)

        # Draw the image
        screen.blit(image, image_rect)  # Draw the image in the center

        # Display the coordinates on the screen
        app_text = font.render("Robot Aided Surgery", True, (0, 0, 0))
        screen.blit(app_text, (10, 10))  # Display the app name at the top left
        coordinates_text = font.render(f"X: {pygame.mouse.get_pos()[0]} Y: {pygame.mouse.get_pos()[1]}", True, (0, 0, 0))
        screen.blit(coordinates_text, (10, screen_height - 40))  # Show coordinates at the bottom left
        alert_text = font.render(alert_message, True, (255, 0, 0))  # Display alert message
        screen.blit(alert_text, (screen_width - alert_text.get_width() - 10, screen_height- alert_text.get_height() - 10))  # Display alert message

        # Draw the button (top-right)
        pygame.draw.rect(screen, button_color, button_rect)
        button_text = font.render("Path gen", True, (255, 255, 255))
        button_text_rect = button_text.get_rect(center=button_rect.center)
        screen.blit(button_text, button_text_rect)

        # draw obstacles and goal
        for obstacle in obstacles:
            pygame.draw.circle(screen, (0, 55, 255), obstacle, 10)
        for g in goal:
            pygame.draw.circle(screen, (0, 255, 55), g, 10)

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if button_rect.collidepoint(event.pos):
                    print("Button clicked!")
                    if goal == []:
                        alert_message = "Please select a goal"
                    else:
                        for obstacle in obstacles:
                            obs_x, obs_z = pixel_to_sim(obstacle[0], obstacle[1])
                            print(f"Obstacle_r: {obstacle}")
                            print(f"Obstacle_s: {obs_x}, {obs_z}")
                            obs = Circle(10 / scale_factor, (obs_x, 0, obs_z))
                            taskspace.set_obstacles(obs, 1, 1)
                else:
                    # Print the coordinates when clicked
                    x, y = event.pos
                    print(f"Clicked at: X={x}, Y={y}")
                    if state == 'goal':
                        goal.append((x, y))
                        state = 'obstacle'
                        print(f"Goal: {goal}")
                    elif state == 'obstacle':
                        obstacles.append((x, y))
                        print(f"Obstacle: {obstacles}")

        # Update the display
        pygame.display.flip()

    # Quit Pygame
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
