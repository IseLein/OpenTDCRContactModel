
import pygame
import sys
import os
import numpy as np
from utils import mod_cr
from utils.node_definition import Node
from utils.taskspace import TaskspaceCircle
from utils.obstacle_definition import Circle
import pandas as pd # Make sure to import pandas

# Scale factor for converting simulation units to pixels
scale_factor = 10000
screen_width, screen_height = 800, 600

def sim_to_pixel(x, y):
    return int(x * scale_factor + screen_width / 2), int(screen_height - (y * scale_factor))

def pixel_to_sim(x, y):
    return (x - screen_width / 2) / scale_factor, (screen_height - y) / scale_factor

def load_and_extract(filename):
    # Load the CSV file
    df = pd.read_csv(filename, header=None)
    return df.to_numpy()

def get_robot_points_pygame(node):
    """
    Convert robot coordinates calculated by positionCalc to Pygame coordinates.
    Uses node.var (curvature values), node.l (segment length), node.nd (num disks),
    and node.tendon (tendon positions) from the Node object.
    """
    if not hasattr(node, 'var') or node.var is None:
        return []

    l_inter = node.l / node.nd  # Segment length for each disk
    var = node.var  # Curvature values (already in node.var after solving)
    nd = node.nd
    pTendon = node.tendon

    pCoord, _ = mod_cr.positionCalc(l_inter, nd, var, pTendon) # Get backbone coordinates

    points = []
    for i in range(nd):
        x = pCoord[0, i]
        z = pCoord[2, i]
        pygame_x, pygame_y = sim_to_pixel(x, z) # Use your sim_to_pixel function
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
    obstacles = [(370, 470), (430, 470)]
    goal = []
    state = 'goal'

    # -- opencr stuff --

    # Robot definition (replace with your actual robot parameters)
    robot_radius = 0.001
    num_disks = 20
    robot = mod_cr.Robot(robot_radius, num_disks)
    initial_length = 0.1
    initial_tendon = 0.1
    robot_node = Node(robot, initial_length, initial_tendon)
    robot_node.set_init_guess(np.array([0.3]*robot.nd))
    robot_node.T = np.eye(4)

    taskspace = TaskspaceCircle()
    for obstacle in obstacles:
        obs_x, obs_z = pixel_to_sim(obstacle[0], obstacle[1])
        print(f"Obstacle_r: {obstacle}")
        print(f"Obstacle_s: {obs_x}, {obs_z}")
        obs = Circle(10 / scale_factor, (obs_x, 0, obs_z))
        taskspace.set_obstacles(obs, 1, 0.0)
    robot_node.run_forward_model(taskspace, True, "KINEMATIC_CPP")

    # Main game loop
    found_goal = False
    running = True
    while running:
        screen.fill((255, 255, 255))  # Fill the screen with white

        # draw rectangle
        pygame.draw.rect(screen, (100, 40, 100), (360, 570, 80, 20))

        # Draw the image
        screen.blit(image, image_rect)  # Draw the image in the center

        # Draw the grid
        for x in range(0, screen_width, grid_size):  # Vertical lines
            pygame.draw.line(screen, grid_color, (x, 0), (x, screen_height), 1)
        for y in range(0, screen_height, grid_size):  # Horizontal lines
            pygame.draw.line(screen, grid_color, (0, y), (screen_width, y), 1)

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
            pygame.draw.circle(screen, (0, 255, 55), g, 20)

        robot_points_pygame = get_robot_points_pygame(robot_node)
        if robot_points_pygame and found_goal:
            pygame.draw.lines(screen, (100, 20, 100), False, robot_points_pygame, 10)

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
                            taskspace.set_obstacles(obs, 1, 0.0)
                        goal_x_sim, goal_z_sim = pixel_to_sim(goal[0][0], goal[0][1])
                        target_radius_sim = 0.01
                        target_orientation = 0
                        target_coords_sim = [goal_x_sim, 0, goal_z_sim]
                        # robot_node.run_forward_model(taskspace, True, "KINEMATIC_CPP")
                        taskspace.set_target(target_coords_sim, target_radius_sim, target_orientation)
                        path_filename = 'pygame_generated_path.csv'
                        taskspace.generate_path(robot_node, target=target_coords_sim, target_radius=target_radius_sim, filename=path_filename, max_iter=250)

                        if os.path.exists(path_filename): # Check if file was created
                            sample_path = load_and_extract(path_filename) # Implement load_and_extract or similar
                            print(f"Path generated and loaded from {path_filename}")
                            if len(sample_path) > 0:
                                last_path_point = sample_path[-1]
                                new_robot_node = Node(robot, last_path_point[0], last_path_point[1])
                                new_robot_node.set_init_guess(robot_node.var[0,::3])
                                if new_robot_node.run_forward_model(taskspace, True, "KINEMATIC_CPP"):
                                    print("draw path")
                                    robot_node = new_robot_node
                                    found_goal = True

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
