import numpy as np
import matplotlib.pyplot as plt
import random

# Define the dimensions of the workspace
x_min, x_max = 0, 10
y_min, y_max = 0, 10

# Define the start and end points
start = (1, 1)
goal = (9, 8)

# Define a list of obstacles as circles with (x, y) positions and radii
obstacle_positions = [(2, 2), (2, 3), (2, 4), (5, 5),
                      (5, 6), (6, 6), (7, 6), (7, 5), (7, 4), (7, 3), (8, 6)]
obstacle_radius = 0.5

# Number of iterations for the RRT algorithm
max_iterations = 1000

# Step size for extending the tree
step_size = 0.5

# Function to check if a point is inside an obstacle


def is_collision(point, obstacles):
    for obstacle in obstacles:
        distance = np.sqrt((point[0] - obstacle[0]) **
                           2 + (point[1] - obstacle[1])**2)
        if distance < obstacle_radius:
            return True
    return False

# Define a node class to store points and parent information


class Node:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent


# Initialize the RRT with the start point
rrt = [Node(start)]

for _ in range(max_iterations):
    # Randomly sample a point in the workspace
    random_point = (random.uniform(x_min, x_max), random.uniform(y_min, y_max))

    # Find the nearest point in the RRT to the random point
    nearest_node = min(rrt, key=lambda node: np.sqrt(
        (node.point[0] - random_point[0])**2 + (node.point[1] - random_point[1])**2))

    # Extend the tree from the nearest point towards the random point
    theta = np.arctan2(
        random_point[1] - nearest_node.point[1], random_point[0] - nearest_node.point[0])
    new_point = (nearest_node.point[0] + step_size * np.cos(theta),
                 nearest_node.point[1] + step_size * np.sin(theta))

    # Check for collisions with obstacles
    if not is_collision(new_point, obstacle_positions):
        rrt.append(Node(new_point, parent=nearest_node))

# Find the path from the goal to the nearest point in the RRT
path = [goal]
nearest_to_goal = min(rrt, key=lambda node: np.sqrt(
    (node.point[0] - goal[0])**2 + (node.point[1] - goal[1])**2))
while path[-1] != start:
    for node in rrt:
        if np.sqrt((node.point[0] - path[-1][0])**2 + (node.point[1] - path[-1][1])**2) < step_size and node.point != path[-1]:
            path.append(node.point)
            break

# Extract the x and y coordinates from the path
path_x, path_y = zip(*path)

# Plot the workspace, obstacles, RRT, and path
plt.figure(figsize=(10, 10))
plt.xlim(x_min - 0.5, x_max + 0.5)
plt.ylim(y_min - 0.5, y_max + 0.5)
plt.xlabel('X-axis')
plt.ylabel('Y-axis')

# Plot obstacles
for obstacle in obstacle_positions:
    circle = plt.Circle(obstacle, obstacle_radius, color='red', alpha=0.5)
    plt.gca().add_patch(circle)

# Plot RRT
for node in rrt:
    if node.parent:
        plt.plot([node.point[0], node.parent.point[0]], [
                 node.point[1], node.parent.point[1]], color='blue')

# Plot path
plt.plot(path_x, path_y, color='green', linewidth=2)

# Plot start and goal
plt.scatter(start[0], start[1], color='green', marker='o', label='Start')
plt.scatter(goal[0], goal[1], color='orange', marker='o', label='Goal')

plt.gca().set_aspect('equal', adjustable='box')
plt.legend()
plt.grid(True)
plt.show()
