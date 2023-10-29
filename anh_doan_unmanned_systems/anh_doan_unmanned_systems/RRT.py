# -*- coding: utf-8 -*-
"""
Created on Sat Oct 28 19:40:52 2023

@author: DCM82X
"""

import numpy as np
import math as m
import matplotlib.pyplot as plt
import time

start_time = time.time()

class Node():
    def __init__(self, x: float, y: float, parent_idx: int = -1) -> None:
        self.x = x
        self.y = y
        self.parent_idx = int(parent_idx)

class Obstacle():
    def __init__(self, x_pos: float, y_pos: float, radius: float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius

    def is_inside(self, curr_x: float, curr_y: float, robot_diameter: float) -> bool:
        inflated_radius = self.radius + 0.5 * robot_diameter
        dist_from = np.sqrt((curr_x - self.x_pos) ** 2 + (curr_y - self.y_pos) ** 2)
        return dist_from <= inflated_radius

def is_not_valid(obst_list: list, x_min: int, y_min: int, x_max: int, y_max: int, x_curr: float, y_curr: float, robot_diameter: float):
    for obs in obst_list:
        if obs.is_inside(x_curr, y_curr, robot_diameter):
            return True

    return not (x_min <= x_curr <= x_max and y_min <= y_curr <= y_max)

def collision_free(obst_list, x1, y1, x2, y2, robot_diameter):
    for obs in obst_list:
        inflated_radius = obs.radius + 0.5 * robot_diameter
        if obs.is_inside((x1 + x2) / 2, (y1 + y2) / 2, robot_diameter):
            return False

    return True

def nearest_node(nodes: list, x: float, y: float):
    closest_idx = 0
    closest_dist = m.inf

    for idx, node in enumerate(nodes):
        dist = m.hypot(node.x - x, node.y - y)
        if dist < closest_dist:
            closest_dist = dist
            closest_idx = idx

    return closest_idx

def step_towards(x1: float, y1: float, x2: float, y2: float, step_size: float):
    theta = m.atan2(y2 - y1, x2 - x1)
    x_new = x1 + step_size * m.cos(theta)
    y_new = y1 + step_size * m.sin(theta)
    return x_new, y_new

# Initialize parameters
start_x = 1
start_y = 1

min_x = 0
max_x = 15

min_y = 0
max_y = 15

goal_x = 7
goal_y = 13

robot_diameter = 1.0
Obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 
8, 8, 8, 8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
Obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7, 
8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13, 
14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]
obstacle_positions= [(x, y) for x, y in zip(Obstacle_x, Obstacle_y)]

obstacle_list = [Obstacle(x, y, 0.5) for x, y in obstacle_positions]

step_size = 1.0

nodes = [Node(start_x, start_y)]
goal_reached = False

while not goal_reached:
    x_rand = np.random.uniform(min_x, max_x)
    y_rand = np.random.uniform(min_y, max_y)

    closest_idx = nearest_node(nodes, x_rand, y_rand)
    closest_node = nodes[closest_idx]

    x_new, y_new = step_towards(closest_node.x, closest_node.y, x_rand, y_rand, step_size)

    if not is_not_valid(obstacle_list, min_x, min_y, max_x, max_y, x_new, y_new, robot_diameter) and collision_free(obstacle_list, closest_node.x, closest_node.y, x_new, y_new, robot_diameter):
        new_node = Node(x_new, y_new, closest_idx)
        nodes.append(new_node)

        if m.hypot(x_new - goal_x, y_new - goal_y) <= step_size:
            print("Goal reached!")
            goal_reached = True

# Plot the results
plt.figure(figsize=(10, 10))

# Plot the grid
for x in np.arange(min_x, max_x + step_size, step_size):
    plt.axvline(x, color='gray', linestyle='--', linewidth=0.5)
for y in np.arange(min_y, max_y + step_size, step_size):
    plt.axhline(y, color='gray', linestyle='--', linewidth=0.5)

# Plot obstacles
for obs in obstacle_list:
    circle = plt.Circle((obs.x_pos, obs.y_pos), obs.radius + 0.5 * robot_diameter, color='red', alpha=0.5)
    plt.gca().add_patch(circle)

# Plot the nodes and edges
for node in nodes:
    if node.parent_idx != -1:
        parent_node = nodes[node.parent_idx]
        plt.plot([node.x, parent_node.x], [node.y, parent_node.y], color='gray', linewidth=0.5)

# Plot the path
goal_idx = nearest_node(nodes, goal_x, goal_y)
curr_idx = goal_idx
while curr_idx != -1:
    curr_node = nodes[curr_idx]
    parent_idx = curr_node.parent_idx
    if parent_idx != -1:
        parent_node = nodes[parent_idx]
        plt.plot([curr_node.x, parent_node.x], [curr_node.y, parent_node.y], color='blue', linewidth=1)
    curr_idx = parent_idx

# Plot the start and goal points
plt.plot(start_x, start_y, marker='o', markersize=10, color='green')
plt.plot(goal_x, goal_y, marker='x', markersize=10, color='red')

# Set the axis limits and show the plot
plt.xlim(min_x, max_x)
plt.ylim(min_y, max_y)
plt.grid(True)
plt.show()
# Extract the path
path = []
goal_idx = nearest_node(nodes, goal_x, goal_y)
curr_idx = goal_idx
while curr_idx != -1:
    curr_node = nodes[curr_idx]
    path.append((curr_node.x, curr_node.y))
    curr_idx = curr_node.parent_idx

# Reverse the path to start from the starting point
path.reverse()
print("Path from start to goal:")
cost = 0
prev_x, prev_y = path[0]
for p in path:
    x, y = p
    print(f"({x:.2f}, {y:.2f})")
    cost += m.hypot(x - prev_x, y - prev_y)
    prev_x, prev_y = x, y

print("Total cost of the path: {:.2f}".format(cost)) 

end_time = time.time()
execution_time = end_time - start_time

print(f"Time taken to execute A* algorithm: {execution_time} seconds")

