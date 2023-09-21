# -*- coding: utf-8 -*-
import numpy as np
import math as m
import matplotlib.pyplot as plt


class Node():
    def __init__(self, x: float, y: float, cost: float, parent_idx: int) -> None:
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_idx = int(parent_idx)


class Obstacle():
    def __init__(self, x_pos: float, y_pos: float, radius: float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius

    def is_inside(self, curr_x: float, curr_y: float, robot_diameter: float) -> bool:
        inflated_radius = self.radius + 0.5 * robot_diameter
        dist_from = np.sqrt((curr_x - self.x_pos) ** 2 +
                            (curr_y - self.y_pos) ** 2)

        if dist_from > inflated_radius:
            return False

        return True


def compute_index(min_x: int, max_x: int, min_y: int, max_y: int, gs: float, curr_x: int, curr_y: int) -> float:
    index = ((curr_x - min_x) / gs) + ((curr_y - min_y) / gs) * \
        ((max_x + gs - min_x) / gs)
    return index


def get_all_moves(current_x: float, current_y: float, gs: float) -> list:
    move_list = []
    gs_x_bounds = np.arange(-gs, gs + gs, gs)
    gs_y_bounds = np.arange(-gs, gs + gs, gs)

    for dx in gs_x_bounds:
        for dy in gs_y_bounds:
            x_next = current_x + dx
            y_next = current_y + dy

            if [x_next, y_next] == [current_x, current_y]:
                continue

            move = [x_next, y_next]
            move_list.append(move)

    return move_list


def is_not_valid(obst_list: list, x_min: int, y_min: int, x_max: int, y_max: int, x_curr: float, y_curr: float, robot_diameter: float):
    for obs in obst_list:
        if obs.is_inside(x_curr, y_curr, robot_diameter):
            return True

    if x_min > x_curr:
        return True
    if x_max < x_curr:
        return True
    if y_min > y_curr:
        return True
    if y_max < y_curr:
        return True

    return False

# Heuristic function (Euclidean distance to the goal)


def heuristic(node, goal_x, goal_y):
    return np.sqrt((node.x - goal_x) ** 2 + (node.y - goal_y) ** 2)


# Initialize parameters
start_x = 0
start_y = 0

min_x = 0
max_x = 10

min_y = 0
max_y = 10

goal_x = 8
goal_y = 9

gs = 0.5

robot_diameter = 1.0

obstacle_positions = [(1, 1), (4, 4), (3, 4), (5, 0),
                      (5, 1), (0, 7), (1, 7), (2, 7), (3, 7)]
obstacle_list = []  # Store obstacle classes
obstacle_radius = 0.25

# Loop through position of obstacles
for obs_pos in obstacle_positions:
    obstacle = Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)
    obstacle_list.append(obstacle)

# Make two bins/dictionaries
unvisited = {}
visited = {}

# Initialize current_node
current_node = Node(start_x, start_y, 0, int(-1))

# Initialize current_index
current_idx = int(compute_index(
    min_x, max_x, min_y, max_y, gs, start_x, start_y))

# Insert current_node into unvisited dictionary with the estimated cost as priority
unvisited[current_idx] = current_node

# While current node position is not equal to goal location
while [current_node.x, current_node.y] != [goal_x, goal_y]:
    current_idx = min(
        unvisited, key=lambda x: unvisited[x].cost + heuristic(unvisited[x], goal_x, goal_y))
    current_node = unvisited[current_idx]
    visited[current_idx] = current_node
    del unvisited[current_idx]

    if [current_node.x, current_node.y] == [goal_x, goal_y]:
        wp_node = current_node
        wp_list = []
        wp_list.append([wp_node.x, wp_node.y])

        while wp_node.parent_idx != -1:
            next_idx = wp_node.parent_idx
            wp_node = visited[next_idx]
            wp_list.append([wp_node.x, wp_node.y])
        wp_list.reverse()

        break

    # Begin search by getting all possible moves
    all_moves = get_all_moves(current_node.x, current_node.y, gs)

    filtered_moves = []

    # Check if moves are valid
    for move in all_moves:
        if is_not_valid(obstacle_list, min_x, min_y, max_x, max_y, move[0], move[1], robot_diameter):
            continue
        else:
            filtered_moves.append(move)

    for move in filtered_moves:
        new_index = int(compute_index(
            min_x, max_x, min_y, max_y, gs, move[0], move[1]))
        new_cost = current_node.cost + \
            m.dist(move, [current_node.x, current_node.y])

        if new_index in visited:
            continue

        if new_index in unvisited:
            if new_cost < unvisited[new_index].cost:
                unvisited[new_index].cost = new_cost
                unvisited[new_index].parent_idx = current_idx
            continue

        new_node = Node(move[0], move[1], new_cost, current_idx)
        unvisited[new_index] = new_node

# Inflate the obstacles for plotting
for obs in obstacle_list:
    obs.radius += 0.5 * robot_diameter
min_x -= 0.5 * robot_diameter
max_x += 0.5 * robot_diameter
min_y -= 0.5 * robot_diameter
max_y += 0.5 * robot_diameter

# Plot the grid, obstacles, and the path
plt.figure(figsize=(10, 10))

# Plot the grid
for x in np.arange(min_x, max_x + gs, gs):
    plt.axvline(x, color='gray', linestyle='--', linewidth=0.5)
for y in np.arange(min_y, max_y + gs, gs):
    plt.axhline(y, color='gray', linestyle='--', linewidth=0.5)

# Plot obstacles
for obs in obstacle_list:
    circle = plt.Circle((obs.x_pos, obs.y_pos),
                        obs.radius, color='red', alpha=0.5)
    plt.gca().add_patch(circle)

# Plot the path
path_x = [point[0] for point in wp_list]
path_y = [point[1] for point in wp_list]
plt.plot(path_x, path_y, color='blue', linewidth=2, label='Path')

# Plot start and goal
plt.scatter(start_x, start_y, color='green', marker='o', label='Start')
plt.scatter(goal_x, goal_y, color='orange', marker='o', label='Goal')

plt.xlim(min_x - 0.5, max_x + 0.5)
plt.ylim(min_y - 0.5, max_y + 0.5)
plt.gca().set_aspect('equal', adjustable='box')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.grid(True)
plt.show()

print("The list of waypoints for the desired path for the A* algorithm is :", wp_list)
print(f"The cost is {current_node.cost:.2f}")
