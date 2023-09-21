import numpy as np
import math as m
import matplotlib.pyplot as plt


class Node():
    def __init__(self, x: float, y: float, cost: float, parent_idx: int, heuristic_cost: float) -> None:
        self.x = x
        self.y = y
        self.cost = cost  # Actual cost (g)
        self.parent_idx = int(parent_idx)
        self.heuristic_cost = heuristic_cost  # Heuristic cost (h)


class Obstacle():
    def __init__(self, x_pos: float, y_pos: float, radius: float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius

    def is_inside(self, curr_x: float, curr_y: float) -> bool:
        dist_from = np.sqrt((curr_x - self.x_pos)**2 +
                            (curr_y - self.y_pos)**2)
        if dist_from > self.radius:
            return False
        return True


def compute_index(min_x: int, max_x: int, min_y: int, max_y: int,
                  gs: float, curr_x: int, curr_y: int) -> float:
    index = ((curr_x - min_x) / gs) + ((curr_y - min_y) / gs) * \
        ((max_x + gs - min_x) / gs)
    return index


def get_all_moves(current_x: float, current_y: float, gs: float) -> list:
    move_list = []
    gs_x_bounds = np.arange(-gs, gs+gs, gs)
    gs_y_bounds = np.arange(-gs, gs+gs, gs)
    for dx in gs_x_bounds:
        for dy in gs_y_bounds:
            x_next = current_x + dx
            y_next = current_y + dy
            if [x_next, y_next] == [current_x, current_y]:
                continue
            move = [x_next, y_next]
            move_list.append(move)
    return move_list


def is_not_valid(obst_list: list,
                 x_min: int,
                 y_min: int,
                 x_max: int,
                 y_max: int,
                 x_curr: float,
                 y_curr: float):
    for obs in obst_list:
        if obs.is_inside(x_curr, y_curr):
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


def calculate_heuristic_cost(node_x: float, node_y: float, goal_x: float, goal_y: float) -> float:
    return abs(node_x - goal_x) + abs(node_y - goal_y)


# initialize some parameters
start_x = 1
start_y = 1

min_x = 0
max_x = 10

min_y = 0
max_y = 10

goal_x = 9
goal_y = 8

gs = 1

obstacle_positions = [(2, 2), (2, 3), (2, 4), (5, 5),
                      (5, 6), (6, 6), (7, 6), (7, 5), (7, 4), (7, 3), (8, 6)]
obstacle_list = []  # store obstacle classes
obstacle_radius = 0.5

# Loop through position of obstacles
for obs_pos in obstacle_positions:
    obstacle = Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)
    obstacle_list.append(obstacle)

# - Make two bins/dictionaries:
unvisited = {}
visited = {}

# - Initialize current_node with the following parameters:
# - position = start position
# - cost = 0
# - parent_index = -1
current_node = Node(start_x, start_y, 0, int(-1),
                    calculate_heuristic_cost(start_x, start_y, goal_x, goal_y))

# - initialize current_index by utilizing compute_index() function
# based on the current position, which is the start
current_idx = int(compute_index(min_x, max_x, min_y, max_y,
                                gs, start_x, start_y))

# - insert current_node into unvisited dictionary,
# use the current_index as the key
unvisited[current_idx] = current_node

# - While current node position is not equal to goal location:
while [current_node.x, current_node.y] != [goal_x, goal_y]:
    # set current_index to min value from unvisited
    current_idx = min(
        unvisited, key=lambda x: unvisited[x].cost + unvisited[x].heuristic_cost)

    # - set current_node to unvisited[current_index]
    current_node = unvisited[current_idx]

    # - put current_node into visited dictionary
    visited[current_idx] = current_node

    # - delete current_index from unvisited
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

    # Begin search by doing the following:
    # - Use get_all_moves() to get all moves based on current position
    all_moves = get_all_moves(current_node.x, current_node.y, gs)

    # initialize a filtered_move list
    filtered_moves = []

    # - With all moves check if move is_not_valid by using is_not_valid() function
    # - This function should check if:
    # - Inside obstacle
    # - Outside boundary
    # - Not on top of ourselves (already done by get_all moves)
    for move in all_moves:
        if (is_not_valid(obstacle_list, min_x, min_y, max_x, max_y,
                         move[0], move[1]) == True):
            continue
        else:
            filtered_moves.append(move)

    #  - loop through all filtered moves:
    for move in filtered_moves:
        # based on this current filtered move:

        # calculate the filtered/new index
        new_index = int(compute_index(min_x, max_x, min_y, max_y,
                                      gs, move[0], move[1]))

        # calculate the filtered/new cost
        # from + to new_node
        new_cost = current_node.cost + \
            m.dist(move, [current_node.x, current_node.y])

        # check if new index is in visited
        if new_index in visited:
            continue

        # - if inside unvisited:
        if new_index in unvisited:
            # compare new_cost to unvisited cost:
            # if new_cost < unvisited cost:
            if new_cost < unvisited[new_index].cost:
                # update the cost value
                unvisited[new_index].cost = new_cost
                # update the parent index
                unvisited[new_index].parent_idx = current_idx

            # continue to next move since it already exists
            continue

        # - this last condition means that we have a node so
        #     make a new node and append to unvisited
        new_heuristic_cost = calculate_heuristic_cost(
            move[0], move[1], goal_x, goal_y)
        new_node = Node(move[0], move[1], new_cost,
                        current_idx, new_heuristic_cost)
        unvisited[new_index] = new_node

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
print("The list of waypoints for the desired path:", wp_list)
print(f"The cost is {current_node.cost:.2f}")
