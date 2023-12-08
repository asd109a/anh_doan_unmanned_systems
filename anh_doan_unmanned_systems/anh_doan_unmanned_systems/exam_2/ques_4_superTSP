from itertools import permutations
import math as m
import time 
import numpy as np
import matplotlib.pyplot as plt

waypoints = [(1,1), (9,7), (1,9), (4,4), (9,4), 
             (6,14), (3,11), (14,1), 
             (1,14), (14,14), (7,10)]

other_waypoints = {}
cost_dictionary = {}

#Compute all the costs from some waypoint to all other waypoints
compute_times = 0
for wp in waypoints: 
    for other_wp in waypoints:
        if wp == other_wp:
            continue
        if (wp, other_wp) in cost_dictionary:
            continue
        if (other_wp, wp) in cost_dictionary:
            continue
        else:
            compute_times += 1
            ## this would be where you plug in astar
            #waypoints = call_astar
            #total distance = sum of all the distances in the path
            total_distance = m.dist(wp, other_wp)
            #a->c
            cost_dictionary[wp, other_wp] = total_distance
            #c->a 
            cost_dictionary[other_wp, wp] = total_distance

# Compute all the possible paths
paths = list(permutations(waypoints, len(waypoints)))
total_cost = []
print("Number of compute times: ", compute_times)
print("Number of paths: ", len(paths))


n_iterations = 100

start_time = time.time()
for j, path in enumerate(paths):    
    sum_cost = 0
    if j % n_iterations == 0:
        print("Iteration: ", j)
    
    for i in range(len(path)-1):
        sum_cost += cost_dictionary[path[i], path[i+1]]
        
    total_cost.append(sum_cost)
        
end_time = time.time()
print("Time elapsed: ", end_time - start_time)
# get best path
min_total_cost = min(total_cost)
min_total_cost_index = total_cost.index(min_total_cost)
best_path = paths[min_total_cost_index]
print("best path", best_path)

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
        dist_from = np.sqrt((curr_x - self.x_pos) ** 2 + (curr_y - self.y_pos) ** 2)
        return dist_from <= inflated_radius

def compute_index(min_x: int, max_x: int, min_y: int, max_y: int, gs: float, curr_x: int, curr_y: int) -> float:
    return ((curr_x - min_x) / gs) + ((curr_y - min_y) / gs) * ((max_x + gs - min_x) / gs)

def get_all_moves(current_x: float, current_y: float, gs: float) -> list:
    gs_x_bounds = np.arange(-gs, gs + gs, gs)
    gs_y_bounds = np.arange(-gs, gs + gs, gs)
    return [[current_x + dx, current_y + dy] for dx in gs_x_bounds for dy in gs_y_bounds if [dx, dy] != [0, 0]]

def is_not_valid(obst_list: list, x_min: int, y_min: int, x_max: int, y_max: int, x_curr: float, y_curr: float, robot_diameter: float):
    for obs in obst_list:
        if obs.is_inside(x_curr, y_curr, robot_diameter):
            return True
    return not (x_min <= x_curr <= x_max and y_min <= y_curr <= y_max)

def heuristic(node, goal_x, goal_y):
    return np.sqrt((node.x - goal_x) ** 2 + (node.y - goal_y) ** 2)


def distance(point1, point2):
    return m.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    

start_x = 1
start_y = 1
min_x = 0
max_x = 15
min_y = 0
max_y = 15
goals = best_path
gs = 0.5
robot_diameter = 1.0
Obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8,
              8, 8, 8, 2, 3, 4, 5, 6, 9, 10, 11, 12, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
              6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
Obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6,
              7, 8, 9, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,
              12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]
obstacle_positions = list(zip(Obstacle_x, Obstacle_y))
obstacle_radius = 0.0
obstacle_list = [Obstacle(x, y, obstacle_radius) for x, y in obstacle_positions]

# Function to execute A* for a single goal
def execute_astar(start_x, start_y, goal_x, goal_y):
    unvisited = {}
    visited = {}
    current_node = Node(start_x, start_y, 0, int(-1))
    current_idx = int(compute_index(min_x, max_x, min_y, max_y, gs, start_x, start_y))
    unvisited[current_idx] = current_node

    while [current_node.x, current_node.y] != [goal_x, goal_y]:
        current_idx = min(unvisited, key=lambda o: unvisited[o].cost + heuristic(unvisited[o], goal_x, goal_y))
        current_node = unvisited[current_idx]
        visited[current_idx] = current_node
        del unvisited[current_idx]

        all_moves = get_all_moves(current_node.x, current_node.y, gs)
        for move in all_moves:
            if is_not_valid(obstacle_list, min_x, min_y, max_x, max_y, move[0], move[1], robot_diameter):
                continue
            new_index = int(compute_index(min_x, max_x, min_y, max_y, gs, move[0], move[1]))
            new_cost = current_node.cost + m.dist(move, [current_node.x, current_node.y])
            if new_index in visited:
                continue
            if new_index in unvisited and new_cost >= unvisited[new_index].cost:
                continue
            new_node = Node(move[0], move[1], new_cost, current_idx)
            unvisited[new_index] = new_node

    wp_list = [[current_node.x, current_node.y]]
    while current_node.parent_idx != -1:
        current_node = visited[current_node.parent_idx]
        wp_list.append([current_node.x, current_node.y])
    return wp_list[::-1]

# Iterate over the goals
final_path = []
current_start_x, current_start_y = start_x, start_y
for goal_x, goal_y in goals:
    wp_list = execute_astar(current_start_x, current_start_y, goal_x, goal_y)
    final_path.extend(wp_list)
    current_start_x, current_start_y = goal_x, goal_y

def calculate_total_path_cost(final_path):
    total_cost = 0
    for i in range(len(final_path) - 1):
        total_cost += distance(final_path[i], final_path[i + 1])
    return total_cost

# Example usage
  # Replace with the actual final_path
total_cost = calculate_total_path_cost(final_path)




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
path_x = [point[0] for point in final_path]
path_y = [point[1] for point in final_path]
plt.plot(path_x, path_y, color='blue', linewidth=2, label='Path')

for goal in goals:
    plt.scatter(goal[0], goal[1], color='red', marker='x', label='Goal')
# Plot start and goal
plt.scatter(start_x, start_y, color='green', marker='o', label='Start')

plt.title("Super TSP")

plt.xlim(min_x - 0.5, max_x + 0.5)
plt.ylim(min_y - 0.5, max_y + 0.5)
plt.gca().set_aspect('equal', adjustable='box')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.grid(True)
plt.show()

print("The list of waypoints for the desired path for the A* algorithm is :", final_path)
print ("the cost of the final path way is :",total_cost)
