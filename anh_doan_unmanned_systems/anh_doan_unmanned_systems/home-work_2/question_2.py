

def is_valid_node(node, obstacles, obs_radius, x_min, x_max, y_min, y_max, step_size):
    x, y = node
    
    for obstacle in obstacles:
        obstacle_x, obstacle_y = obstacle
        # Calculate the distance between the node and the obstacle
        distance = ((x - obstacle_x) ** 2 + (y - obstacle_y) ** 2) ** 0.5
        if distance <= obs_radius:  # means node is not valid
            return False
        # check if the node is in the boundaries
    if x < x_min:
        return False
    if x > x_max:
        return False
    if y < y_min:
        return False
    if y > y_max:
        return False
    
    return True

# Define the obstacle list, grid boundaries, and step size
obstacles = [(1, 1), (4, 4), (3, 4), (5, 0), (5, 1), (0, 7), (1, 7), (2, 7), (3, 7)]

x_min= 0
x_max = 10
y_min = 0
y_max = 10
step_size = 0.5
obs_radius = 0.5/2

# Test the function with the location (2, 2)
node = (2, 2)
is_valid = is_valid_node(node, obstacles,obs_radius, x_min, x_max, y_min, y_max, step_size)

if is_valid:
    print(f"The location {node} is valid.")
else:
    print(f"The location {node} is not valid.")