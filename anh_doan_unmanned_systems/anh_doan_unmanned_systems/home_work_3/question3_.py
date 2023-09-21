import matplotlib.pyplot as plt
import numpy as np
import random


class RRTNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:
    def __init__(self, start, goal, obstacles, step_size=0.5, max_iterations=1000):
        self.start = RRTNode(start[0], start[1])
        self.goal = RRTNode(goal[0], goal[1])
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.nodes = [self.start]

    def is_collision_free(self, x, y):
        for obstacle in self.obstacles:
            if np.linalg.norm(np.array([x - obstacle[0], y - obstacle[1]])) <= obstacle[2]:
                return False
        return True

    def find_nearest_node(self, x, y):
        min_distance = float('inf')
        nearest_node = None
        for node in self.nodes:
            distance = np.linalg.norm(np.array([x - node.x, y - node.y]))
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        return nearest_node

    def generate_random_point(self):
        if random.random() < 0.1:
            return self.goal.x, self.goal.y
        else:
            # Adjust the bounds according to your environment
            x = random.uniform(0, 10)
            # Adjust the bounds according to your environment
            y = random.uniform(0, 10)
            return x, y

    def extend_tree(self):
        random_x, random_y = self.generate_random_point()
        nearest_node = self.find_nearest_node(random_x, random_y)

        delta_x = random_x - nearest_node.x
        delta_y = random_y - nearest_node.y
        distance = np.linalg.norm(np.array([delta_x, delta_y]))

        if distance <= self.step_size:
            new_node = RRTNode(random_x, random_y)
        else:
            scale = self.step_size / distance
            new_x = nearest_node.x + delta_x * scale
            new_y = nearest_node.y + delta_y * scale
            new_node = RRTNode(new_x, new_y)

        if self.is_collision_free(new_node.x, new_node.y):
            new_node.parent = nearest_node
            self.nodes.append(new_node)
            return new_node

    def plan(self):
        for _ in range(self.max_iterations):
            new_node = self.extend_tree()
            if new_node:
                if np.linalg.norm(np.array([new_node.x - self.goal.x, new_node.y - self.goal.y])) <= self.step_size:
                    self.goal.parent = new_node
                    self.nodes.append(self.goal)
                    return self.construct_path()
        return None

    def construct_path(self):
        path = []
        current_node = self.goal
        while current_node:
            path.append([current_node.x, current_node.y])
            current_node = current_node.parent
        return path[::-1]


def plot_obstacles(obstacles):
    for obstacle in obstacles:
        circle = plt.Circle(
            (obstacle[0], obstacle[1]), obstacle[2], color='red')
        plt.gca().add_patch(circle)


def calculate_path_cost(path):
    cost = 0.0
    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        cost += np.linalg.norm(np.array([x2 - x1, y2 - y1]))
    return cost


def main():
    start = (1, 1)
    goal = (9, 8)
    obstacles = [(2, 2, 0.25), (2, 3, 0.25), (2, 4, 0.25), (5, 5, 0.25), (6, 6, 0.25), (7, 6, 0.25), (7, 5, 0.25),
                 (7, 4, 0.25), (7, 3, 0.25), (8, 6, 0.25)]  # (x, y, radius)

    rrt = RRT(start, goal, obstacles)
    path = rrt.plan()

    if path:
        plt.figure(figsize=(8, 8))
        plot_obstacles(obstacles)
        plt.plot(start[0], start[1], 'go', label='Start')
        plt.plot(goal[0], goal[1], 'ro', label='Goal')

        # Plot the tree (valid nodes)
        for node in rrt.nodes:
            if node.parent:
                plt.plot([node.x, node.parent.x], [
                         node.y, node.parent.y], 'y-', linewidth=0.5)

        path_points = np.array(path)
        plt.plot(path_points[:, 0], path_points[:, 1], 'b-', label='Path')
        plt.xlim(0, 10)
        plt.ylim(0, 10)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.legend()
        plt.grid()
        plt.show()

        # Calculate and print the cost of the shortest path
        shortest_path_cost = calculate_path_cost(path)
        print(f"Shortest Path Cost: {shortest_path_cost:.2f}")
    else:
        print("No path found!")


if __name__ == "__main__":
    main()
