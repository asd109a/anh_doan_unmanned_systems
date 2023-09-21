import numpy as np
import matplotlib.pyplot as plt

class GridMap:
    def __init__(self, gs, min_x, max_x, min_y, max_y):
        self.gs = gs
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.nodes = {}
        self.calculate_nodes()

    def calculate_nodes(self):
        node_index = 0
        for y in np.arange(self.min_y, self.max_y + self.gs, self.gs):
            for x in np.arange(self.min_x, self.max_x + self.gs, self.gs):
                self.nodes[(x, y)] = node_index
                node_index += 1

    def get_node_index(self, x, y):
        return self.nodes.get((x, y), None)

    def plot_grid_with_indices(self):
        plt.figure(figsize=(10, 10))
        for y in np.arange(self.min_y, self.max_y +1, self.gs):
            for x in np.arange(self.min_x, self.max_x +1, self.gs):
                index = self.get_node_index(x, y)
                if index is not None:
                    plt.text(x, y, str(index), color='red', fontsize=8)
                plt.plot(x, y)

        plt.xlim(self.min_x, self.max_x + 0.5)
        plt.ylim(self.min_y, self.max_y + 0.5)
        plt.show()

gs = 0.5
min_x = 0
max_x = 10
min_y = 0
max_y = 10

grid_map = GridMap(gs, min_x, max_x, min_y, max_y)

grid_map.plot_grid_with_indices()