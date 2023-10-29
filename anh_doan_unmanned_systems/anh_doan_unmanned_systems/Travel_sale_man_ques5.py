
import matplotlib.pyplot as plt
from itertools import permutations
import math

def distance(point1, point2):
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

start_location = (0, 0)
points = [(2, 2), (5, 3), (3, 4), (6, 4)]

min_cost = float('inf')
min_path = None
costs = []

for perm in permutations(points):
    total_cost = 0
    current_location = start_location
    x = [start_location[0]]
    y = [start_location[1]]
    for point in perm:
        total_cost += distance(current_location, point)
        x.append(point[0])
        y.append(point[1])
        current_location = point
    costs.append((perm, total_cost))
    if total_cost < min_cost:
        min_cost = total_cost
        min_path = perm
    plt.plot(x, y, color='grey', alpha=0.3)

# Print all costs
for path, cost in costs:
    print(f"Path: {path} Cost: {cost}")

# Print minimum cost
print(f"Minimum cost path: {min_path} Minimum cost: {min_cost}")

# Highlight the minimum cost path
x, y = zip(*[start_location] + list(min_path))
plt.plot(x, y, color='red', linewidth=2, label='Minimum Cost Path')

# Annotate the starting point and waypoints
plt.annotate('Start', (x[0], y[0]), textcoords="offset points", xytext=(0, 10), ha='center')
for i, (xi, yi) in enumerate(min_path):
    plt.annotate(f'Point {i + 1}', (xi, yi), textcoords="offset points", xytext=(0, 10), ha='center')

plt.legend()
plt.show()
