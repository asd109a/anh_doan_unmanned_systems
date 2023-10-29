# -*- coding: utf-8 -*-
"""
Created on Sat Oct 28 22:59:56 2023

@author: DCM82X
"""
from itertools import permutations
import math

def distance(curr_point, next_point):
    return math.sqrt((next_point[0] - curr_point[0])**2 + (next_point[1] - curr_point[1])**2)

start_location = (0, 0)
points = [(2, 2), (5, 3), (3, 4), (6, 4)]

min_cost = float('inf')
min_path = None

for perm in permutations(points):
    total_cost = 0
    current_location = start_location
    for point in perm:
        total_cost += distance(current_location, point)
        current_location = point
    if total_cost < min_cost:
        min_cost = total_cost
        min_path = perm
    print(f"Path: {perm} Cost: {total_cost}")

print(f"Minimum cost path: {min_path} Minimum cost: {min_cost}")

