# -*- coding: utf-8 -*-
"""
Created on Wed Aug 30 19:00:48 2023

@author: DCM82X
"""

class Node:
    def __init__(self, x, y, parent_cost, index):
        self.x = x
        self.y = y
        self.parent_cost = parent_cost
        self.index = index

node1 = Node(1, 10, 10,21)
node2 = Node(2,4, 7,8)

print("Node 1:")
print("x:", node1.x)
print("y:", node1.y)
print("parent_cost:", node1.parent_cost)
print("index:", node1.index)

print("Node 2:")
print("x:", node2.x)
print("y:", node2.y)
print("parent_cost:", node2.parent_cost)
print("index:", node2.index)

