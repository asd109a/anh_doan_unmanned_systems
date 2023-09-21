'''
question 1
'''
import math as m

def Euclidean_distance(node_1, node_2):
    x1, y1 = node_1
    x2, y2 = node_2
    distance = m.dist(node_1,node_2)
    return distance


node_1 = (2, 1)
node_2 = (3, 2)

distance = Euclidean_distance(node_1, node_2)
print(f"The Euclidean distance between {node_1} and {node_2} is {distance:.2f}")

