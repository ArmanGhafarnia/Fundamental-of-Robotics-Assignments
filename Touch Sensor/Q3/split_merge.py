import math
import numpy as np
from matplotlib import pyplot as plt

POINTS = []

with open('data.txt') as file:
    x = file.read()[2:-2].split('), (')
    for i in x:
        POINTS.append([float(y) for y in i.split(',')])


thresh = 0.0001


def get_line(p1, p2):
    slope = (p1[1] - p2[1])/(p1[0] - p2[0])
    bias = p1[1] - slope*p1[0]
    return slope, bias


def distance_to_line(slope, bias, p):
    return abs(slope*p[0] - p[1] + bias) / math.sqrt(slope**2 + 1)

LIST_OF_LINES = []
def split_merge(points):

    slope, bias = get_line(points[0], points[-1])

    dists = [distance_to_line(slope, bias, point) for point in points]

    max_index = np.argmax(dists)

    if dists[max_index] > thresh:

        split_merge(points[:max_index+1])
        split_merge(points[max_index:])
        
        pass
        
    else:
        LIST_OF_LINES.append((points[0], points[1]))




split_merge(POINTS)


for p1, p2 in LIST_OF_LINES:
    
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]])

for i in range(len(LIST_OF_LINES)):
    
    p1 = LIST_OF_LINES[i][1]
    p2 = LIST_OF_LINES[(i+1)%len(LIST_OF_LINES)][0]
    
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]])
    

    
    
    
plt.show()    
    