from controller import Robot
import math
from matplotlib import pyplot as plt
import numpy as np

# Get robot's heading in degree based on compass values
def get_robot_heading(compass_value):
    rad = math.atan2(compass_value[1], compass_value[0])
    bearing = (rad  - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0

    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading


robot = Robot()

VELOCITY = 5
timestep = int(robot.getBasicTimeStep())


left_motor = robot.getDevice('left wheel motor')

left_motor.setPosition(float('inf'))
left_motor.setVelocity(VELOCITY)

compass = robot.getDevice('compass')
dist_sensor = robot.getDevice('distance sensor')
dist_sensor.enable(1)
compass.enable(1)



first, flag = True, False
first_teta = 0

dists = []

while robot.step(timestep) != -1:
    
    if first:
        first = False
        first_teta = get_robot_heading(compass.getValues())
    

    teta = get_robot_heading(compass.getValues())
    dist = dist_sensor.getValue()
    

    
    if teta < first_teta:
        flag = True
        
    # Now We've Completed one 360 degree turn:
    if flag  and teta > first_teta:
        print('--\tEnd of Sampling--')
        print(f'--\t{len(dists)} Samples Gathered!--')
        break
    

    dists.append((dist, teta))
    print(dist, teta)



xs, ys = [], []
for distance, teta in dists:
    xs.append(math.cos(teta*math.pi/180)*distance)
    ys.append(math.sin(teta*math.pi/180)*distance)


def get_line(p1, p2):
    slope = (p1[1] - p2[1])/(p1[0] - p2[0])
    bias = p1[1] - slope*p1[0]
    return slope, bias


def distance_to_line(slope, bias, p):
    return abs(slope*p[0] - p[1] + bias) / math.sqrt(slope**2 + 1)

LIST_OF_LINES = []
def split(points, thresh):

    slope, bias = get_line(points[0], points[-1])

    dists = [distance_to_line(slope, bias, point) for point in points]

    max_index = np.argmax(dists)

    if dists[max_index] > thresh:
        split(points[:max_index+1], thresh)
        split(points[max_index:], thresh)
        
    else:
        LIST_OF_LINES.append((points[0], points[1]))
    


POINTS = [(xs[i], ys[i]) for i in range(len(xs))]
THRESHOLD = 10**(-2)

split(POINTS, thresh = THRESHOLD)


plt.subplot(122)
plt.scatter([0], [0], c='green', s=100, marker='$R$')
plt.title(f'Split and Merge with threshhold = {THRESHOLD}')
plt.xlabel('X(m)')
plt.ylabel('Y(m)')


# merge: 
for i in range(len(LIST_OF_LINES)):
    
    p0 = LIST_OF_LINES[i][0]
    p1 = LIST_OF_LINES[i][1]
    p2 = LIST_OF_LINES[(i+1)%len(LIST_OF_LINES)][0]
    
    plt.plot([p0[0], p1[0]], [p0[1], p1[1]], c='black')
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], c='black')


plt.subplot(121)
plt.title(f'{len(dists)} Sampled Data\nVelocity = {VELOCITY} rad/s')
plt.scatter([0], [0], c='green', s=100, marker='$R$')
plt.scatter(xs, ys, c=range(len(xs)), cmap='plasma')
plt.colorbar(location='left', label = 'Order Of Sampled Data')
plt.xlabel('X(m)')
plt.ylabel('Y(m)')
plt.show()    
   