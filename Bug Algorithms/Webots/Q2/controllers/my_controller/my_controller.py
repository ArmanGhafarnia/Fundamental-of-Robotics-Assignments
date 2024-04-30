"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from math import pi as PI, atan2, dist, sin, cos, sqrt
from matplotlib import pyplot as plt
import numpy as np

START_K_P, INCREMENT_K_P = 1, 0.01
K_P, K_A, K_B, K_F = START_K_P, 60, -10, 300
R, L = 0.0205, 0.026
DIST_THRESHOLD = 0.035
FOLLOW_DIST = 0.025
TURN_VELOCITY = 6
FOLLOW_BOUNDARY_VELOCITY = 4
ADJ_VELOCITY = 0.5

# goal coordinates:
gx, gy = 0, -0.5

def get_robot_heading(compass_value):
    rad = atan2(compass_value[1], compass_value[0])
    bearing = (rad - PI/2) / PI * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0

    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading
    
def go_in_upper_circle(angle: float) -> float:
    if angle > PI:
        angle -= 2 * PI
    elif angle < -PI:
        angle += 2 * PI
    return angle

def get_polar_error() -> list[float, float, float]:
    x, y, _ =   gps.getValues()
    teta = get_robot_heading(compass.getValues())*PI/180

    p = dist((gx, gy), (x, y))
    a = -teta + atan2(gy - y, gx - x)
    b = -teta-a

    b, a = go_in_upper_circle(b), go_in_upper_circle(a)
    return p, a, b
    
def normalize(vl, vr):
    if not (-2*PI<=vl<=2*PI) or not (-2*PI<=vr<=2*PI):
        coeff = (2*PI-0.1)/ max(abs(vl), abs(vr))   
        vl *= coeff
        vr *= coeff
    return vl, vr

def get_velocities() -> list[float, float]:
    p, a, b = get_polar_error()
    global K_P
    v: float = K_P*p
    w: float = K_A*a + K_B*b
    
    if K_P < 6-INCREMENT_K_P:
        K_P += INCREMENT_K_P

    v_left = (v - L*w/2)
    v_right = (v + L*w/2)
    
    return normalize(v_left, v_right)



# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
sampling_period = 1  # in ms


left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
f = robot.getDevice('front distance sensor')
fr = robot.getDevice('front right distance sensor')
fl = robot.getDevice('front left distance sensor')
r = robot.getDevice('right distance sensor')
l = robot.getDevice('left distance sensor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Enables the devices
gps.enable(sampling_period)
compass.enable(sampling_period)
f.enable(sampling_period)
r.enable(sampling_period)
l.enable(sampling_period)
fr.enable(sampling_period)
fl.enable(sampling_period)


def get_distances():
    front = f.getValue()
    left = l.getValue()
    right = r.getValue()
    front_right = fr.getValue()
    front_left = fl.getValue()
    
    return front, right, left, front_right, front_left
    
def where_is_obstacle():
    strings = ['FRONT', 'RIGHT', 'LEFT', 'FRONT_RIGHT', 'FRONT_LEFT']
    dists = get_distances()
    for i in range(len(dists)):
            if dists[i] <= DIST_THRESHOLD:
                return strings[i]
    return None


def turn_right():
    return TURN_VELOCITY, -TURN_VELOCITY

def turn_left():
    return -TURN_VELOCITY, TURN_VELOCITY
   

follow_boundary = False
turning_to_follow = False
turning_in_the_end = False
theta_at_start_of_turning = 0
left_follow_dist = 0
cnt = 1
first = True
first_turn_cors = (100, 100)
first_turn_cnt = 0

def handle_front_obstacle():
    global turning_to_follow, follow_boundary, theta_at_start_of_turning
    global left_follow_dist
    vl, vr = turn_left()
    turning_to_follow = True
    theta_at_start_of_turning = get_robot_heading(compass.getValues())
    K_P = START_K_P
    return vl, vr


def avoid_obstacle_velocities():
    global turning_to_follow, follow_boundary, theta_at_start_of_turning
    global left_follow_dist, cnt, turning_in_the_end, first, first_turn_cors
    global first_turn_cnt
    obstacle = where_is_obstacle()

    if turning_to_follow:
        if first:
            first_turn_cors = gps.getValues()[:2]
            first = False
            first_turn_cnt = cnt
            
        print(f'TURNING {cnt}')
        vl, vr = turn_left()
        theta = get_robot_heading(compass.getValues())
        if abs(theta - theta_at_start_of_turning) > 88:
            turning_to_follow = False
            print(f'END OF TURN {cnt}')
            follow_boundary = True
            
    elif follow_boundary:
        print(f'FOLLOWING_BOUNDARY {cnt}')
        front, right, _, _, _ = get_distances()
        
        if front <= DIST_THRESHOLD:
            turning_to_follow = True
            theta_at_start_of_turning = get_robot_heading(compass.getValues())
            follow_boundary = False
        
        vl, vr = [FOLLOW_BOUNDARY_VELOCITY]*2
        
        vl -= K_F * (FOLLOW_DIST - right)
        vr += K_F * (FOLLOW_DIST - right)
        
        
    else:
        
        if obstacle is None:
            print(f'No OBSTACLES {cnt}')
            return get_velocities()     
            
        elif obstacle == 'FRONT':
            print('FRONT')
            vl, vr = handle_front_obstacle()
        
        elif obstacle == 'FRONT_LEFT':
            print('FRONT_LEFT')
            vl, vr = turn_right()
            
        elif obstacle == 'FRONT_RIGHT':
            print('FRONT_RIGHT')
            vl, vr = turn_left() 
        
        elif obstacle == 'LEFT':
            print('LEFT')
            follow_boundary = True
            vl, vr = turn_right()
            
        elif obstacle == 'RIGHT':
            follow_boundary = True
            vl, vr = turn_left()
        
        else:
            print(obstacle)
            return get_velocities()
     
    return normalize(vl, vr)
   
   
OBSTACLE_POINTS = [] 
def gather_data():
    front, right, left, _, _ = get_distances()
    x, y, _ = gps.getValues()
    theta = get_robot_heading(compass.getValues())
    theta *= PI/180
     
    if right < DIST_THRESHOLD*2:
        obstacle_point = (right*cos(theta-PI/2) + x, right*sin(theta-PI/2) + y) 
        OBSTACLE_POINTS.append(obstacle_point)
               

def get_line(p1, p2):
    slope = (p1[1] - p2[1])/(p1[0] - p2[0])
    bias = p1[1] - slope*p1[0]
    return slope, bias


def distance_to_line(slope, bias, p):
    return abs(slope*p[0] - p[1] + bias) / sqrt(slope**2 + 1)  

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
    


THRESHOLD = 10**(-1.6)
def split_merge():
    split(OBSTACLE_POINTS, thresh = THRESHOLD)
    
    plt.title(f'Split and Merge with threshold = {THRESHOLD}')
    plt.xlabel('X(m)')
    plt.ylabel('Y(m)')
    
    
    # merge: 
    for i in range(len(LIST_OF_LINES)):
        
        p0 = LIST_OF_LINES[i][0]
        p1 = LIST_OF_LINES[i][1]

        p2 = LIST_OF_LINES[(i+1)%len(LIST_OF_LINES)][0]

        plt.plot([p0[0], p1[0]], [p0[1], p1[1]], c='black')
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], c='black')
    
    plt.show()



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    vl, vr = avoid_obstacle_velocities()
    
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)
    

    gather_data()
    
    
    cnt += 1
    

    if cnt - first_turn_cnt > 1000 and dist(gps.getValues()[:2], first_turn_cors) < 10**(-1):
        split_merge()
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break


np.savetxt('point.txt', OBSTACLE_POINTS, fmt = '%.7f')

# OBSTACLE_POINTS = np.loadtxt('point.txt')

plt.subplot(121)
plt.scatter([i[0] for i in OBSTACLE_POINTS], [i[1] for i in OBSTACLE_POINTS], c = range(len(OBSTACLE_POINTS)), cmap = 'winter')
plt.colorbar(location='left', label = 'Order Of Sampled Data')
plt.xlabel('X(m)')
plt.ylabel('Y(m)')

THRESHOLD = 10**(-1.65)
plt.subplot(122)
split_merge()


    

