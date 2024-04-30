"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from math import pi as PI, atan2, dist, sqrt, sin, cos

START_K_P, INCREMENT_K_P = 1, 0.01
K_P, K_A, K_B, K_F = START_K_P, 60, -10, 200
R, L = 0.0205, 0.026
DIST_THRESHOLD = 0.033
TURN_VELOCITY = 6
FOLLOW_BOUNDARY_VELOCITY = 6
ADJ_VELOCITY = 0.5
DONT_CHANGE_K_P = False

# goal coordinates:
gx, gy = 0, -0.5
sx, sy = 0.08, 0.3

def get_line(p1, p2):
    slope = (p1[1] - p2[1])/(p1[0] - p2[0])
    bias = p1[1] - slope*p1[0]
    return slope, bias

STRAIGHT_LINE_SLOPE, STRAIGHT_LINE_BIAS = get_line((sx, sy), (gx, gy))

def distance_to_line(slope, bias, p):
    return abs(slope*p[0] - p[1] + bias) / sqrt(slope**2 + 1)

def is_on_line(p, acc=10**(-3)):
    return distance_to_line(STRAIGHT_LINE_SLOPE, STRAIGHT_LINE_BIAS, p) <= acc


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
    if not (-6.28 <= vl <= 6.28) or not (-6.28 <= vr <= 6.28):
        coeff = (6.279)/ max(abs(vl), abs(vr))   
        vl *= coeff
        vr *= coeff
    return vl, vr

def get_velocities() -> list[float, float]:
    p, a, b = get_polar_error()
    global K_P
    v: float = K_P*p
    w: float = K_A*a + K_B*b
    
    
    if K_P < 6-INCREMENT_K_P and not DONT_CHANGE_K_P:
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
go_in_line = False
theta_at_start_of_turning = 0
left_follow_dist = 0
cnt = 0
TURN_POINTS = [(100, 100)]


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
    global left_follow_dist, cnt, turning_in_the_end, go_in_line
    global K_P, DONT_CHANGE_K_P, TURN_POINTS
    obstacle = where_is_obstacle()

    cnt += 1

    if turning_to_follow:
        vl, vr = turn_left()
        theta = get_robot_heading(compass.getValues())
        if abs(theta - theta_at_start_of_turning) > 88:
            turning_to_follow = False
            follow_boundary = True
            
    elif follow_boundary:
        front, right, _, _, _ = get_distances()
        
        if front <= DIST_THRESHOLD:
            turning_to_follow = True
            theta_at_start_of_turning = get_robot_heading(compass.getValues())
            follow_boundary = False
        
        if is_on_line(gps.getValues()[:2]):
            
            x, y = gps.getValues()[:2]
            been_here_before = False
            for TURN_POINT in TURN_POINTS:
                if dist((x, y), TURN_POINT) < 10**(-1):
                    # print('BEEN HERE BEFORE!')
                    been_here_before = True
                 
            
            if not been_here_before and obstacle != 'LEFT':
                go_in_line = True
                # print('FOLLOWING STOPPED!', cnt)
                K_P = START_K_P
                DONT_CHANGE_K_P = True
                follow_boundary = False
                if dist((gx, gy), (x, y)) < dist((gx, gy), (TURN_POINTS[-1])):
                    TURN_POINTS.remove(TURN_POINTS[-1])
                TURN_POINTS.append((x, y))
        
        vl, vr = [FOLLOW_BOUNDARY_VELOCITY]*2
        vl -= K_F * (DIST_THRESHOLD - right)
        vr += K_F * (DIST_THRESHOLD - right)
    
     
    elif go_in_line:
        if obstacle == 'FRONT':
            go_in_line = False
        return get_velocities()
                
        
        
    else:
        
        if obstacle is None:
            return get_velocities()     
            
        elif obstacle == 'FRONT':
            vl, vr = handle_front_obstacle()
        
        elif obstacle == 'FRONT_LEFT':
            vl, vr = turn_right()
            
        elif obstacle == 'FRONT_RIGHT':
            vl, vr = turn_left() 
        
        elif obstacle == 'LEFT':
            follow_boundary = True
            vl, vr = turn_right()
            
        elif obstacle == 'RIGHT':
            follow_boundary = True
            vl, vr = turn_left()
        
        else:
            return get_velocities()
     
    return normalize(vl, vr)
         
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    vl, vr = avoid_obstacle_velocities()
    
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)

    

