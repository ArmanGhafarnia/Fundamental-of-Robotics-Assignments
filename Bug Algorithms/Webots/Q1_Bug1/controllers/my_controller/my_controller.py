"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from math import pi as PI, atan2, dist

START_K_P, INCREMENT_K_P = 1, 0.01
K_P, K_A, K_B, K_F = START_K_P, 60, -10, 150
R, L = 0.0205, 0.026
DIST_THRESHOLD = 0.035
TURN_VELOCITY = 6
FOLLOW_BOUNDARY_VELOCITY = 4
ADJ_VELOCITY = 0.5
HIT_POINT = (100, 100) 

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
chase_goal = False
theta_at_start_of_turning = 0
left_follow_dist = 0
cnt = 0
change_hit_point_cnt = 0

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
    global left_follow_dist, cnt, turning_in_the_end, HIT_POINT, chase_goal
    global change_hit_point_cnt
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
        
        vl, vr = [FOLLOW_BOUNDARY_VELOCITY]*2
        
        vl -= K_F * (DIST_THRESHOLD - right)
        vr += K_F * (DIST_THRESHOLD - right)
        
        
        x, y = gps.getValues()[:2]
        
        if dist((x, y), HIT_POINT) < 10**(-2) and (cnt - change_hit_point_cnt > 1000):
            follow_boundary = False
            chase_goal = True
        
        if dist((x, y), (gx, gy)) < dist(HIT_POINT, (gx, gy)):
            HIT_POINT = (x, y)
            change_hit_point_cnt = cnt
        
        
    elif chase_goal:
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
    

    
    
    

