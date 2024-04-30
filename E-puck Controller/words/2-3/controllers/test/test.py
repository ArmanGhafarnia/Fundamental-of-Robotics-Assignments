"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
import matplotlib.pyplot as plt
import numpy as np
import math

# Get robot's heading in degree based on compass values
def get_robot_heading(compass_value):
    rad = math.atan2(compass_value[0], compass_value[1])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0
    
    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading

# create the Robot instance.
robot = Robot()




# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)


left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Set the motors to rotate for ever
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
# But with no velocity :)
left_motor.setVelocity(0)
right_motor.setVelocity(0)


sampling_period = 1 # in ms
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
# Enables the devices
gps.enable(sampling_period)
compass.enable(sampling_period)


robot.step(1000) # take some dummy steps in environment for safe initialization of the initial heading and position
initial_gps_value = gps.getValues()
initial_compass_value = compass.getValues()




cesi_R = [[],[],[],[]] 


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    robot.step()
    
    left_motor.setVelocity(-math.cos(robot.getTime()))
    right_motor.setVelocity(math.sin(robot.getTime()))
    
    cesi_R[0].append(gps.getValues()[0])
    cesi_R[1].append(gps.getValues()[1])
    cesi_R[2].append(get_robot_heading(compass.getValues()))
    cesi_R[3].append(robot.getTime())
    
    if robot.getTime() > 20:
        break

        
plt.plot(cesi_R[0], cesi_R[1])
plt.xlabel('X(m)')
plt.ylabel('Y(m)')
plt.title('Coordinates of Robot')
plt.show()

plt.plot(cesi_R[3], cesi_R[2])
plt.ylim(0, 360)
plt.xlabel('Time(s)')
plt.ylabel('θ(deg)')
plt.title('θ-Time of Robot')
plt.show() 