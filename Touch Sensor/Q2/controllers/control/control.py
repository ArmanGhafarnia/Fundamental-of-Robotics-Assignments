"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from matplotlib import pyplot as plt

SAMPLING_PERIOD = 1



# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(4)
right_motor.setVelocity(4)


touch_sensor = robot.getDevice('touch sensor')
touch_sensor.enable(SAMPLING_PERIOD)

x_fs, t = [], []

while robot.step(timestep) != -1:

    #print(touch_sensor.getLookupTable())
    force_3d = touch_sensor.getValues()
    x, y, z = force_3d[0], force_3d[1], force_3d[2]
    
    print(x, y, z)   
    
    x_fs.append(x)
    t.append(robot.getTime())

    if robot.getTime()>5:
        break 
     
max_force = max(x_fs)

plt.axhline(max_force, linestyle='-.', color='deeppink', label = f'max force = {max_force: .3f}N')  
plt.plot(t, x_fs, color='purple', label = 'force vs time')
plt.legend(loc='best')
plt.xlabel('Time(s)')
plt.ylabel('Force(N)')
plt.show()


