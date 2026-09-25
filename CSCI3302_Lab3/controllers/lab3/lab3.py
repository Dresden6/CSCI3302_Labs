"""csci3302_lab3 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np

pose_x = 0
pose_y = 0
pose_theta = 0

# create the Robot instance.
robot = Supervisor()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_WHEEL_RADIUS = 0.0205
EPUCK_MAX_WHEEL_SPEED = 0.1257 # ePuck wheel speed in m/s
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = 0
vR = 0

# Initialize gps and compass for odometry
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

# TODO: Find waypoints to navigate around the arena while avoiding obstacles
# Use shift+drag on the ping pong marker in the simulator to find good waypoints.
# Add them as (x, y) tuples. You need at least one waypoint before running!
waypoints = [(-0.155,-0.195), (-0.26,-0.257), (-0.243, -0.37), (-0.14, -0.38), (-0.0295, -0.386), (-0.07, -0.393), (0.187, -0.392), (0.284, -0.385), (0.285, -0.281), (0.15, -0.198), (0.04, -0.113), (0.0301, 0.0381), (0.125, 0.122), (0.232, 0.188), (0.31, 0.248), (0.241, 0.322), (0.127, 0.389), (-0.0392, 0.396), (-0.185, 0.393), (-0.273, 0.381), (-0.277, 0.302), (-0.233, 0.215), (-0.233, 0.0729), (-0.272, 0.016),(-0.264, -0.13)] # e.g. [(-0.1, -0.4), (0.3, -0.4), ...]
# Index indicating which waypoint the robot is reaching next
index = 0

# Get ping pong ball marker that marks the next waypoint the robot is reaching
marker = robot.getFromDef("marker").getField("translation")

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:
    # Safety check: make sure waypoints are defined
    if len(waypoints) == 0:
        print("ERROR: No waypoints defined! Please add waypoints to the waypoints list.")
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        continue

    # Set the position of the marker
    marker.setSFVec3f([waypoints[index][0], waypoints[index][1], 0.01])
    
    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # Read pose_x, pose_y, pose_theta from gps and compass
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]
    pose_theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])
    
    # TODO: controller
    
    
    # T_inverse maps world coordinates to robot coordinates
    T_inverse = np.array([
    [np.cos(pose_theta), np.sin(pose_theta), 0], 
    [-np.sin(pose_theta), np.cos(pose_theta), 0], 
    [0, 0, 1]
    ])
    
    # Goal (x, y, theta) In robot coordinate space:
    target = T_inverse @ np.array([[waypoints[index][0] - pose_x], [waypoints[index][1] - pose_y], [pose_theta]])
    
    vL = (target[0] - (target[2] * EPUCK_AXLE_DIAMETER)/2) / EPUCK_AXLE_DIAMETER
       
    
    print(target)
    
    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
