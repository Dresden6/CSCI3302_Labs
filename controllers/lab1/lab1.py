from controller import Robot, DistanceSensor, Motor
from enum import Enum

# time in [ms] of a simulation step
TIME_STEP = 64
MAX_SPEED = 6.28

WALL_FOLLOW_DIST = 85

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

ls = []
lsNames = ['ls0', 'ls1', 'ls2', 'ls3', 'ls4', 'ls5', 'ls6', 'ls7']

for i in range(len(lsNames)):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

vL = 1
vR = 1


class State(Enum):
    FOLLOW_L = 1
    TURN_R = 2
    TURN_180 = 3
    FOLLOW_R = 4
    STOP = 5

curr_state = State.FOLLOW_L


# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    lsValues = []
    for i in range(8):
        lsValues.append(ls[i].getValue())

    match curr_state.name:
        case "FOLLOW_L":
            coeff = (WALL_FOLLOW_DIST - psValues[5]) * 0.0005
            vL = 1 - coeff
            vR = 1 + coeff
            
            if(psValues[7] > 80 and psValues[0] > 80): 
                print("TURN_R now")
                curr_state = State.TURN_R
            
        case "TURN_R":
            vL = 1
            vR = -1
            
            if(psValues[4] > 80 and psValues[3] > 80): 
                print("FOLLOW_L now")
                curr_state = State.FOLLOW_L
            
            pass
            
        case "TURN_180":
            pass
            
        case "FOLLOW_R":
            pass
            
        case "STOP":
            pass
            
        case _:
            pass
            
            
    if(vL > MAX_SPEED): vL = MAX_SPEED
    if(vL < -MAX_SPEED): vL = MAX_SPEED
    if(vR > MAX_SPEED): vR = MAX_SPEED
    if(vR < -MAX_SPEED): vR = MAX_SPEED
    
    # write actuators inputs
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
   
   
   
   
   
   