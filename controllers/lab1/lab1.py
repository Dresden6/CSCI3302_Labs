from controller import Robot, DistanceSensor, Motor
from enum import Enum

# time in [ms] of a simulation step
TIME_STEP = 64
MAX_SPEED = 6.28
TRAVEL_SPEED = 2 # Don't go faster, it breaks other things

# light detection threshold
THRESHOLD=750

# Wall Follow PID
FOLLOW_P = 0.0065
FOLLOW_D = 0.03


WALL_FOLLOW_DIST = 130

TURN_180_STEPS=67
turn_counter=0

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

vL = TRAVEL_SPEED
vR = TRAVEL_SPEED


class State(Enum):
    FOLLOW_L = 1
    TURN_R = 2
    TURN_180 = 3
    FOLLOW_R = 4
    TURN_L = 5
    STOP = 6

curr_state = State.FOLLOW_L

prev_L_err = 0
prev_R_err = 0

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    lsValues = []
    for i in range(8):
        lsValues.append(ls[i].getValue())
        
    # print(psValues[5])

    match curr_state.name:
        case "FOLLOW_L":
            err = WALL_FOLLOW_DIST - psValues[5]
            coeff = (err * FOLLOW_P) + ((err - prev_L_err) * FOLLOW_D)
            vL = TRAVEL_SPEED - coeff
            vR = TRAVEL_SPEED + coeff
            
            prev_L_err = err
            
            if(psValues[7] > 80 and psValues[0] > 80): 
                print("TURN_R now")
                curr_state = State.TURN_R
            
            if (lsValues[0] < THRESHOLD and lsValues[1] < THRESHOLD and lsValues[2] < THRESHOLD and
                lsValues[5] < THRESHOLD and lsValues[6] < THRESHOLD and lsValues[7] < THRESHOLD):
                print("TURN_180 now")
                curr_state = State.TURN_180
            
        case "TURN_R":
            vL = 1
            vR = -1
            
            if(psValues[4] > 80 and psValues[3] > 80): 
                print("FOLLOW_L now")
                curr_state = State.FOLLOW_L
            
            pass
            
        case "TURN_180":
            vL = 1
            vR = -1
            turn_counter += 1

            if turn_counter >= TURN_180_STEPS:
                turn_counter = 0
                print("FOLLOW_R now")
                curr_state = State.FOLLOW_R
            
            pass
            
        case "FOLLOW_R":
            err = WALL_FOLLOW_DIST - psValues[2]
            coeff = (err * FOLLOW_P) + ((err - prev_R_err) * FOLLOW_D)
            vL = TRAVEL_SPEED + coeff
            vR = TRAVEL_SPEED - coeff
            
            prev_R_err = err
            
            if(psValues[7] > 80 and psValues[0] > 80): 
                print("TURN_L now")
                curr_state = State.TURN_L
                
                
            if (lsValues[0] < THRESHOLD and lsValues[1] < THRESHOLD and lsValues[2] < THRESHOLD and
                lsValues[5] < THRESHOLD and lsValues[6] < THRESHOLD and lsValues[7] < THRESHOLD):
                print("STOP now")
                curr_state = State.STOP
                
                
        case "TURN_L": # Necessary for wall-following on the right
            vL = -1
            vR = 1
            
            if(psValues[4] > 80 and psValues[3] > 80): 
                print("FOLLOW_L now")
                curr_state = State.FOLLOW_R
            
            pass
            
        case "STOP":
            vR=0
            vL=0
            pass
            
        case _:
            pass
            
    
    # Clamp motor outputs
    if(vL > MAX_SPEED): vL = MAX_SPEED
    if(vL < -MAX_SPEED): vL = MAX_SPEED
    if(vR > MAX_SPEED): vR = MAX_SPEED
    if(vR < -MAX_SPEED): vR = MAX_SPEED
    
    # write actuators inputs
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
   
   
   
   
   
   