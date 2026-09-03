from controller import Robot, DistanceSensor, Motor
from enum import Enum

# time in [ms] of a simulation step
TIME_STEP = 64
MAX_SPEED = 6.28
TRAVEL_SPEED = 2 # Don't go faster, it breaks other things


# Wall Follow PID
FOLLOW_P = 0.0065
FOLLOW_D = 0.03


WALL_FOLLOW_DIST = 130
LIGHT_THRESHOLD = 1000

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

turn_counter = 0
TURN_180_STEPS = 63
ready_for_second_light = False

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    lsValues = []
    for i in range(8):
        lsValues.append(ls[i].getValue())
        
 
    firstLightDetected = (
        lsValues[0] < LIGHT_THRESHOLD and
        lsValues[1] < LIGHT_THRESHOLD and
        lsValues[2] < LIGHT_THRESHOLD and
        lsValues[5] < LIGHT_THRESHOLD and
        lsValues[6] > 3000 and
        lsValues[7] > 3000
    )
    
    secondLightDetected = (
        lsValues[0] < LIGHT_THRESHOLD and
        lsValues[2] < LIGHT_THRESHOLD and
        lsValues[5] < LIGHT_THRESHOLD and
        lsValues[6] < LIGHT_THRESHOLD and
        lsValues[7] < LIGHT_THRESHOLD
    )
    

    match curr_state.name:
        case "FOLLOW_L":
            err = WALL_FOLLOW_DIST - psValues[5]
            coeff = (err * FOLLOW_P) + ((err - prev_L_err) * FOLLOW_D)
            vL = TRAVEL_SPEED - coeff
            vR = TRAVEL_SPEED + coeff
            
            prev_L_err = err
            
            # TODO check light sensors & transition to 180 turn
            print(
                "CHECK:",
                round(lsValues[0]),
                round(lsValues[1]),
                round(lsValues[2]),
                round(lsValues[5]),
                round(lsValues[6]),
                round(lsValues[7])
            )
            if firstLightDetected:
                print("FIRST LIGHT -> TURN_180 now")
                turn_counter = 0
                ready_for_second_light = False
                curr_state = State.TURN_180
                
            elif psValues[7] > 80 and psValues[0] > 80:
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
            # TODO needs to be implemented
            vL = 1
            vR = -1
            turn_counter += 1
            
            
            if turn_counter >= TURN_180_STEPS:
                print("FOLLOW_R now")
                
                turn_counter = 0
                prev_R_err = WALL_FOLLOW_DIST - psValues[2]
                curr_state = State.FOLLOW_R
            
            
        case "FOLLOW_R":
            err = WALL_FOLLOW_DIST - psValues[2]
            coeff = (err * FOLLOW_P) + ((err - prev_R_err) * FOLLOW_D)
            vL = TRAVEL_SPEED + coeff
            vR = TRAVEL_SPEED - coeff
            
            prev_R_err = err
            
            # TODO check light sensors & transition to stop   
            if secondLightDetected:
                print("SECOND LIGHT -> STOP")
                curr_state = State.STOP

            elif psValues[7] > 150 and psValues[0] > 150:
                print("TURN_L now")
                curr_state = State.TURN_L
                
                
                
                
        case "TURN_L": # Necessary for wall-following on the right
            vL = -1
            vR = 1
            
            if(psValues[4] > 80 and psValues[3] > 80): 
                print("FOLLOW_R now")
                curr_state = State.FOLLOW_R
            
            pass
            
        case "STOP":
            # TODO needs to be implemented
            vL = 0
            vR = 0
            
        case _:
            pass
            
    
    # Clamp motor outputs
    if(vL > MAX_SPEED): vL = MAX_SPEED
    if(vL < -MAX_SPEED): vL = -MAX_SPEED
    if(vR > MAX_SPEED): vR = MAX_SPEED
    if(vR < -MAX_SPEED): vR = -MAX_SPEED
    
    # write actuators inputs
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
   
   
   
   
   
   