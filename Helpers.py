import math
import random
import Vars

# Check whether the array of joint positions is legal
def CheckPositions(input) -> bool:
    # Verify that none of the joint limits are exceeded
    for i in range(Vars.DOF):
        if (input[i] < Vars.JOINT_MIN_LIMITS[i] or input[i] > Vars.JOINT_MAX_LIMITS[i]): return False
    return True

# ###########################################################################################

# Modify the input joint positions to ensure that they are in the required ranges
def ClampPositions(input) -> list:
    inputOut = [0]*Vars.DOF
    for i in range(Vars.DOF):
        if (input[i] >= Vars.JOINT_MIN_LIMITS[i] and input[i] <= Vars.JOINT_MAX_LIMITS[i]): 
            inputOut[i] = input[i]
        elif (input[i] < Vars.JOINT_MIN_LIMITS[i]): 
            inputOut[i] = Vars.JOINT_MIN_LIMITS[i]  # Clamp to minimum value

        else: 
            v = input[i]
            dev = v - Vars.JOINT_MAX_LIMITS[i]
            dif = Vars.JOINT_MAX_LIMITS[i] - Vars.JOINT_MIN_LIMITS[i]
            ratio = 1.0*dev / dif
            t = math.trunc(ratio)
            valueFinal = Vars.JOINT_MIN_LIMITS[i] + (ratio - t)*dif
            #print(valueFinal)
            inputOut[i] = valueFinal   # Too large; clamp to maximum value
            #print(f"Converted {input[i]} to {inputOut[i]}")
            inputOut[i] = valueFinal  #JOINT_MAX_LIMITS[i]

    return inputOut

# ###########################################################################################

# Move each of the robot joints to a specified position
# is requireValid is true and the input joints are not all in the required limits, 
# False will be returned to indicate an error
def MoveToJointPositionsRaw(data, input, requireValid) -> bool:
    if requireValid:
        if not(CheckPositions(input)): return False
    inputValidated = ClampPositions(input)   

    # Tell each joint to move to a position; no control smoothness
    for i in range(Vars.DOF):
        data.ctrl[i] = inputValidated[i]
    return True

# ###########################################################################################

# Compute the mean squared error for the arm position relative to the desired joint angles
def ComputeError(data, required) -> float:
    sum = 0
    for i in range(Vars.DOF):
        sum += ((data[i] - required[i])**2)
    return sum / Vars.DOF


def HaveReachedTarget(data, x, y, z, delta = 0.0002):
    xActual = data.site_xpos[0][0]
    yActual = data.site_xpos[0][1]
    zActual = data.site_xpos[0][2]

    err = (x-xActual)**2+ (y-yActual)**2+ (z-zActual)**2
    return (err < delta)

    
def SetPosRandom(data):
    for i in range(Vars.DOF):
        curRand = random.random()
        randomVal = Vars.JOINT_MIN_LIMITS[i] + (Vars.JOINT_MAX_LIMITS[i] - Vars.JOINT_MIN_LIMITS[i]) * curRand
        data.ctrl[i] = randomVal