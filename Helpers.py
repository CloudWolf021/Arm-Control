import math
import random
import numpy as np
import mujoco

import Vars

'''
A module with various motion and computation helpers
'''

'''
Check whether the input array of joint positions is legal given the joint limits of the arms.
'''
def CheckPositions(input) -> bool:
    # Verify that none of the joint limits are exceeded
    for i in range(Vars.DOF):
        if (input[i] < Vars.JOINT_MIN_LIMITS[i] or input[i] > Vars.JOINT_MAX_LIMITS[i]): return False
    return True

# ###########################################################################################

'''
Modify the input joint positions to ensure that they are in the required position ranges.
For each position, if it is in the valid range, it is unchanged; if it is less than the 
minimum, it is clamped to the minimum; if it is greater than the maximum, it is clamped to
the maximum. 
'''
def ClampPositions(input) -> list:
    inputOut = [0]*Vars.DOF
    for i in range(Vars.DOF):
        if (input[i] >= Vars.JOINT_MIN_LIMITS[i] and input[i] <= Vars.JOINT_MAX_LIMITS[i]): 
            inputOut[i] = input[i]
        elif (input[i] < Vars.JOINT_MIN_LIMITS[i]): 
            inputOut[i] = Vars.JOINT_MIN_LIMITS[i]  # Clamp to minimum value
        else:  # Exceeds maximum
            inputOut[i] = Vars.JOINT_MAX_LIMITS[i]  # Clamp to maximum value

    return inputOut

# ###########################################################################################

# Move each of the robot joints to a specified position
# is requireValid is true and the input joints are not all in the required limits, 
# False will be returned to indicate an error
# Will support indexing for the second arm roo
def MoveToJointPositionsRaw(data, input, requireValid, isFirstArm = True) -> bool:
    if requireValid:
        if not(CheckPositions(input)): return False
    inputValidated = ClampPositions(input)  

    offset = 0
    if not(isFirstArm): 
        offset = Vars.DOF 

    # Tell each joint to move to a position; no control smoothness
    for i in range(Vars.DOF):
        data.ctrl[i + offset] = inputValidated[i]
    return True

# ###########################################################################################

# Compute the mean squared error for the arm position relative to the desired joint angles
def ComputeError(data, required) -> float:
    sum = 0
    for i in range(Vars.DOF):
        sum += ((data[i] - required[i])**2)
    return sum / Vars.DOF

# Input coordinates are global, and the instantaneous global coordinates are also obtained. 

def HaveReachedTarget(data, x, y, z, delta = 0.0002, isFirstArm = True):
    index = 0
    if (not (isFirstArm)):
        # Second arm
        index = 1   
    xActual = data.site_xpos[index][0]
    yActual = data.site_xpos[index][1]
    zActual = data.site_xpos[index][2]


    err = (x-xActual)**2+ (y-yActual)**2+ (z-zActual)**2
    #print(xActual)
    #print(yActual)
    #print(zActual)
    return (err < delta)

    
def SetPosRandom(data):
    for i in range(Vars.DOF):
        curRand = random.random()
        randomVal = Vars.JOINT_MIN_LIMITS[i] + (Vars.JOINT_MAX_LIMITS[i] - Vars.JOINT_MIN_LIMITS[i]) * curRand
        data.ctrl[i] = randomVal

def TruncateJacobian(jacobian, startIndex):
    res = np.zeros((3, Vars.DOF))
    for i in range(3):
        for j in range(Vars.DOF):
            res[i][j] = jacobian[i][j+startIndex]
    return res

def ExtractFirstJacobian(model, data):
    jacNeeded1 = np.zeros((3, Vars.ADJ_DOF))
    jacOther1 = np.zeros((3, Vars.ADJ_DOF))
    mujoco.mj_jacSite(model, data, jacNeeded1, jacOther1, 0)

    # Extract the relevant portion of the jacobian
    return TruncateJacobian(jacNeeded1, 0)

def ExtractSecondJacobian(model, data):
    jacNeeded2 = np.zeros((3, Vars.ADJ_DOF))
    jacOther2 = np.zeros((3, Vars.ADJ_DOF))
    mujoco.mj_jacSite(model, data, jacNeeded2, jacOther2, 1)

    # Extract the relevant portion of the jacobian
    return TruncateJacobian(jacNeeded2, Vars.DOF)
