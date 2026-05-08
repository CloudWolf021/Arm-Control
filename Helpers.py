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

'''
Move each of the robot joints to a specified position by directly setting the core Mujoco control
inputs (data.ctrl). 

If requireValid is true and the input joint positions are not in the required ranges,
False will be returned to indicate an error.

This method supports both the first and second robot arm, by taking an input parameter and 
utilizing a start index. Compute
'''
def MoveToJointPositionsRaw(data, input, requireValid, isFirstArm = True) -> bool:
    if requireValid:
        if not(CheckPositions(input)): 
            return False
    inputValidated = ClampPositions(input)  

    # The second arm is controlled by indices 7-13, inclusive. 
    offset = 0
    if not(isFirstArm): 
        offset = Vars.DOF 

    # Tell each joint to move to a position; no control smoothness
    for i in range(Vars.DOF):
        data.ctrl[i + offset] = inputValidated[i]
    return True

# ###########################################################################################

'''
Move the first arm back to its initial position

Note: this is called when there is is an unreachable position or timeout for reaching the final
position. 
'''

def Reset(data, model, viewer):
    for i in range(Vars.DOF):
        data.ctrl[i] = 0
    data.ctrl[3] = -0.15

    for i in range(Vars.MODEL_STEP_ITERS):
        mujoco.mj_step(model, data)
    viewer.sync()

# ###########################################################################################

'''
Determine if either the first or second arm has reached the desired target position, using a 
specific threshold. This function takes global (x, y, z) coordinates, and obtains 
instantaneous global coordinates for the end effector, which are used for determining error. 

To determine deviation from the target position, squared error is used. 
'''
def HaveReachedTarget(data, x, y, z, delta = 0.0002, isFirstArm = True):
    # Must ensure to get the coordinates of the correct end effector - for the first arm
    # it is the first site, and for the second it is the second site. 
    index = 0
    if (not (isFirstArm)):
        # Second arm
        index = 1   
    xActual = data.site_xpos[index][0]
    yActual = data.site_xpos[index][1]
    zActual = data.site_xpos[index][2]

    err = (x-xActual)**2+ (y-yActual)**2+ (z-zActual)**2
    return (err < delta)

# ###########################################################################################

'''
Set the joint positions randomly (using data.ctrl, leading to delayed changes due to arm dynamics)

This method is only used for the primary robotic arm. 
''' 
def SetPosRandom(data):
    for i in range(Vars.DOF):
        curRand = random.random()

        # Get a random, valid joint position for each joint. 
        randomVal = Vars.JOINT_MIN_LIMITS[i] + (Vars.JOINT_MAX_LIMITS[i] - Vars.JOINT_MIN_LIMITS[i]) * curRand
        data.ctrl[i] = randomVal

# ###########################################################################################
# ###########################################################################################
# BEGIN Jacobian Helpers

'''
Take the relevant portion of the Jacobian for one of the robotic arms

If startIndex is 0, then the first Vars.DOF elements are taken for each row,
corresponding to the Jacobian for the first row. Else, if the startIndex is Vars.DOF,
the elements at indices 7-13, inclusive, for the second arm, are taken.
'''
def TruncateJacobian(jacobian, startIndex):
    res = np.zeros((3, Vars.DOF))
    for i in range(3):
        for j in range(Vars.DOF):
            res[i][j] = jacobian[i][j+startIndex]
    return res

# ###########################################################################################

'''
Using TruncateJacobian, extract the relevant 3 by Vars.DOF matrix for the first arm.
'''
def ExtractFirstJacobian(model, data, isLargeMatrix = True):
    numCols = Vars.ADJ_DOF
    if not(isLargeMatrix):
        numCols = Vars.DOF

    jacNeeded1 = np.zeros((3, numCols))
    jacOther1 = np.zeros((3, numCols))
    mujoco.mj_jacSite(model, data, jacNeeded1, jacOther1, 0)

    # Extract the relevant portion of the jacobian
    return TruncateJacobian(jacNeeded1, 0)

# ###########################################################################################

'''
Using TruncateJacobian, extract the relevant 3 by Vars.DOF matrix for the second arm.
'''
def ExtractSecondJacobian(model, data):
    jacNeeded2 = np.zeros((3, Vars.ADJ_DOF))
    jacOther2 = np.zeros((3, Vars.ADJ_DOF))
    mujoco.mj_jacSite(model, data, jacNeeded2, jacOther2, 1)

    # Extract the relevant portion of the jacobian
    return TruncateJacobian(jacNeeded2, Vars.DOF)

# END Jacobian Helpers

