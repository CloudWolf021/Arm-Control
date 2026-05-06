import mujoco
import numpy as np
import math

import Vars

# Use gradient descent to step the current desired position

def GetRowOut(index, jacobian, curVals):
    sum = 0
    for i in range(Vars.DOF):
        sum += curVals[i] * jacobian[index][i]
    return sum

def GetSingleSolutionJacobian(jacobian, desiredXYZ, init = [0, 0, 0, 0, 0, 0, 0]):
    # Solve jacobian * joint_pos_delta_needed = desiredXYZChange using least-squares and gradient descent
    # By using a default of all zeroes, we effectively resume from the previous location with zero change
    # in joint positions
    # Reference on list comprehension syntax: https://www.w3schools.com/python/python_lists_comprehension.asp
    cur = [init[i] for i in range(Vars.DOF)]
    learningRate = 0.89
    numSteps = 60
    for i in range(numSteps):
        ffX = GetRowOut(0, jacobian, cur)
        ffY = GetRowOut(1, jacobian, cur)
        ffZ = GetRowOut(2, jacobian, cur)
        errX = ffX-desiredXYZ[0]
        errY = ffY-desiredXYZ[1]
        errZ = ffZ-desiredXYZ[2]
        for j in range(Vars.DOF):
            curGradX = 2*errX*jacobian[0][j]
            curGradY = 2*errY*jacobian[1][j]
            curGradZ = 2*errZ*jacobian[2][j]
            totalGrad = curGradX+curGradY+curGradZ
            cur[j] -= totalGrad*learningRate
    return np.array(cur)

def GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacobian, isFirstArm):
    prevSq = Vars.SSQ_ERROR

    index = 0
    if not(isFirstArm):
        # First arm
        index = 1
    initX = data.site_xpos[index][0]
    initY = data.site_xpos[index][1]
    initZ = data.site_xpos[index][2]

    dx = x-initX
    dy = y-initY
    dz = z-initZ

    # Compute multiplier depending on deviation

    sq = dx**2+dy**2+dz**2
    Vars.SSQ_ERROR = sq
    deltaErr = prevSq - sq 

    # Piecewise linear model to prevent stagnation in decreasing the end effector position error. 
    t1 = 0.0006
    v1 = 23
    t2 = 0.017
    v2 = 2.6
    t3 = 0.028
    v3 = 1.25
    
    if (sq < t1):
        param = v1
    elif (t1 <= sq <= t2):
        param = (sq-t1)/(t2-t1)*(v2-v1)+v1
    elif (t2 <= sq <= t3):
        param = (sq-t2)/(t3-t2)*(v3-v2)+v2
    else:
        param = v3

    #print(f"{sq}, {deltaErr}")
    if (deltaErr < Vars.IMPROVEMENT_FAIL_THRESHOLD and deltaErr > 0 and sq > t2):
        Vars.CUR_FAIL_ITERS += 1
    else:
       Vars.CUR_FAIL_ITERS = 0 

    param = 1

    # Get delta update
    #res = param*(trans @ np.array([dx, dy, dz]))
    res = param*GetSingleSolutionJacobian(jacobian, np.array([dx, dy, dz]))

    offset = 0
    if not(isFirstArm):
        # Correctly index into qpos
        offset = Vars.DOF
    
    return [data.qpos[offset]+res[0], data.qpos[offset+1]+res[1], data.qpos[offset+2]+res[2], data.qpos[offset+3]+res[3], data.qpos[offset+4]+res[4], data.qpos[offset+5]+res[5], data.qpos[offset+6]+res[6]]
