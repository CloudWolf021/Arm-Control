import mujoco
import numpy as np
import math


import Vars

def GetRowOut(index, jacobian, curVals):
    sum = 0
    for i in range(Vars.DOF):
        sum += curVals[i] * jacobian[index][i]
    return sum

def GetSingleSolutionJacobian(jacobian, desiredXYZ, init = [0, 0, 0, 0, 0, 0, 0]):
    # Solve jacobian * joint_pos_delta_needed = desiredXYZChange using least-squares and gradient descent
    cur = [init[0], init[1], init[2], init[3], init[4], init[5], init[6]]
    learningRate = 0.04
    numSteps = 200
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

def GetRawJointPositionListJacobianSolve(data, model, x, y, z):
    prevSq = Vars.SSQ_ERROR
    initX = data.site_xpos[0][0]
    initY = data.site_xpos[0][1]
    initZ = data.site_xpos[0][2]

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

    # @@@@@@@@@@@@@@@@@@@@ 
    jacNeeded = np.zeros((3, Vars.DOF))
    jacOther = np.zeros((3, Vars.DOF))
    jac = mujoco.mj_jacSite(model, data, jacNeeded, jacOther, 0)
    # After extracting jacobian, transpose
    #trans = np.transpose(jacNeeded)

    # Get delta update
    #res = param*(trans @ np.array([dx, dy, dz]))
    res = param*GetSingleSolutionJacobian(jacNeeded, np.array([dx, dy, dz]))
    
    return [data.qpos[0]+res[0], data.qpos[1]+res[1], data.qpos[2]+res[2], data.qpos[3]+res[3], data.qpos[4]+res[4], data.qpos[5]+res[5], data.qpos[6]+res[6]]
