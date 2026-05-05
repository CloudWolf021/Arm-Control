import mujoco
import numpy as np
import math


import Vars
import MatrixHelper

# Use the Jacobian and its transpose for the inverse kinematics
def GetRawJointPositionListJacobianT(data, model, x, y, z):

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


    # @@@@@@@@@@@@@@@@@@@@ 
    jacNeeded = np.zeros((3, Vars.DOF))
    jacOther = np.zeros((3, Vars.DOF))
    mujoco.mj_jacSite(model, data, jacNeeded, jacOther, 0)
    # After extracting jacobian, transpose
    trans = np.transpose(jacNeeded)

    # Get delta update
    res = param*(trans @ np.array([dx, dy, dz]))
    
    return [data.qpos[0]+res[0], data.qpos[1]+res[1], data.qpos[2]+res[2], data.qpos[3]+res[3], data.qpos[4]+res[4], data.qpos[5]+res[5], data.qpos[6]+res[6]]
    # Use prop to get deviation

# ###########################################################################################

# Use the Jacobian and its pseudoinverse for the inverse kinematics

def GetRawJointPositionListJacobianPInv(data, model, x, y, z):
    initX = data.site_xpos[0][0]
    initY = data.site_xpos[0][1]
    initZ = data.site_xpos[0][2]

    dx = x-initX
    dy = y-initY
    dz = z-initZ

    sq = dx**2+dy**2+dz**2
    Vars.SSQ_ERROR = sq
    param = 1  

    jacNeeded = np.zeros((3, Vars.DOF))
    jacOther = np.zeros((3, Vars.DOF))
    mujoco.mj_jacSite(model, data, jacNeeded, jacOther, 0)
    #trans = np.transpose(jacNeeded)
    ps = np.linalg.pinv(jacNeeded)
    
    res = param*(ps @ np.array([dx, dy, dz]))
    return [data.qpos[0]+res[0], data.qpos[1]+res[1], data.qpos[2]+res[2], data.qpos[3]+res[3], data.qpos[4]+res[4], data.qpos[5]+res[5], data.qpos[6]+res[6]]
    
# ###########################################################################################

# Use the Jacobian and a safe pseudoinverse for the inverse kinematics

def GetRawJointPositionListJacobianPInvSpecial(data, model, x, y, z):
    initX = data.site_xpos[0][0]
    initY = data.site_xpos[0][1]
    initZ = data.site_xpos[0][2]

    dx = x-initX
    dy = y-initY
    dz = z-initZ

    sq = dx**2+dy**2+dz**2
    Vars.SSQ_ERROR = sq
    param = 0.96  

    jacNeeded = np.zeros((3, Vars.DOF))
    jacOther = np.zeros((3, Vars.DOF))
    mujoco.mj_jacSite(model, data, jacNeeded, jacOther, 0)
    #trans = np.transpose(jacNeeded)
    ps = MatrixHelper.GetAdjustedPInv(jacNeeded)
    
    res = param*(ps @ np.array([dx, dy, dz]))
    return [data.qpos[0]+res[0], data.qpos[1]+res[1], data.qpos[2]+res[2], data.qpos[3]+res[3], data.qpos[4]+res[4], data.qpos[5]+res[5], data.qpos[6]+res[6]]
  