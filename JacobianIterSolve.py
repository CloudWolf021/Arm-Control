import mujoco
import numpy as np
import math

import Vars
import MatrixHelper
import Helpers

'''
A module for the main iterative solver methods based off of the Jacobian, excluding
gradient descent. There are 2 core helpers and three variants that use the transpose,
pseudoinverse, or safe pseudoinverse of the Jacobian. 
'''

# BEGIN Main solver helpers

'''
Get the correct scalar factor for the iterative updates used in the Jacobian-based inverse kinematics methods

The parameter is 1 for the pseudoinverse-based methods, and is computed using a piecewise linear function
for the transpose case. This prevents stagnation in the position updates. The coefficients were found through tuning
to ensure that position updates are neither too aggressive nor overly slow. As the squared error decreases, the
parameter must generally increase. 
'''
def GetParam(sq, isTranspose) -> float:
    if isTranspose:
        # Local params for the piecewise linear model
        t1 = 0.0006
        v1 = 23
        t2 = 0.017
        v2 = 2.6
        t3 = 0.028
        v3 = 1.25
        
        if (sq < t1):
            return v1
        elif (t1 <= sq <= t2):
            return (sq-t1)/(t2-t1)*(v2-v1)+v1
        elif (t2 <= sq <= t3):
            return (sq-t2)/(t3-t2)*(v3-v2)+v2
        else:
            return v3
    else:
        return 1

# ###########################################################################################

'''
The main helper function used for the pseudoinverse and transpose solving methods

- This function first computes the current deviations in x, y, and z, along with the square error. 
- Then, it will compute the scaling parameter for the joint position update using the helper GetParam.
- Finally, the standard operation param*(approxMatrix @ np.array([dx, dy, dz])) is performed to 
  obtain the true updates, which are added to the current joint positions, to get the raw new joint
  positions.
'''
def GetRawJointPositionListHelper(data, model, x, y, z, jacobian, approxMatrix, isFirstArm, solverType):
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

    sq = dx**2+dy**2+dz**2
    Vars.SSQ_ERROR = sq
    deltaErr = prevSq - sq 
    mustCorrect = False

    # Track continuous lack of improvements
    if (deltaErr/sq < Vars.IMPROVEMENT_FAIL_THRESHOLD/Vars.CHECK_IMPROVEMENT_FAIL_THRESHOLD and deltaErr > 0 and sq > Vars.CHECK_IMPROVEMENT_FAIL_THRESHOLD and prevSq > 0):  
        Vars.CUR_FAIL_ITERS += 1
    else:
       Vars.CUR_FAIL_ITERS = 0 

    # Compute multiplier depending on squared error
    param = GetParam(sq, solverType == Vars.JT) 
    res = param*(approxMatrix @ np.array([dx, dy, dz]))
    
    offset = 0
    if not(isFirstArm):
        # Correctly index into qpos
        offset = Vars.DOF
    
    # Reference on list comprehension syntax: https://www.w3schools.com/python/python_lists_comprehension.asp
    return [data.qpos[offset+i]+res[i] for i in range(Vars.DOF)]

# END Main solver helpers

# ###########################################################################################

'''
Use the transpose of the Jacobian to iterate one step of the inverse kinematics solving.
'''
def GetRawJointPositionListJacobianT(data, model, x, y, z, jacobian, isFirstArm = True):
    return GetRawJointPositionListHelper(data, model, x, y, z, jacobian, np.transpose(jacobian), isFirstArm, Vars.JT)
    
# ###########################################################################################

'''
Get new joint positions using the raw pseudoinverse, which can be affected by singularities.
'''
def GetRawJointPositionListJacobianPInv(data, model, x, y, z, jacobian, isFirstArm = True):
    return GetRawJointPositionListHelper(data, model, x, y, z, jacobian, np.linalg.pinv(jacobian), isFirstArm, Vars.JPINV)
    
# ###########################################################################################

'''
Get new joint positions using a special, safe pseudoinverse.
'''
def GetRawJointPositionListJacobianPInvSpecial(data, model, x, y, z, jacobian, isFirstArm = True):
    return GetRawJointPositionListHelper(data, model, x, y, z, jacobian, MatrixHelper.GetAdjustedPInv(jacobian), isFirstArm, Vars.JPINVS)