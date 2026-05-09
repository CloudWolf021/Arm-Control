import mujoco
import numpy as np
import math

import Vars
import MatrixHelper

'''
A module for solving the inverse kinematics problem using gradient descent.
'''


'''
Feed the found coefficients forwards, and get a specific output value.
Index is 0, 1, or 2, indicating which output value is obtained. In this
case, output values correspond to the true change produced by the joint
position changes (which are the coefficients being tuned). 
'''
def GetOutVal(index, jacobian, curVals):
    sum = 0
    for i in range(Vars.DOF):
        sum += curVals[i] * jacobian[index][i]
    return sum

# ###########################################################################################

'''
Use gradient descent to obtain a solution for joint_pos_delta_needed in 
jacobian * joint_pos_delta_needed = desiredXYZChange.

The optimization criteria is the squared error from the desired joint (x, y, z) change.

By using a default of all zeroes for init, we effectively resume from the previous location with 
zero joint position changes. 
'''
def GetSingleSolutionJacobian(jacobian, desiredXYZ, learningRate = 0.89, init = [0, 0, 0, 0, 0, 0, 0]):
    # Reference on list comprehension syntax: https://www.w3schools.com/python/python_lists_comprehension.asp
    cur = [init[i] for i in range(Vars.DOF)]
    numSteps = 60
    for i in range(numSteps):
        # Current computed changes
        ffX = GetOutVal(0, jacobian, cur)
        ffY = GetOutVal(1, jacobian, cur)
        ffZ = GetOutVal(2, jacobian, cur)

        errX = ffX-desiredXYZ[0]
        errY = ffY-desiredXYZ[1]
        errZ = ffZ-desiredXYZ[2]

        for j in range(Vars.DOF):
            # Gradients computed using squared error
            curGradX = 2*errX*jacobian[0][j]
            curGradY = 2*errY*jacobian[1][j]
            curGradZ = 2*errZ*jacobian[2][j]
            totalGrad = curGradX+curGradY+curGradZ
            cur[j] -= totalGrad*learningRate
    return np.array(cur)

# ###########################################################################################

'''
A modified gradient descent-based algorithm that left multiplies the standard equation by 
the transpose of the Jacobian on both sides, and performs an adjustment to attempt to avoid
singularities. 

Note: # Inspired by the Levenberg-Marquardt Algorithm: https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
'''

def GetSingleSolutionJacobianSpecial(jacobian, desiredXYZ, init = [0, 0, 0, 0, 0, 0, 0]):
    # Let the jacobian be J, and dj be the joint position changes, and dx be the overall position change
    # J*d=dx -> (J^T*J)*dj = J^T*dx
    # Since J^T*J is a square matrix, we can adjust it
    transpose = np.transpose(jacobian)
    raw = transpose @ jacobian

    adjusted = MatrixHelper.AdjustMatrix(raw)
    rhs = transpose @ desiredXYZ
    # Now solving adjusted*dj = rhs
    learningRate = 1.6
    cur = [init[i] for i in range(Vars.DOF)]
    numSteps = 145
    for i in range(numSteps):
        curLhs = adjusted @ np.array(cur)
        # For each joint
        for j in range(Vars.DOF):
            totalGrad = 0
            # Sum gradient for each output
            for k in range(Vars.DOF):
                totalGrad += 2 * (curLhs[k] - rhs[k]) * adjusted[k][j]
            #print(totalGrad)
            cur[j] -= totalGrad * learningRate
    return np.array(cur)

# ###########################################################################################

'''
Get a new update for the joint positions based on the least-squares solution found by gradient
descent.

Note: the solution obtained may not actually lead to the desired (x, y, z) position since the
Jacobian is not constant, and this method is used for just a single step of the solving process.

Note: Logic here is slightly different than for than for the other solvers so some code is 
duplicated here for simplicity. 

Note: this method supports standard gradient descent, gradient descent with learning rate adjustment, 
and special gradient descent using a modified matrix.
'''
def GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacobian, handleSingularity, adjustMatrix, isFirstArm = True):
    prevSq = Vars.SSQ_ERROR
    learningRate = 0.89
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

    # Track continuous lack of improvements
    # Instead of assessing raw change in error, it is assessed relative to the current error
    # Vars.IMPROVEMENT_FAIL_THRESHOLD/Vars.CHECK_IMPROVEMENT_FAIL_THRESHOLD is used as an approximate proportional threshold
    if (deltaErr/sq < Vars.IMPROVEMENT_FAIL_THRESHOLD/Vars.CHECK_IMPROVEMENT_FAIL_THRESHOLD and deltaErr > 0 and sq > Vars.CHECK_IMPROVEMENT_FAIL_THRESHOLD and prevSq > 0):
        Vars.CUR_FAIL_ITERS += 1
        if handleSingularity:
            learningRate = 1.8
    else:
       Vars.CUR_FAIL_ITERS = 0 
       if handleSingularity:
        # revert
            learningRate = 0.89

    param = 1
    # Get delta update in res
    if adjustMatrix:
        res = param*GetSingleSolutionJacobianSpecial(jacobian, np.array([dx, dy, dz]))
    else:
        # Standard
        res = param*GetSingleSolutionJacobian(jacobian, np.array([dx, dy, dz]), learningRate)

    offset = 0
    if not(isFirstArm):
        # Correctly index into qpos
        offset = Vars.DOF
    
    return [data.qpos[offset]+res[0], data.qpos[offset+1]+res[1], data.qpos[offset+2]+res[2], data.qpos[offset+3]+res[3], data.qpos[offset+4]+res[4], data.qpos[offset+5]+res[5], data.qpos[offset+6]+res[6]]
