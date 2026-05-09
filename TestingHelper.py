import mujoco
import numpy as np
import time

import Vars
import Helpers
import JacobianGradSolve
import JacobianIterSolve
import Model

'''
A helper module for testing the different inverse kinematics solver methods.
'''


'''
Test a single command to a position (x, y, z) for a specific control type/method. 
This will return a result list in the form [state, iterations, deltaTimeMs].

Note: control with reverting back to the home position is not tested
'''
def TestSingleCommand(data, model, controlType, x, y, z):
    iter = 0
    Vars.CUR_FAIL_ITERS = 0
    startTime = time.perf_counter()
    while not(Helpers.HaveReachedTarget(data, x, y, z, 0.0008) or Vars.CUR_FAIL_ITERS == Vars.NUM_ITERS_FAIL or iter == Vars.TIMEOUT_ITERS):
        iter+=1
        # For the first step, will have 1 iteration, which is correct
        mujoco.mj_step1(model, data)

        # Extract the portion of the jacobian for (x, y, z) position
        jacNeeded = np.zeros((3, Vars.DOF))
        jacOther = np.zeros((3, Vars.DOF))
        mujoco.mj_jacSite(model, data, jacNeeded, jacOther, 0)

        # Call the appropriate method depending on what control type is desired. 
        if (controlType == Vars.JT):
            Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianT(data, model, x, y, z, jacNeeded), False)
        elif (controlType == Vars.JPINV):
            Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInv(data, model, x, y, z, jacNeeded), False)
        elif (controlType == Vars.JPINVS):
            Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInvSpecial(data, model, x, y, z, jacNeeded), False)
        elif (controlType == Vars.JSOLVE):
            Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacNeeded, False, False), False)
        elif (controlType == Vars.JSOLVELR):
            # Parameter for handling singularities is True
            Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacNeeded, True, False), False)
        elif (controlType == Vars.JSOLVEM):
            # Parameter for adjusting matrix is True
            Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacNeeded, False, True), False)        
        elif (controlType == Vars.MODEL):
            Helpers.MoveToJointPositionsRaw(data, Model.GetRawJointPositionListModel(data, model, x, y, z), False)
            mujoco.mj_step2(model, data)  
            
            # Will always get the same output from the model -> must allow the internal simulation to iterate and reach the target joint positions. 
            # Note: this will not lead to the timeout due to the magnitude of Vars.MODEL_STEP_ITERS 
            for i in range(Vars.MODEL_STEP_ITERS):
                mujoco.mj_step(model, data)
                iter+=1
            break
        mujoco.mj_step2(model, data)

    state = Vars.SUCCESS
    if (iter == Vars.TIMEOUT_ITERS):
        state = Vars.TIMEOUT
    elif (Vars.CUR_FAIL_ITERS == Vars.NUM_ITERS_FAIL):
        state = Vars.UNREACHABLE
    elif (controlType == Vars.MODEL):
        if Helpers.HaveReachedTarget(data, x, y, z, 0.0008):
            state = Vars.SUCCESS
        else:
            # Put as unreachable as a placeholder for failure
            state = Vars.UNREACHABLE

    # time source: https://docs.python.org/3/library/time.html#time.perf_counter
    newTime = time.perf_counter()
    deltaTimeMs = int(1000*(newTime - startTime))

    return [state, iter, deltaTimeMs]


# ###########################################################################################

'''
Test a motion trace for a specific control type. 

Note: there is some code duplication, but it is simpler to have slightly different logic in this method
Note: paths is a list of lists, where internal lists are required to be well-formed, in the form [x, y, z]
Note: if resetBetweenSegments, each portion of the trace is considered separately, and the arm starts from
its base orientation in each case. 
'''

def TestTraceSingleMethod(controlType, path, resetBetweenSegments):
    try:
        # Load model and data
        model = mujoco.MjModel.from_xml_path(Vars.ARM_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"There was an error obtaining the model. Path may not exist or something else went wrong. The error is {e}")
        return
    totalTime = 0
    totalIters = 0
    successCount = 0
    timeoutCount = 0
    unreachableCount = 0
    totalTimeUnreachable = 0
    totalItersUnreachable = 0

    indivResults = []

    for p in path:
        x = p[0]
        y = p[1]
        z = p[2]
        res = TestSingleCommand(data, model, controlType, x, y, z)
        state = res[0]
        indivResults.append(state)
        iters = res[1] 
        time = res[2]

        totalTime += time
        totalIters += iters
        
        if state == Vars.SUCCESS:
            successCount += 1
        elif state == Vars.TIMEOUT:
            timeoutCount += 1
        else:
            unreachableCount += 1
            totalTimeUnreachable += time
            totalItersUnreachable += iters
        
        if resetBetweenSegments:
            # Will most likely succeed since already loaded once - assuming no external interference
            # Thus, not error checking
            model = mujoco.MjModel.from_xml_path(Vars.ARM_PATH)
            data = mujoco.MjData(model)

        
    avgTimeUnreachable = 0
    avgItersUnreachable = 0
    if unreachableCount > 0:
        avgTimeUnreachable = int(totalTimeUnreachable/unreachableCount)
        avgItersUnreachable = int(totalItersUnreachable/unreachableCount)
        
    # Return format
    output = [totalTime, totalIters, successCount, timeoutCount, unreachableCount, avgTimeUnreachable, avgItersUnreachable]
    for stateOut in indivResults:
        output.append(stateOut)
    return output
        
# ###########################################################################################

'''
Test a specific movement command trace on all methods, and display the results for each.
'''

def TestTraceAllMethods(path, resetBetweenSegments):
    pathCount = len(path)
    # Use consecutive nature of numerical ids
    results = [TestTraceSingleMethod(id, path, resetBetweenSegments) for id in range(Vars.JT, Vars.MODEL+1)]

    # Will look at the results over all methods, and if it the point was reachable, then it is overall
    # Will be used to access failures of timeouts and unreachable point detection.
    falseUnreachable = [0]*Vars.MODEL
    falseTimeout = [0]*Vars.MODEL
    for i in range(pathCount):
        curReachable = False
        # Corresponds to the number of methods, since they are 1-indexed, and this is the highest index
        for j in range(Vars.MODEL):
            # Access individual results after the overall statistics
            if results[j][i+Vars.TEST_PATH_RES_COUNT] == Vars.SUCCESS:
                curReachable = True
                break
        # Check false detections of unreachability and faulty timeouts
        if curReachable:
            for j in range(Vars.MODEL):
                if results[j][i+Vars.TEST_PATH_RES_COUNT] == Vars.UNREACHABLE:
                    # Adjust -> not truly unreachable
                    falseUnreachable[j] += 1
                elif results[j][i+Vars.TEST_PATH_RES_COUNT] == Vars.TIMEOUT:
                    # Adjust -> should not have timed out
                    falseTimeout[j] += 1

    # Print information for each method
    for i in range(Vars.MODEL):
        print(f"{Vars.METHOD_NAMES[i]}:\nTime (ms): {results[i][0]}, Iters: {results[i][1]}, " + 
              f"Successes: {results[i][2]}, Timeouts: {results[i][3]} (False: {falseTimeout[i]}), " + 
              f"Unreachable: {results[i][4]} (False: {falseUnreachable[i]}), " + 
              f"Avg. Unreachable Time: {results[i][5]}, Avg. Unreachable Iters: {results[i][6]}")