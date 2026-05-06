import mujoco
import mujoco.viewer
import numpy as np
import math
import random

import Helpers
# Global variables and constants
import Vars
import Model
import JacobianIterSolve
import JacobianGradSolve

# Helper source https://mujoco.readthedocs.io/en/stable/python.html

###############################################################################
###############################################################################

# High-level data collection routine for the model
def CollectDataRoutine(seedIn = 11538813):
    try:
        # Load model and data
        model = mujoco.MjModel.from_xml_path(Vars.ARM_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print("There was an error obtaining the model. Path may not exist or something else went wrong. The error is ")
        print(e)
        return

    iter = 0
    random.seed(seedIn)
    j1 = []
    j2 = []
    j3 = []
    j4 = []
    j5 = []
    j6 = []
    j7 = []
    x = []
    y = []
    z = []

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while iter < Vars.NUM_ITERS_COLLECT:
            iter+=1
            # Helper source https://mujoco.readthedocs.io/en/stable/programming/simulation.html#
            mujoco.mj_step(model, data)
            if (iter % 60 == 0):
                Helpers.SetPosRandom(data)
            mujoco.mj_step2(model, data)

            j1.append(data.qpos[0])
            j2.append(data.qpos[1])
            j3.append(data.qpos[2])
            j4.append(data.qpos[3])
            j5.append(data.qpos[4])
            j6.append(data.qpos[5])
            j7.append(data.qpos[6])

            x.append(data.site_xpos[0][0])
            y.append(data.site_xpos[0][1])
            z.append(data.site_xpos[0][2])

            viewer.sync()

        # Finally, fit a linear model
        Model.FitLinearModel(x, y, z, j1)
        Model.FitLinearModel(x, y, z, j2)
        Model.FitLinearModel(x, y, z, j3)
        Model.FitLinearModel(x, y, z, j4)
        Model.FitLinearModel(x, y, z, j5)
        Model.FitLinearModel(x, y, z, j6)
        Model.FitLinearModel(x, y, z, j7)

# ###########################################################################################

# Attempt to move to a specific position with no safety checks besides the iteration timeout
# Intended to be used as a helper for when a sphere is moved between the two arms
# The input (x, y, z) coordinates are global
# Smaller delta so that the arm does not try to go further into the sphere
# This method can handle moving either arm

def MoveToLocationUnchecked(x, y, z, model, data, viewer, controlType, isFirstArm = True, delta = 0.005):
    iter = 0
    # Wait additional iterations for the system to stabilize
    waitIters = 0
    while (iter < Vars.TIMEOUT_ITERS + Vars.EXTRA_WAIT_ITERS):
        if Helpers.HaveReachedTarget(data, x, y, z, delta, isFirstArm):
            if waitIters == Vars.EXTRA_WAIT_ITERS:
                print("Reached Point")
                return
            else:
                waitIters += 1
                iter += 1
                continue
        iter += 1
        #print(data.qpos)
        #data.ctrl[2*Vars.DOF]  = 0.2
        #print(data.qpos)

        mujoco.mj_step1(model, data)

        if (isFirstArm):
            jacobian = Helpers.ExtractFirstJacobian(model, data)
        else:
            jacobian = Helpers.ExtractSecondJacobian(model, data)

        # Call the appropriate method depending on what control type is desired. 
        # The control routine using an unreliable global model is not used. 
        if (controlType == Vars.JT):
            Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianT(data, model, x, y, z, jacobian, isFirstArm), False, isFirstArm)
        if (controlType == Vars.JPINV):
            Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInv(data, model, x, y, z, jacobian, isFirstArm), False, isFirstArm)
        if (controlType == Vars.JSOLVE):
            Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacobian, isFirstArm), False, isFirstArm)
        elif (controlType == Vars.JPINVS):
            Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInvSpecial(data, model, x, y, z, jacobian, isFirstArm), False, isFirstArm)
        
        mujoco.mj_step2(model, data)
                
        viewer.sync()
    print("Failed to reach point")
        

# This method can only handle moving one of the arms

def ControlRoutine(controlType, verbose = False, delta = 0.0008):
    try:
        # Load model and data
        model = mujoco.MjModel.from_xml_path(Vars.ARM_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"There was an error obtaining the model. Path may not exist or something else went wrong. The error is {e}")
        return

    iter = 0
    curStartIter = 0
    # Set to initial end effector positions -> effectively 0 error
    xN = 0
    yN = 0
    zN = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while iter < Vars.NUM_ITERS_REG:
            if (iter == 0 or Helpers.HaveReachedTarget(data, xN, yN, zN, delta) or 
               Vars.CUR_FAIL_ITERS == Vars.NUM_ITERS_FAIL or 
               (iter - curStartIter) == Vars.TIMEOUT_ITERS): 
                # prompt input
                if (iter > 0):
                    print(f"The number of iterations is {iter - curStartIter} and error is {Vars.SSQ_ERROR}")
                    curStartIter = 0
                if ((iter - curStartIter) == Vars.TIMEOUT_ITERS):
                    print("Iteration limit exceeded. Position may also be unreachable.")
                if (Vars.CUR_FAIL_ITERS == Vars.NUM_ITERS_FAIL):
                    Vars.CUR_FAIL_ITERS = 0
                    print("The position is not reachable.")
                print("Enter a position for the arm to move to:\nX = ", end = "")
                # Use eval: source GeeksForGeeks: https://www.geeksforgeeks.org/python/eval-in-python/
                xN = eval(input())
                print("\nY = ", end = "")
                yN = eval(input())
                print("\nZ = ", end = "")
                zN = eval(input())
                curStartIter = iter

            iter+=1
            # For the first step, will have 1 iteration, which is correct
            mujoco.mj_step1(model, data)

            # Extract the portion of the jacobian for (x, y, z) position
            jacNeeded = np.zeros((3, Vars.DOF))
            jacOther = np.zeros((3, Vars.DOF))
            mujoco.mj_jacSite(model, data, jacNeeded, jacOther, 0)

            # Call the appropriate method depending on what control type is desired. 
            if (controlType == Vars.JT):
                Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianT(data, model, xN, yN, zN, jacNeeded), False)
            if (controlType == Vars.JPINV):
                Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInv(data, model, xN, yN, zN, jacNeeded), False)
            if (controlType == Vars.JSOLVE):
                Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, xN, yN, zN, jacNeeded), False)
            elif (controlType == Vars.MODEL):
                Helpers.MoveToJointPositionsRaw(data, Model.GetRawJointPositionListModel(data, model, xN, yN, zN), False)
            elif (controlType == Vars.JPINVS):
                Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInvSpecial(data, model, xN, yN, zN, jacNeeded), False)

            if verbose: print(f"x: {data.site_xpos[0][0]} / wanted {xN}, y: {data.site_xpos[0][1]} / wanted {yN}, z: {data.site_xpos[0][2]} / wanted {zN}")
            
            mujoco.mj_step2(model, data)

            viewer.sync()

# ###########################################################################################

# Move a object between 2 versions of the arm

def MoveObjectRoutine():
    try:
        # Load model and data
        model = mujoco.MjModel.from_xml_path(Vars.DUAL_ARM_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"There was an error obtaining the model. Path may not exist or something else went wrong. \nThe error is {e}.")
        return
    with mujoco.viewer.launch_passive(model, data) as viewer:
        MoveToLocationUnchecked(Vars.ARM1_HOME_X, Vars.ARM1_HOME_Y, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE)
        MoveToLocationUnchecked(Vars.ARM1_HOME_X, Vars.ARM2_OFFSET-Vars.ARM1_HOME_Y, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE, False)
        
        # The arms continuously trade off control and try to keep a sphere between them
        while(True):
            horizPos = data.site_xpos[2][0]
            if horizPos > Vars.SPHERE_CENTER_OFFSET/2:
                horizPos += 2.4*Vars.SPHERE_CENTER_OFFSET
            elif horizPos < -Vars.SPHERE_CENTER_OFFSET/2:
                horizPos -= 2.4*Vars.SPHERE_CENTER_OFFSET    
            MoveToLocationUnchecked(horizPos, data.site_xpos[2][1]-Vars.SPHERE_CENTER_OFFSET, Vars.ARM_LOW_Z, model, data, viewer, Vars.JSOLVE)
            MoveToLocationUnchecked(data.site_xpos[2][0] * Vars.SPHERE_HORIZ_OFFSET_MUL, data.site_xpos[2][1]-Vars.SPHERE_CENTER_OFFSET, Vars.ARM_LOW_Z, model, data, viewer, Vars.JSOLVE)
            MoveToLocationUnchecked(Vars.ARM1_HOME_X, Vars.ARM1_HOME_Y, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE)

            horizPos = data.site_xpos[2][0]
            if horizPos > Vars.SPHERE_CENTER_OFFSET/2:
                horizPos += 2.4*Vars.SPHERE_CENTER_OFFSET
            elif horizPos < -Vars.SPHERE_CENTER_OFFSET/2:
                horizPos -= 2.4*Vars.SPHERE_CENTER_OFFSET    
            # Second arm
            MoveToLocationUnchecked(horizPos, data.site_xpos[2][1]+Vars.SPHERE_CENTER_OFFSET, Vars.ARM_LOW_Z, model, data, viewer, Vars.JSOLVE, False)
            MoveToLocationUnchecked(data.site_xpos[2][0] * Vars.SPHERE_HORIZ_OFFSET_MUL, data.site_xpos[2][1]+Vars.SPHERE_CENTER_OFFSET, Vars.ARM_LOW_Z, model, data, viewer, Vars.JSOLVE, False)
            MoveToLocationUnchecked(Vars.ARM1_HOME_X, Vars.ARM2_OFFSET-Vars.ARM1_HOME_Y, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE, False)

    return


# ###########################################################################################
# ###########################################################################################
# ###########################################################################################

# Main entry point which handles base input and runs either data collection or a high-level
# control routine where the user indicates what positions the robot arm to move to
def main ():
    print("Please enter a mode:\n1. Run control\n2. Collect Data\n3. Move Sphere Between Arms")
    choice = input()
    
    if (not(choice == "1" or choice == "2" or choice == "3")):
        print("Invalid. Terminating.")
        return
    if (choice == "1"):
        print("Please enter a solving method:\n1. Solve with transpose\n2. Solve with raw psuedoinverse\n3. Solve with safe pseudoinverse\n4. Solve with gradient descent\n5. Solve with model (not recommended)\n")
        solverChoice = input()
        firstChar = solverChoice[0]
        charASCII = ord(firstChar)
        if (charASCII >= Vars.ASCII_1 and charASCII <= Vars.ASCII_5):
            # Exploit sequential order of options
            ControlRoutine(charASCII - Vars.ASCII_1 + 1, False, 0.00008)   
            return
        print("Invalid. Terminating.")
        return
    elif (choice == "2"):  
        CollectDataRoutine()
        return
    else:   # Move object
        MoveObjectRoutine()
        return

# Run the main entry point
main()