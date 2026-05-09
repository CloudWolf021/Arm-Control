import mujoco
import mujoco.viewer
import numpy as np
import math
import random
import time

import Helpers
import TestingHelper
import Vars
import Model
import JacobianIterSolve
import JacobianGradSolve

'''
The main module that orchestrates the desired functionalities such as collecting data, moving an arm between points in space,
running a simulation where two identical arms pass a ball between each other, and collecting data on the methods.

- Key helper source https://mujoco.readthedocs.io/en/stable/python.html
'''


###############################################################################
###############################################################################


'''
High-level data collection routine for the fitted model. 

The arm is moved between randomly generated joint position sets, and the joint positions in
addition to the output are recorded.
'''
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

'''
Attempt to move to a specific position with no safety checks besides the iteration timeout.
The input coordinates are global, and are not relative to each arm. 

Note: This is intended to be used as a helper for when a sphere is moved between the two arms.
It can be used for either arm. 
'''
def MoveToLocationUnchecked(x, y, z, model, data, viewer, controlType, isLargeMatrix, isFirstArm = True, delta = 0.005):
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

        mujoco.mj_step1(model, data)

        if (isFirstArm):
            jacobian = Helpers.ExtractFirstJacobian(model, data, isLargeMatrix)
        else:
            jacobian = Helpers.ExtractSecondJacobian(model, data)

        # Call the appropriate method depending on what control type is desired. 
        # The control routine using an unreliable global model is not used. 
        if (controlType == Vars.JT):
            Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianT(data, model, x, y, z, jacobian, isFirstArm), False, isFirstArm)
        elif (controlType == Vars.JPINV):
            Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInv(data, model, x, y, z, jacobian, isFirstArm), False, isFirstArm)
        elif (controlType == Vars.JSOLVE):
            Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacobian, False, isFirstArm), False, isFirstArm)
        elif (controlType == Vars.JSOLVELR):
            # Parameter for handling singularities is True
            Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacobian, True, False, isFirstArm), False, isFirstArm)
        elif (controlType == Vars.JSOLVEM):
            # Parameter for adjusting matrix is True
            Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacobian, False, True, isFirstArm), False, isFirstArm)
        elif (controlType == Vars.JPINVS):
            Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInvSpecial(data, model, x, y, z, jacobian, isFirstArm), False, isFirstArm)
        
        mujoco.mj_step2(model, data)
                
        viewer.sync()
    print("Failed to reach point")
        
# ###########################################################################################

'''
A high-level test routine that will run each solving method on several traces to collect data
regarding their efficacy. Metrics such as successes, iterations, and times will be reported. 
'''

def TestRoutine():
    TestingHelper.TestTraceAllMethods([[0.4, 0.4, 0.4]], False)
    
    # Test positions where x/y are -0.7, -0.35, 0, 0.35, 0.7 
    # z is 0, 0.2, 0.4, 0.6, 0.8, 1
    # Some of these positions are unreachable, as a test
    # All motions are separate, and a simulation initialization is performed prior to each one
    singleSequence = []
    for i in range(-2, 3):
        for j in range(-2, 3):
            for k in range(6):
                x = 0.35*i 
                y = 0.35*j 
                z = 0.2*k
                singleSequence.append([x, y, z])

    TestingHelper.TestTraceAllMethods(singleSequence, True)

# ###########################################################################################

'''
The high-level control routine for moving an arm between global positions. 
- This method will first launch the simulation
- Then, it will continually ask the user for input (x, y, z) coordinates and attempt to move the arm
  to them
- There is a high-level iteration limit and each specific movement has a limited iteration count.

If singularityAdj is enabled, the arm will perform corrective measures when a possible singularity is detected.

Note: This method can only handle moving only the first arm. MoveToLocationUnchecked can handle
either arm.
'''
def ControlRoutine(controlType, numStdUpdateSteps, verbose = False, delta = 0.0008, singularityAdj = False):
    try:
        # Load model and data
        model = mujoco.MjModel.from_xml_path(Vars.ARM_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"There was an error obtaining the model. Path may not exist or something else went wrong. The error is {e}")
        return

    iter = 0
    curStartIter = 0
    startTime = 0
    # Set to initial end effector positions -> effectively 0 error
    xN = 0
    yN = 0
    zN = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while iter < Vars.NUM_ITERS_REG:
            if (iter == 0 or Helpers.HaveReachedTarget(data, xN, yN, zN, delta) or 
               Vars.CUR_FAIL_ITERS == Vars.NUM_ITERS_FAIL or 
               (iter - curStartIter) == Vars.TIMEOUT_ITERS): 
                # Handle getting new input and possibly reporting outcome of previous motion 

                if (iter > 0):
                    # In the case of using the model, iter will be 0 and this logic will correctly not be executed. 

                    if verbose: 
                        print("SUMMARY:\n")

                        if ((iter - curStartIter) == Vars.TIMEOUT_ITERS):
                            if singularityAdj:
                                Helpers.Reset(data, model, viewer)
                            print("- Iteration limit exceeded. Position may be unreachable.\n")
                        elif (Vars.CUR_FAIL_ITERS == Vars.NUM_ITERS_FAIL):
                            if singularityAdj:
                                Helpers.Reset(data, model, viewer)
                            print("- The position appears to be unreachable.\n")

                        print(f"- x: {data.site_xpos[0][0]} / wanted {xN}, y: {data.site_xpos[0][1]} / wanted {yN}, z: {data.site_xpos[0][2]} / wanted {zN}\n")
                        print(f"- Summed Squared Error is {Vars.SSQ_ERROR} and the average absolute error per axis is {np.sqrt(Vars.SSQ_ERROR/3)}\n")
                        print(f"- The number of outer iterations is {iter - curStartIter}.\n")

                        # time source: https://docs.python.org/3/library/time.html#time.perf_counter
                        newTime = time.perf_counter()
                        print(f"- Delta time is {1000*(newTime-startTime)} ms.")
   
                # Prompt input
                print("Enter a position for the arm to move to:\nX = ", end = "")

                # Use eval: source GeeksForGeeks: https://www.geeksforgeeks.org/python/eval-in-python/
                xN = eval(input())
                print("\nY = ", end = "")
                yN = eval(input())
                print("\nZ = ", end = "")
                zN = eval(input())

                curStartIter = iter
                Vars.CUR_FAIL_ITERS = 0
                startTime = time.perf_counter()


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
            elif (controlType == Vars.JPINV):
                Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInv(data, model, xN, yN, zN, jacNeeded), False)
            elif (controlType == Vars.JPINVS):
                Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInvSpecial(data, model, xN, yN, zN, jacNeeded), False)
            elif (controlType == Vars.JSOLVE):
                Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, xN, yN, zN, jacNeeded, False, False), False)
            elif (controlType == Vars.JSOLVELR):
                # Parameter for handling singularities is True
                Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, xN, yN, zN, jacNeeded, True, False), False)
            elif (controlType == Vars.JSOLVEM):
                # Parameter for adjusting matrix is True
                Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, xN, yN, zN, jacNeeded, False, True), False)        
            elif (controlType == Vars.MODEL):
                Helpers.MoveToJointPositionsRaw(data, Model.GetRawJointPositionListModel(data, model, xN, yN, zN), False)
                
                # Will always get the same output -> must allow the internal simulation to iterate and reach the target joint positions. 
                for i in range(Vars.MODEL_STEP_ITERS):
                    mujoco.mj_step(model, data)
                    viewer.sync()

                # Will indicate to prompt the user for new input by triggering the if statement 
                iter = 0
                curX = data.site_xpos[0][0]
                curY = data.site_xpos[0][1]
                curZ = data.site_xpos[0][2]
                sqErr = (xN-curX)**2+(yN-curY)**2+(zN-curZ)**2
                avgErrDir = np.sqrt(sqErr/3)

                if verbose: 
                    print("SUMMARY:\n")
                    print(f"- x: {curX} / wanted {xN}, y: {curY} / wanted {yN}, z: {curZ} / wanted {zN}\n")
                    print(f"- Summed Squared Error is {sqErr} and the average absolute error per axis is {avgErrDir}\n")
                    print("- Solving has no notion of iterations.\n ")
                    newTime = time.perf_counter()
                    print(f"- Delta time is {1000*(newTime-startTime)} ms.")
                continue            

            mujoco.mj_step2(model, data)  
            viewer.sync() 

            # Perform additional updates to reach the desired joint positions, controlled by the input numStdUpdateSteps
            # Always perform at least 1 step, and may need to perform additional here
            for i in range(numStdUpdateSteps-1):
                mujoco.mj_step(model, data) 
                viewer.sync() 
            
            

# ###########################################################################################

'''
The high-level routine for the sphere movement simulation using two arms. 
- The simulation is loaded
- The arms will pass control to each other and repeatedly attempt to pass the sphere towards the
  other arm. 
'''
def MoveObjectRoutine():
    try:
        # Load model and data
        model = mujoco.MjModel.from_xml_path(Vars.DUAL_ARM_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"There was an error obtaining the model. Path may not exist or something else went wrong. \nThe error is {e}.")
        return
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Move the two arms to the home position
        MoveToLocationUnchecked(Vars.ARM1_HOME_X, Vars.ARM1_HOME_Y, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE, True)
        MoveToLocationUnchecked(Vars.ARM1_HOME_X, Vars.ARM2_OFFSET-Vars.ARM1_HOME_Y, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE, True, False)
        
        # The arms continuously trade off control and try to keep a sphere between them
        while(True):
            horizPos = data.site_xpos[2][0]
            if horizPos > Vars.SPHERE_CENTER_OFFSET/2:
                horizPos += 2.4*Vars.SPHERE_CENTER_OFFSET
            elif horizPos < -Vars.SPHERE_CENTER_OFFSET/2:
                horizPos -= 2.4*Vars.SPHERE_CENTER_OFFSET   

            # First arm

            # Move to corrected position to account possibility of the sphere moving outwards excessively 
            MoveToLocationUnchecked(horizPos, data.site_xpos[2][1]-Vars.SPHERE_CENTER_OFFSET, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE, True)

            # Move slightly inwards, likely pushing the ball closer to the centerline between the two arms. 
            MoveToLocationUnchecked(data.site_xpos[2][0] * Vars.SPHERE_HORIZ_OFFSET_MUL, data.site_xpos[2][1]-Vars.SPHERE_CENTER_OFFSET, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE, True)
            
            # Move to the home position
            MoveToLocationUnchecked(Vars.ARM1_HOME_X, Vars.ARM1_HOME_Y, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE, True)

            horizPos = data.site_xpos[2][0]
            if horizPos > Vars.SPHERE_CENTER_OFFSET/2:
                horizPos += 2.4*Vars.SPHERE_CENTER_OFFSET
            elif horizPos < -Vars.SPHERE_CENTER_OFFSET/2:
                horizPos -= 2.4*Vars.SPHERE_CENTER_OFFSET    
            
            # Second arm

            # Move to corrected position to account possibility of the sphere moving outwards excessively
            MoveToLocationUnchecked(horizPos, data.site_xpos[2][1]+Vars.SPHERE_CENTER_OFFSET, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE, True, False)
            
            # Move slightly inwards, likely pushing the ball closer to the centerline between the two arms.
            MoveToLocationUnchecked(data.site_xpos[2][0] * Vars.SPHERE_HORIZ_OFFSET_MUL, data.site_xpos[2][1]+Vars.SPHERE_CENTER_OFFSET, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE, True, False)
            
            # Move to the home position
            MoveToLocationUnchecked(Vars.ARM1_HOME_X, Vars.ARM2_OFFSET-Vars.ARM1_HOME_Y, Vars.ARM1_HOME_Z, model, data, viewer, Vars.JSOLVE, True, False)
    return


# ###########################################################################################
# ###########################################################################################
# ###########################################################################################


'''
Main entry point which handles base input

There are four options:
- A high-level control routine which prompts the user to input global positions to which the 
  robot arm end effector must move to. 
- Data is collected for training an inverse kinematics model
- A simulation where two robot arms move a sphere between them, taking turns, and ensuring that it
  does not move out of their reach. 
- A testing routine for the different methods that will report information for several motion traces
'''
def main ():
    print("Please enter a mode (1-4 inclusive):\n1. Run control\n2. Collect Data\n3. Move Sphere Between Arms\n4. Test Methods")
    choice = input()

    if (choice == Vars.RUN_CONTROL):
        # Loop continuously until obtaining valid input
        while (True):
            print("Please enter a solving method (1-7 inclusive):")
            print("1. Solve with transpose")
            print("2. Solve with raw pseudoinverse")
            print("3. Solve with safe pseudoinverse")
            print("4. Solve with gradient descent")
            print("5. Solve with gradient descent and singularity avoidance (learning rate adjustment)")
            print("6. Solve with gradient descent and singularity avoidance (matrix adjustment)")
            print("7. Solve with model (not recommended)")

            solverChoice = input()
            # If empty input will short-circuit access at index 0
            # Expecting 1, 2, 3, 4, 5, 6, or 7
            if (len(solverChoice) == 1 and ord(solverChoice[0]) >= Vars.ASCII_1 and ord(solverChoice[0]) <= Vars.ASCII_7):
                # Utilize sequential order of options
                print("Enter y to reset the arm after possible singularities and unreachable points are encountered, or any other input to not do so.")
                handleSingularities = input()
                mustHandle = (handleSingularities == "y")
                ControlRoutine(ord(solverChoice[0]) - Vars.ASCII_1 + 1, 1, True, 0.00008, mustHandle)   
                return
            # Input string was not correct
            print("Invalid input - please enter a digit 1-7, inclusive.\n")
    elif (choice == Vars.COLLECT_DATA):  
        CollectDataRoutine()
        return
    elif (choice == Vars.RUN_DUAL_ARMS):
        MoveObjectRoutine()
        return
    elif (choice == Vars.TEST):
        TestRoutine()
    else:  
        print("Invalid option - please enter a digit 1-4, inclusive. Terminating.")


# Run the main entry point
main()