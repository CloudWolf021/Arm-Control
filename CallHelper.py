import Vars
import JacobianIterSolve
import JacobianGradSolve
import Helpers

'''
A module to encapsulate the common logic pattern of branching depending on what control method should be used.
This method is in its own module to prevent circular module dependencies. 
'''


'''
Handle the branching logic depending on what control type or arm is desired.
'''
def ControlBranching(controlType, model, data, x, y, z, jacobian, isFirstArm):
    if (controlType == Vars.JT):
        Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianT(data, model, x, y, z, jacobian, isFirstArm), False, isFirstArm)
    elif (controlType == Vars.JPINV):
        Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInv(data, model, x, y, z, jacobian, isFirstArm), False, isFirstArm)
    elif (controlType == Vars.JPINVS):
        Helpers.MoveToJointPositionsRaw(data, JacobianIterSolve.GetRawJointPositionListJacobianPInvSpecial(data, model, x, y, z, jacobian, isFirstArm), False, isFirstArm)
    elif (controlType == Vars.JSOLVE):
        Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacobian, False, False, False, isFirstArm), False, isFirstArm)
    elif (controlType == Vars.JSOLVELR):
        # Parameter for handling singularities is True
        Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacobian, True, False, False, isFirstArm), False, isFirstArm)
    elif (controlType == Vars.JSOLVEM):
        # Parameter for adjusting matrix is True
        Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacobian, False, True, False, isFirstArm), False, isFirstArm)
    elif (controlType == Vars.JSOLVEG):
        # Parameter for adjusting gradient is True
        Helpers.MoveToJointPositionsRaw(data, JacobianGradSolve.GetRawJointPositionListJacobianSolve(data, model, x, y, z, jacobian, False, False, True, isFirstArm), False, isFirstArm)
