import mujoco
import numpy as np
import math


import Vars

# ###########################################################################################

def FitLinearModel(x, y, z, out):
    iters = 400
    numPoints = len(x)
    learnRate = 0.00023
    # Regularize 
    #leanRate = learnRate# / numPoints
    a = 1
    b = 1
    c = 1
    d = 1

    for i in range (iters):
        err = 0
        gradA = 0
        gradB = 0
        gradC = 0
        gradD = 0
        for j in range(numPoints):
            delta = (a*x[j]+b*y[j]+c*z[j]+d-out[j])
            gradA += delta *x[j]
            gradB += delta *y[j]
            gradC += delta *z[j]
            gradD += delta
            #print(delta)
            err += delta**2
        #(err)
        # 
        a -= learnRate * gradA
        b -= learnRate * gradB
        c -= learnRate * gradC
        d -= learnRate * gradD
    print(f"J1_COEFF = [{a}, {b}, {c}, {d}]\n")
    #print(err)
    #print("\n")

# ###########################################################################################

# Use the fitted linear models
def GetRawJointPositionListModel(data, model, x, y, z):
    j1 = Vars.J1_COEFF[0]*x+Vars.J1_COEFF[1]*y+Vars.J1_COEFF[2]*z+Vars.J1_COEFF[3]
    j2 = Vars.J2_COEFF[0]*x+Vars.J2_COEFF[1]*y+Vars.J2_COEFF[2]*z+Vars.J2_COEFF[3]
    j3 = Vars.J3_COEFF[0]*x+Vars.J3_COEFF[1]*y+Vars.J3_COEFF[2]*z+Vars.J3_COEFF[3]
    j4 = Vars.J4_COEFF[0]*x+Vars.J4_COEFF[1]*y+Vars.J4_COEFF[2]*z+Vars.J4_COEFF[3]
    j5 = Vars.J5_COEFF[0]*x+Vars.J5_COEFF[1]*y+Vars.J5_COEFF[2]*z+Vars.J5_COEFF[3]
    j6 = Vars.J6_COEFF[0]*x+Vars.J6_COEFF[1]*y+Vars.J6_COEFF[2]*z+Vars.J6_COEFF[3]
    j7 = Vars.J7_COEFF[0]*x+Vars.J7_COEFF[1]*y+Vars.J7_COEFF[2]*z+Vars.J7_COEFF[3]
    return [j1, j2, j3, j4, j5, j6, j7]