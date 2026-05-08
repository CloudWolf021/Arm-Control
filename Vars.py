'''
A module for storing the general constants and the global variables for state tracking. 
'''


ARM_PATH = "Arm/franka_fr3/scene.xml"
DUAL_ARM_PATH = "Arm/franka_fr3/armsScene.xml"

# Coefficients a, b, c, and d for mapping x, y, z points into joint positions
J1_COEFF = [-0.6082788138793246, -0.1436887525703529, -0.5991220013395426, 0.384121173975713]
J2_COEFF = [0.9787398282038505, 0.394790181553633, -1.0759467792266664, 0.6689576691743827]
J3_COEFF = [0.8538476651322048, 0.5733402119070105, -0.15665339475152984, 0.16385553262952424]
J4_COEFF = [-0.1570388379908024, 0.4729924899167755, 1.3903324390249738, -2.324434089706862]
J5_COEFF = [0.8666970216948578, 0.6320634809996812, 0.6506290422606809, -0.9537519548580329]
J6_COEFF = [-0.7938367508375074, -0.5208225493558957, -1.05198114362534, 2.8706225798542824]
J7_COEFF = [2.0157677154681846, -0.43709335271725364, -2.015475928600283, 1.0147229521093233]

# Enumeration for the type of control used; used to prevent duplication

# Transpose
JT = 1
# Raw pseudoinverse
JPINV = 2
# Corrected Pseudoinverse
JPINVS = 3
# Gradient descent solving
JSOLVE = 4
# Special gradient descent that tries to recover from singularities by adjusting the learning rate and creating more instability in position
JSOLVELR = 5
# Special gradient descent that performs adjustments to the matrix being used (J^T*J)
JSOLVEM = 6
# Using fitted model
MODEL = 7

# Base mode enumeration
RUN_CONTROL = "1"
COLLECT_DATA = "2"
RUN_DUAL_ARMS = "3"

# BEGIN Iteration constants

# How many iterations the simulation will run for unless if it is terminated early
NUM_ITERS_COLLECT = 2400
NUM_ITERS_REG = 10800

# How many sequential failures to decrease the error by a threshold are needed until it is
# determined that the desired end effector position cannot be reached. 
# Important to have a high number to prevent false detections of unreachable points
NUM_ITERS_FAIL = 200

# The maximum legal iterations for a single call for the arm to move
TIMEOUT_ITERS = 1000

# Extra iterations to wait for the sphere to settle
EXTRA_WAIT_ITERS = 10

# Iterations to wait for the internal simulation to finish stepping in the case of using the model
MODEL_STEP_ITERS = 500

# END Iteration constants

# Threshold at which end effector position is considered to not have decreased sufficiently across
# 2 successive simulation iterations
IMPROVEMENT_FAIL_THRESHOLD = 0.00002

# Squared error must exceed this for the improvement failure to be checked, to prevent false 
# detections of unreachability when the arm is still converging to the target position. 
CHECK_IMPROVEMENT_FAIL_THRESHOLD = 0.025

EXPECTED_JOINT_NAME_LEN = 9
ASCII_1 = 49
ASCII_7 = 55

# DOF for a single arm
DOF = 7

# Total DOF for two arms and 1 free joint for the sphere
ADJ_DOF = 20

# Ordered limits extracted from the xml format: https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_fr3/fr3.xml
# Citation outside of document
JOINT_MIN_LIMITS = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]
JOINT_MAX_LIMITS = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]

# Positions for the initial movement of the first arm when a sphere is moved between the two arms
ARM1_HOME_X = -0.04
ARM1_HOME_Y = 0.31
ARM1_HOME_Z = 0.14

# Sphere and dual-arm related constants
ARM2_OFFSET = 1.5
SPHERE_CENTER_OFFSET = 0.14
# Tend to push the sphere closer back into the center - adjust outwards
SPHERE_HORIZ_OFFSET_MUL = 1.2

# END Constants

# ###########################################################################################

# BEGIN Globals

# Determine if the arm has stabilized at a location that is distant from the target point
SSQ_ERROR = -1
# Number of successive iterations with limited change
CUR_FAIL_ITERS = 0

# END Globals