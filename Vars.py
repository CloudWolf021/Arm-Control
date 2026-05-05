ARM_PATH = "Arm/franka_fr3/scene.xml"

# Coefficients a, b, c, and d for mapping x, y, z points into joint positions
J1_COEFF = [-0.6082788138793246, -0.1436887525703529, -0.5991220013395426, 0.384121173975713]
J2_COEFF = [0.9787398282038505, 0.394790181553633, -1.0759467792266664, 0.6689576691743827]
J3_COEFF = [0.8538476651322048, 0.5733402119070105, -0.15665339475152984, 0.16385553262952424]
J4_COEFF = [-0.1570388379908024, 0.4729924899167755, 1.3903324390249738, -2.324434089706862]
J5_COEFF = [0.8666970216948578, 0.6320634809996812, 0.6506290422606809, -0.9537519548580329]
J6_COEFF = [-0.7938367508375074, -0.5208225493558957, -1.05198114362534, 2.8706225798542824]
J7_COEFF = [2.0157677154681846, -0.43709335271725364, -2.015475928600283, 1.0147229521093233]

# Effective enumeration for the type of control used; used to prevent duplication
JT = 1
JPINV = 2
JPINVS = 3
JSOLVE = 4
MODEL = 5

# How many iterations the simulation will run for unless if it is terminated early
NUM_ITERS_COLLECT = 2400
NUM_ITERS_REG = 10800

# How many sequential failures to decrease the error by a threshold are needed until it is
# determined that the desired end effector position cannot be reached. 
# Important to have a high number to prevent false detections of unreachable points
NUM_ITERS_FAIL = 200

# The maximum legal iterations for a single call for the arm to move
TIMEOUT_ITERS = 2000

# Threshold at which end effector position is considered to not have decreased sufficiently across
# 2 successive simulation iterations
IMPROVEMENT_FAIL_THRESHOLD = 0.000005

EXPECTED_JOINT_NAME_LEN = 9
ASCII_1 = 49
ASCII_5 = 53
ASCII_7 = 55

DOF = 7
# Ordered limits extracted from the xml format: https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_fr3/fr3.xml
# Citation outside of document
JOINT_MIN_LIMITS = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]
JOINT_MAX_LIMITS = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]

# END Constants

# BEGIN Globals

# Determine if the arm has stabilized at a location that is distant from the target point
SSQ_ERROR = -1
CUR_FAIL_ITERS = 0

# END Globals