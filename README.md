# **Inverse Kinematics for Robotic Arm Control**

## **Abstract**

## **Background and Motivation**

The user of a robotic arm does not reason in terms of the positions each of the joints will need. Instead, the user will have a desired position in 3d-space, and likely an orientation. For simplicity, we only consider raw end effector position, and leave orientation as an extension for future work. 

Furthermore, with higher degrees of freedom, robotic arms are more agile, and capable, though this can come at the expense of having multiple valid arm orientations and encountering additional numerical issues. 

Thus, the high-level problem becomes transforming a desired position in physical 3-dimensional space to a set of joint positions (in our case, 7, since we use a robotic arm with 7 degrees of freedom). These joint positions can be then used to control the arm. There are numerous established methods for this, though extensions are needed to properly handle singularities and cases where the desired end effector position is not obtainable. 

To control the robotic arm in this s

We seek to compare these different methods under different requests, and expand upon them to detect singularities and avoid them to prevent numerical instability. 

## **Platform**

We use Google DeepMind's open-source MuJoCo environment for simulating the robotic arm and readily obtaining the Jacobian. Though a physical simulation is not strictly necessary, it is useful for observing the general sequence of joint positions taken when converging to a final solution. This also allows us to physically see undesirable oscillations

We use an XML model of the Franka FR3 arm with 7 degrees of freedom, constructed by Google Menagerie using available specifications from Franka. This is a fairly generic arm with position redundancy, and creates additional numerical challenges, as desired. 

## **Fundamental Solving Methods**

We note four methods - three of which are valid, and one of which is experimental.

- Numerical iteration using the Jacobian and its transpose 


- Numerical iteration using the Jacobian and its pseudoinverse - in particular, the Jacobian does not need to be square, and no true inverse likely exists. Furthermore, even if it is square, it is possible that it is non-invertible due to a nontrivial null space. 

- Nested numerical iteration using gradient descent to move towards a least-squares solution. 


## **Methods: Analysis Framework**

## **Methods: Testing Simple Methods**

### **The Effect of Gradient Descent Parameters**

We consider the case of moving from the initial, default upwards arm orientation to the (x, y, z) position of (0.4, 0.4, 0.4), using a position threshold of 0.00008.

The overall logic is a multi-level numerical problem. Internally, at each iteration, a rough update for the joint positions is obtained, and this routine is performed until the end effector is within the threshold of the target position. 

 First, we optimize the number of gradient descent iterations. 

There is a clear tradeoff between the number of iterations in the gradient descent loop at each outer step and the number of outer full position update steps that are run. However, to minimize overall computational work, we minimize the total number of steps, which is the product of the number of outer iterations and inner iterations. We observe that from the sampled values, the optimal inner step count is 60, with 76800 total steps. An intuitive explanation for this is that with more than 60 steps, the benefit from the better solution is not as significant, and that with less steps, there is not sufficient convergence towards a least-square solution for the desired joint angle updates. 

![Total Iterations as a Function of Inner Iteration Count](Graphics/innerIterTuning.png)

Then, we expect that this result can translate to the use of other learning rates, and using 60 as the internal step count, we optimize the learning rate by observing the total number of external iterations needed. We then find that the minimum number of outer iterations needed is 250, with a learning rate of 0.89, which is substantially higher than the initial rate that was used. This indicates that the learning rate generally has a range of suitable values, and increasing it until some level leads to faster convergence without leading to instability.  

![Iteration Count as a Function of Learning Rate](Graphics/learnTuning.png)

Thus, we use an internal step count of 60 and a learning rate of 0.89, which appears to lead to the fastest overall solution.  

## **High-Level Framework**

Often times, it is known where the end effector should be at particular points in time. Thus, we would like the high-level control to move the arm between user-entered positions, and hold the final state until the next command, once a specific error threshold has been achieved. 

## **Assessment of Basic Methods**

To assess the basic methods, we use a set of test routes between points in 3-dimensional space. We observe whether they successfully converge, how many total steps are used, and how smooth the algorithm until convergence. These methods do not account for singularities, and it is possible for them to fail.  

## **Using the Jacobian for Arm Movement**


The Jacobian is a very important matrix that measures the derivative of the x-y-z components of the end effector with respect to each joint position, as well as the change in angles.

Thus, we have a link between the deviation from desired output end effector positions and viable updates in the joint positions to reduce end effector position error. 

## **Basic Method Results:**

Finally, we compare the three core methods based on the framework we noted. 

### **Detecting Unreachable Points**

When running the raw stepping method using the Jacobian, it is possible that the end effector position error will never be within the desired threshold. Thus, we can implement a feature to check the rate of change in position error. Once it falls below some threshold, we assume that the arm will not reach the target, and thus inform the user and prompt for a new arm position. 

### **Singularity Corrections for the Pseudoinverse**



## **Application: Moving Cube Between 2 Robotic Arms**

## **Future Work**

- It would be interesting to expand the scope of a model that attempts to convert input x-y-z coordinates into 7 joint positions. Adding data should allow the model to better learn the dynamics of the system.

- The MuJoCo simulator will properly prevent an arm from moving below the plane of the ground. However, in conventional applications it is rarely desirable for the arm to move while it is in contact with the ground. Thus, it could be wise to plan out better update trajectories in comparison to simply updating using the Jacobian. 

- Extending the state inputs so that the orientation of the arm is also considered would be more universally applicable. This will involve using the angular portion of the Jacobian and also taking three additional inputs that determine end effector orientation. In practice, this is quite useful since the end effector of an arm will need to be in some particular orientation once it reaches the target (x, y, z) point.

## **Conclusion**

We have found that the core inverse kinematics solving methods work relatively well in cases where no singularities emerge. However, oscillations and poor convergence can occur near singular points, though this can be partially remedied by applying corrections to the Jacobian Matrix. 

## **Acknowledgements**

Thank you to Professor Chris Atkeson and Henry Liao for the advice and help to change the overall trajectory of this project. 


## Sources

1. Buss and Kim. https://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf (2009). This paper introduction gives more context for using the Jacobian to iteratively step joint positions to converge to the target position. 



