# **Comparing Inverse Kinematics Methods for Robotic Arm Control**

## **1 Abstract**

## **2 Background and Motivation**

Fundamentally, the tasks a robotic arm must complete involve the positions of the end effector in 3-dimensional space. In most applications, a set of desired end effector orientations will likely be supplied too. For simplicity, we only consider absolute end effector position, and leave orientation as an extension for future work - in particular, the methods we investigated will still hold when orientation is handled. Also, by only considering position, there is more redundancy: 7 degrees of freedom are used to reach a location given as (x, y, z). This arm is more agile and capable in comparison to arms with fewer joints, though this can come at the expense of encountering additional numerical issues. 

The high-level problem becomes transforming a desired position in physical 3-dimensional space to a set of joint positions (in our case, 7, since we use a robotic arm with 7 degrees of freedom) through inverse kinematics. There are numerous established methods for this, this is a widely studied and surprisingly difficult numerical problem. 

We seek to implement these strategies, and extend them to attempt to correct for singularities and poor numerical behaviors, and compare their performance. With the availability of many methods, it is valuable to see which is most optimal for a general application so that they do not all need to be tested. 


## **3 Platform and High-Level Organization**

### **3.1 Platform:**

We use the open-source MuJoCo (Google DeepMind) environment for simulating the robotic arm and readily obtaining the Jacobian. Though a physical simulation is not strictly necessary, it is useful for observing the general sequence of joint positions taken when converging to a final solution. In practical applications, the joint positions are computed and then the robot will move to them, and the solving trajectory is not seen. However, it can be useful to visually assess a trajectory for instability and oscillations. 

We use an XML model of the Franka FR3 arm with 7 degrees of freedom, constructed by Google Menagerie using available specifications from Franka. This is a fairly generic arm with 7 degrees of freedom and position redundancy.

![*Alt: The Franka FR3 arm*](Graphics/arm.png)

**Figure 3.1: The Franka FR3 arm after being instructed to move to (0.4, 0.4, 0.4)**

### **3.2 Organization:**
On a high-level, there are four important program routines:

- **A control routine:** The control routine mimics the general manner an arm would be used. It is generic, and can utilize any of the 8 solving methods, and repeatedly prompts the user for positions that the arm must move to. This also enables the user to specify whether the arm should return to its home position after each failed request to help prevent convergence problems. 
- **A data collection routine:** The data collection routine randomly moves the arm and collects joint and end effector positions to fit a set of linear models for mapping 3-d positions to joint angles. 
- **A routine for having 2 robot arms pass a sphere between each other:** This routine is included as an application to verify the performance of the basic gradient descent method, and use it in a cooperative setting. 
- **A method testing routine:** The testing routine runs the 8 core methods on a set of position traces, and reports statistics for each. 

### **3.3 Modularity and Extensibility:**

Numerous Python modules are used to separate different project components and promote code reuse. It is simple to add additional inverse kinematics methods, and the core solving functionality is placed in modules. 

## **4 Basic Solving Methods**

We note four methods - three valid, Jacobian-based solvers, and an experimental, model-based approach.

### **4.1 Jacobian-Based Methods**

The classical inverse kinematics methods appear to be generally based on using the Jacobian, and performing joint position updates until the end effector reaches the target. This is assessed by computing the deviation, and determining if it is within a small threshold. 

### **4.1.1 Using the Jacobian for Arm Movement**

At each iteration, we attempt to solve $J*\vec{dj} = \vec{dx}$ for $\vec{dj}$, where $J$ is the Jacobian, $\vec{dj}$ is the vector of joint position updates, and $\vec{dx}$ is the current position error. Specifically, $\vec{dx} = \vec{target} - \vec{cur}$, where $\vec{target}$ is the desired position in 3-d space, and $\vec{cur}$ is the current end effector position. 

This is equivalent to solving a linear equation that may not have any solutions or may have infinitely many solutions. 

The Jacobian measures the derivative of the x, y, and z with respect to each joint position, as well as the change in angles.

Thus, we have a link between the deviation from desired output end effector positions and viable updates in the joint positions to reduce end effector position error. 

A limitation of this project is that we must actually step the simulation to obtain the Jacobian, since we do not manually compute it. 

Then, on each iteration, the problem 

The core problem is .....
Iterate...

#### **4.1.1.1 Unreachable Position Detection**

When running the raw stepping method using the Jacobian, it is possible that the end effector position error will never be within the desired threshold. Thus, we can implement a feature to check the rate of change in position error. Once it falls below some threshold, we assume that the arm will not reach the target, and thus inform the user and prompt for a new arm position. 

At each iteration, we assess the proportional change in squared error from the previous iteration, relative to the current error. If this proportion is less than a small constant, then the number of consecutive improvement failure iterations is incremented, and otherwise this count is reset. If this count reaches a specific number of iterations (200, though this can be tuned further), then a position has been determined to be unreachable since the arm is making minimal proportional progress towards the solution. 


Previously, the unreachability detection algorithm simply assessed the change in error relative to a threshold, leading to poor detection due to a very low threshold, to prevent excessive false classifications. With the current algorithm, when overall error is small, a smaller error change is needed to trigger a detected failure, reducing misclassifications that occur when the arm will actually reach the target position, as desired. 

#### **4.1.1.1 Iterations and Simulator Use**


@@@@@@@@@@@@@@@@@@

Having multiple iterations of the simulation at each step led to higher numbers of total motion iterations needed, since computing update joint positions more times leads to less deviation in the Jacobian matrix. 

4/56
3/74
2/110
1/218

@@ Note in extension -> this constraint comes from the framework. 


### **4.1.2 Method 1: Jacobian Transpose**

As noted by Buss and Kim (2009), 

@@ Write

### **4.1.3 Method 2: Jacobian Pseudoinverse**

In particular, the Jacobian does not need to be square, and no true inverse likely exists. Furthermore, even if it is square, it is possible that it is non-invertible due to a nontrivial null space.

@@ Write

### **4.1.3 Method 3: Jacobian Gradient Descent**

When using pure gradient descent, 

@@ Write; pseudocode

#### **4.1.3.1 Fine-Tuning Gradient Descent Parameters**

We consider the case of moving from the initial, default upwards arm orientation to the (x, y, z) position of (0.4, 0.4, 0.4), using a position threshold of 0.00008. As a note, our later, in-depth analysis is performed with a threshold of 0.0008 for more tolerance. 



 First, we optimize the number of optimization steps for each usage of gradient descent.

To minimize overall computational work, we minimize the total number of steps, which is the product of the number of outer iterations and the inner gradient descent iterations. We observe that from the sampled values, the optimal inner step count is 60, at which there are 76800 total steps. 


![*Alt: Total Iterations as a Function of Inner Iteration Count*](Graphics/innerIterTuning.png)

**Figure 4.1: Total steps as a function of inner gradient descent iterations**

Using an internal step count of 60, we then optimize the learning rate by observing the total number of outer iterations needed. We then find that with a learning rate of 0.89, there are 250 steps, corresponding to an approximate minimum. This is a significant improvement in comparison to the nearly 1500 steps used when the learning rate is under 0.1. 


@@@@@@@@@

![*Alt: Iteration Count as a Function of Learning Rate*](Graphics/learnTuning.png)

**Figure 4.2: Outer gradient descent iterations as a function of learning rate**

Thus, we use an internal step count of 60 and a learning rate of 0.89 for optimal inverse kinematics solution convergence. These carefully tuned parameters may explain, at least in part, the high efficacy of the pure gradient-descent based approach, as seen in Chapter 6. 

### **4.2 Method 4: Model-Based Solving**

@@@@@@@@@@ Finish

As an experiment, we consider a model for mapping 3-d coordinates to joint positions. 

This approach can be improved in the future, and this is detailed in Section 8.1. 

## **5 Additional Solving Methods and Interventions**

We consider several additional methods and interventions to help avoid handle singularities. 

### **5.1 Method 5: Modified Pseudoinverse**

It is often undesirable to have matrices with small determinants. This can be associated with instability and rows or matrix columns that are close to multiples of each other, even if the rank is full. Thus, as an intervention, we adjust the matrix $J^T*J$ and manually compute the pseudoinverse instead of using numpy's linalg.pinv method. 

@@@ Expand; pseudocode


### **5.2 Method 6: Modified Gradient Descent: Learning Rate Adjustment**

Using the algorithm to detect potential singularities, on each iteration, if we detect a proportionally low decrease in error, we increase the learning rate from 0.89 to 1.8. This is motivated by the idea that the adjustment could help the gradient descent algorithm escape from a region that will not allow for the target position to be reached. 

We also attempted to increase the multiplier for the joint position updates instead of this, but found that instability increased excessively for some reachable points, and that this was not always sufficient as a singularity avoidance method for other locations. 

### **5.3 Method 7: Modified Gradient Descent: Matrix Adjustment**

The general equation to solve is $J*\vec{dj} = \vec{dx}$, as noted earlier. As inspired by the Levenberg-Marquardt algorithm, we can consider $J^T*J$ and perform corrections to this, as in method 5. 

We transform $J*\vec{dj} = \vec{dx}$ into $(J^T*J)*\vec{dj} = J^T*\vec{dx}$ by left-multiplying by $J^T$, and modify $(J^T*J)$ into $adj$ to ensure that it meets the determinant threshold. This perturbation is expected to help prevent numerical instability and can help potentially escape singularities.  

Then, we use gradient descent with a fixed number of steps to solve $adj*\vec{dj} = J^T*\vec{dx}$ for $\vec{dj}$ by minimizing squared error. At each step, we compute the gradient with respect to $\vec{dj}$, and perform an update.   

Other matrix adjustment schemes were also attempted. For example, the update rule $new = (old+k_1)*k_2$ for constants $k_1$ and $k_2$ was for main diagonal entries was used. However, when running an extensive test over 102 target positions, there were 44 false timeouts (exceeding the maximum iteration count when a position is actually reachable) as opposed to 30 false timeouts. Even though this method appeared to reduce convergence iterations in simple cases, it actually led to poorer overall performance. 

### **5.4 Method 8: Modified Gradient Descent: Gradient Adjustment**

@@ Write

### **5.5 Home Position Intervention**

In the main control routine, we allow for the arm to return to a home position after each failed motion attempt. Most failed cases arise from the arm being in strange orientations after attempting to move to an unreachable position. Thus, as an intervention, returning to a safer position can lead to higher overall efficacy for movement requests that follow an unreachable position. The position the arm moves to allows for reliable motion to reachable points, and based on tests singularities are generally not encountered. 

As a note, this is not considered to be a specific method - instead it is is a fix applied at the end of each failed motion request.

![*Alt: The Franka FR3 arm in a mostly neutral position*](Graphics/armUp.png)

**Figure 5.1: The Franka FR3 arm in a mostly neutral position**



## **6 Analyzing The Inverse Kinematics Methods**

### **6.1 Testing Framework, In-Depth**

To assess the methods, we use the fourth high-level program option for testing. We use 10 different traces with varying purpose for an overall investigation of the 8 different methods. 

Each method is used separately with each trace, with the option to reinitialize the simulation before each position request in the trace. 

The statistics we obtain for each method, per trace, are elapsed time, total outer iterations, success count, timeout count, number of points detected as unreachable, the average time and iterations to determine unreachability, and the number of false timeouts and unreachability detections. 

It is important to note that for the final, most exhaustive trace, we do not manually determine the reachability of each point. Instead, we rely on the algorithms to determine if a position is reachable. If at least one method leads to reaching the target position, then it is reachable, and otherwise it is unreachable. It is possible that this approach falsely misclassifies some positions as unreachable, but due to the varied behaviors of the algorithms and start from the home position before each iteration, we assume that this is not a problem, or will not substantially alter the results. 

### **6.1.1 Traces**

- **Trace 1** assesses basic functionality: moving to the valid point (0.4, 0.4, 0.4). We also investigate the solving trajectory taken by a selected portion of the methods for this trace. 
- **Trace 2** tests a sequence of 3 valid positions without resetting the arm
- **Trace 3** tests a sequence of 7 valid positions, without resetting the arm
- **Trace 4** tests sending the arm to the unreachable position (1, 1, 1), which is not
  far removed from the range of the arm. 
- **Traces 5-9** test sequences of mixed valid positions and unreachable positions, including ones that are very far from the arm (including (-90, -90, 90)). These positions can be filtered out, but they are used to test the robustness of the algorithms due to the potentially undesirable ending positions (based on singularities and end effector location relative to the next target point). For these traces, the arm is not reset after unreachable points. 
- **Trace 10** is the core trace with the largest sample size. The arm is reset after each request, where x and y can be -0.7, -0.35, 0, 0.35, or 0.7, and z can be 0, 0.2, 0.4, 0.6, 0.8, or 1. Points that have a sum of their squared components exceeding 1.1 are not considered, since it is expected that they are out of bounds based on observations of position reachability. Regardless, 102 points are tested on each method, and there is a mix of reachable and unreachable points. For example, the position (0.7, 0.7, 0) is included, and it is not reachable. Overall, this is the most complete trace for testing the methods.  

 

### **6.2 Method Assessment and Results**

**Trace 1:** All methods except the model pass this trace. 

![*Alt: Trace 1 statistics*](Graphics/trace1.png)

**Figure 6.1: Trace 1 statistics**

The raw pseudoinverse method has the best runtime and the second-lowest iteration count. Though gradient descent with a gradient modification leads to 1 less outer computation iteration, it is almost 9 times slower. The two methods that adjust $J^T*J$ are over 60 times slower than the fastest valid method, with no benefit in this case. This is expected since the simplest methods should be the fastest, and still converge well for a simple case. 

As expected, the linear model fails to yield a valid solution. In fact, the average error in the x, y, and z directions is approximately 0.17 (each target position is 0.4). 

@@@@@@@@@@@@@

<video controls width="250">
    <source src="/Graphics/pinv.mp4" type="video/mp4">
</video>

@@@@@@@

[![Inverse Kinematics with the Jacobian transpose](./Graphics/transpose.mp4)](./Graphics/transpose.mp4)

@@@@@@@@@@

![*Alt: Inverse Kinematics with the Jacobian transpose*](Graphics/transpose.mp4)

**Figure 6.2: Inverse Kinematics with the Jacobian Transpose**


@@@@@@@@





![*Alt: Inverse Kinematics with the Jacobian pseudoinverse*](Graphics/pinv.mp4)

**Figure 6.3: Inverse Kinematics with the Jacobian pseudoinverse**

![*Alt: Inverse Kinematics with the modified Jacobian pseudoinverse*](Graphics/pinv2.mp4)

**Figure 6.4: Inverse Kinematics with the modified Jacobian pseudoinverse**

![*Alt: Inverse Kinematics with Gradient Descent and the Jacobian*](Graphics/gradientDescent.mp4)

**Figure 6.5: Inverse Kinematics with Gradient Descent and the Jacobian**

![*Alt: Inverse Kinematics with Gradient Descent and a matrix adjustment*](Graphics/gradientDescentMatrixAdj.mp4)

**Figure 6.6: Inverse Kinematics with Gradient Descent and a matrix adjustment**

![*Alt: Inverse Kinematics with Gradient Descent and a gradient adjustment*](Graphics/gradientDescentGradientAdj.mp4)

**Figure 6.7: Inverse Kinematics with Gradient Descent and a gradient adjustment**

-----------------

Using the pseudoinverse, pure gradient descent, or gradient descent with a modified gradient lead to the most direct trajectories. The gradient descent modification leads to faster convergence close to the solution, and has slightly better performance in comparison to regular gradient descent. 

**Trace 2:** All methods except the model pass this trace with 3 chained, valid position commands. As in the previous case, the pseudoinverse is the fastest at 73 ms and 640 iterations, and the gradient descent with a modified gradient has 633 iterations but a runtime of 663 ms.

------------------------

**Trace 3:** This longer trace of 7 reachable positions leads to the transpose and gradient descent with a modified learning rate failing on 1 position each. 

![*Alt: Trace 3 statistics*](Graphics/trace3.png)

**Figure 6.8: Trace 3 statistics**

Based on this trace, it becomes clear that the raw pseudoinverse approach leads to the best performance - it reaches all positions and runs almost 10 times as fast as the second-fastest method that reaches all targets (gradient descent). Though the transpose is faster, it is clear that it is less precise as an approximator for the joint position updates. 

--------------------------

**Trace 4:** The target position (1, 1, 1) is unreachable, and trivially no methods allow the arm to reach it. However, the modified Jacobian pseudoinverse method and the transpose method detect that the point is not reachable, and report this in approximately 700 iterations. This is beneficial over a timeout since less time is used and there is a stronger prediction that the position is actually unreachable (when not considering other knowledge about reachable positions).

--------------------------

**Traces 5-9:**

![*Alt: Traces 5-9 statistics*](Graphics/traces5_9.png)

**Figure 6.9: Traces 5-9 statistics**

It is clear that the basic pseudoinverse method continues to perform the best. Furthermore, the transpose method is clearly the most ineffective at handling singularities (excluding the model). However, the gradient descent methods with gradient correction and learning rate adjustment had 6 successes, which is better than the base gradient descent method. This indicates that in some cases, the interventions can be beneficial. Also of note is that the gradient term intervention method performed the best on trace 7, and that out of 10 reachable positions, the maximum reached was 8. This indicates that none of the methods universally work, and that some will be more successful in specific cases. 

-------------------------

**Trace 10:**

Finally, we consider the result of the methods on the most extensive test set. 


![*Alt: Trace 10 statistics*](Graphics/trace10.png)

**Figure 6.10: Trace 10 statistics**

@@@

----------------

Thus, we conclude that the pure pseudoinverse method has the best general performance. 

## **7 Application: 2 Robotic Arms Pass a Sphere Between Each Other**

To test the inverse kinematics methods and experiment with cooperative arms, we consider a secondary setup. This utilizes two identical Franka FR3 robotic arms, and a sphere in-between them. The key task is for the two arms to properly move the sphere so that it does not move out of their reach, and for each robot to pass the ball to the other. 

Initially, the end effector of the arms was instructed to go to the center of the sphere. This would generate excessive contact forces, and the ball would move to quickly. As a result, an offset was added so that the end effector moves near the sphere, but is not attempting to move into the sphere. 

Afterwards, several iterations were needed until the ball would remain in reach of the two arms. In particular, when the sphere is deviating significantly laterally, the arms must attempt to push it back towards the plane between them (determined by $x = 0$). 

![*Alt: Arms passing a sphere between each other*](Graphics/dualArms.mp4)

**Figure 7.1: Sphere Passing Routine**


## **8 Future Work**

### **8.1 Linear Model Improvements**

The current model used for inverse kinematics works very poorly (in trace 10, only 1 success for 79 different reachable positions). It only considers linear relationships, and performs multiple linear regression separately for the different joint positions. This cannot accurately learn the true kinematics of the arm, and serves as a test. In the future, a single, larger nonlinear model, or multiple local models can be made for this. Subdividing the 3-d space into segments and making models for each one should lead to a better understanding of the kinematics. As a further extension, the model could be used for obtaining an approximate set of joint positions, which are used as the starting point for gradient descent or one of the other iterative methods that were investigated. 

### **8.2 Method Combinations**

Multiple methods can be combined to yield faster convergence. For example, the transpose operation is numerically safe, but leads to poorer overall performance. Thus, the joint update vector outputted by this method could be used as the starting point for gradient descent, for better runtime and potentially more robustness. 

### **8.3 Considering Arm Orientation**

Extending the state inputs so that the orientation of the arm is also considered would be more universally applicable. This will involve using the angular portion of the Jacobian and also taking three additional inputs that determine end effector orientation. In practice, this is quite useful since the end effector of an arm will often need to be in some particular orientation once it reaches the target (x, y, z) point.

### **8.4 Better Singularity Detection**

Currently, it is somewhat difficult to detect when the arm has encountered a true singularity and will likely not reach its target. The algorithm used that investigates consecutive change in error proportional to absolute square error can lead to false detections of unreachability in the case that a singularity is encountered, and a position is actually reachable. Fitting a classification model based on square error and change in squared error could lead to better detection, and allow for more aggressive adjustments to when needed. Currently, substantial adjustments in cases where there are no singularity-related issues can lead to commands to easily reachable positions failing. 

### **8.5 Better Unreachable Position Detection**

Finally, filtering out unreachable positions without attempting to move to them could reduce the numerical issues we encounter. Training another classification model to determine whether (x, y, z) points are reachable could be needed since the reachable point cloud will likely not have a clean form for the Franka FR3 arm. 


## **9 Conclusion**

We have found that the core inverse kinematics solving methods work relatively well in most cases. However, oscillations and poor convergence can still occur. Numerous interventions were attempted, but as a whole they appear to actually hurt performance. In certain failure cases, they are clearly beneficial, but in the most extensive trace, using pure gradient descent and an unmodified pseudoinverse led to the best performance performance in terms of correctness. However, the pseudoinverse algorithm is almost 10 times faster than gradient descent, and as a whole appears to be the best method, which is somewhat unexpected.   

There are many algorithmic improvements that can be made to improve the accuracy of inverse kinematics solving. A core idea will likely be combining multiple approaches and making use of trained neural networks to help with unreachability and singularity detection. This combined with the basic methods should lead to both correctness and runtime gains. 

## **10 Acknowledgements**

Thank you to Professor Chris Atkeson and Henry Liao for the project advice and change towards assessing and applying different inverse kinematics methods, with a focus on singularities.


## **11 Sources**

1. Buss and Kim. https://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf (2009). This paper introduction gives more context for using the Jacobian to iteratively step joint positions to converge to the target position. 

2. https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm. This is an inspiration for one of the intervention methods.

3. https://developer.mozilla.org/en-US/docs/Web/HTML/Reference/Elements/video. This is useful for embedding videos in markdown using HTML elements. 

 *Note*: Other helper sources are listed throughout the code, and are not listed here since they are not directly related to the core analysis logic. 



