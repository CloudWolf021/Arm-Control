# **Inverse Kinematics for Robotic Arm Control**

## **Abstract**

## **Introduction**

## **Background**

## **Platform**

We use Google DeepMind's open-source MuJoCo environment for simulating the robotic arm and obtaining relevant Inverse Kinematics Information. In comparison to other platforms such as ROS2 with the Gazebo environment, MuJoCo is simpler to use and provides convenient built-in functionality. 

For simplicity, we use a XML of the Franka FR3 arm with 7 degrees of freedom, constructed by Google Menagerie using available specifications. This arm is generic and has position redundancy, creating additional algorithmic challenges. 

## **High-Level Framework**

Often times, it is known where the end effector should be at particular points in time. Thus, we would like the high-level control to move the arm between user-entered positions, and hold the final state until the next command, once a specific error threshold has been achieved. 

## **Methods**

## **Using the Jacobian for Arm Movement**


The Jacobian is a very important matrix that measures the derivative of the x-y-z components of the end effector with respect to each joint position, as well as the change in angles.

Thus, we have a link between the deviation from desired output end effector positions and viable updates in the joint positions to reduce end effector position error. 

### **Improving Robustness**

When running the raw stepping method using the Jacobian, it is possible that the end effector position error will never be within the desired threshold. Thus, we can implement a feature to check the rate of change in position error. Once it falls below some threshold, we assume that the arm will not reach the target, and thus inform the user and prompt for a new arm position. 

This should be fairly simple to implement by either using a global variable or maintaining the previous squared error to get a rate of change or raw change across successive iterations. 

## **Future Work**

- It would be interesting to expand the scope of a model that attempts to convert input x-y-z coordinates into 7 joint positions. Adding data should allow the model to better learn the dynamics of the system.

- The MuJoCo simulator will properly prevent an arm from moving below the plane of the ground. However, in conventional applications it is rarely desirable for the arm to move while it is in contact with the ground. Thus, it could be wise to plan out better update trajectories in comparison to simply updating using the Jacobian. 

- Extend the state inputs so that the orientation of the arm is also considered. This will involve using the angular portion of the Jacobian. In practice, this is quite useful since the end effector of an arm will need to be in some particular orientation once it reaches the target (x, y, z) point.

## **Conclusion**




## Sources

1. Buss and Kim. https://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf (2009). This paper introduction gives more context for using the Jacobian to iteratively step joint positions to converge to the target position. 



