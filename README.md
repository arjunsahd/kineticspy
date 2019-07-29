# simulpy
Simulation tool for robotics

Robotics is a very vast branch of engineering involoving computer, mechanical and electrical engineering.

While there are many libraries available online for robotics. They are not that easy to use for beginners.
To create interest in robotics we need to provide easier visualisation platforms so that the interest can
nurture.

The aim of this project is to do exactly that. By using plotly we are providing a 3D toggle to visualize 
the trajectory and orientation of the robot.

As currently plotly's 3D animation toolbox is still under development we are limited to matplotlib for
animation of the robot.
All the contributions are welcome!!
I have not yet documented anything will do that in about a months time.

If any doubts mail me or raise an issue.

Currently I am working on animation through trajectories
And the library is not very well oriented there are redundancies which can be removed
Will work on that.

Current func:
1. Create Robot object
2. Forward Dynamics
3. Trajectory ploting for continuously varying joints
4. Inverse Kinmatics 
5. Minimum Trajectory Potting i.e straight line

Current Goals:
1. add animation
2. Make the library suitable to be used using DH paramaeters
 
PS: I am new to open source XD

Note: Functions using inverse kinematics functions are only available for 3DOF arm.
