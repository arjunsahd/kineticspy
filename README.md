# kineticspy
Simulation tool for robotics
Open nbviewer, if graphics are not loaded on the webpage. This is because of the size of graphics is too large to upload.

## Installation
pip install kineticspy
## About
Robotics is a very vast branch of engineering involoving computer, mechanical and electrical engineering.

While there are many libraries available online for robotics. They are not that easy to use for beginners.
To create interest in robotics we need to provide easier visualisation platforms so that the interest can
nurture.

The aim of this project is to do exactly that. By using plotly I am providing a 3D toggle to visualize 
the trajectory and orientation of the robo. Plus analyse the kinetics of the robot. EVerything is very easy and systematic to use.

As currently plotly's 3D animation toolbox is still under development we are limited to matplotlib for
animation of the robot.
Any contribution is welcome!!

And the library is not at par with its aim. Currently it has only been tested on 3R robotics arm.

Current func:
1. Create Robot object
2. Forward Dynamics
3. Trajectory ploting for continuously varying joints
4. Inverse Kinmatics 
5. Minimum Trajectory Potting i.e straight line
6. Via Points Addition

Current Goals:
1. add animation
2. Make the library suitable to be used using DH paramaeters
3. Add various options for plotting Robot

Note: Functions using inverse kinematics functions are only available for 3DOF arm.
