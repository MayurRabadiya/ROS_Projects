task_5_mr

This ROS package encompasses two main nodes designed for controlling a rover in a corridor and generating a star-shaped path. It leverages dynamic reconfiguration and ROS services for parameter updates.

Nodes:

tutorial_laser

Description: Implements a PD-controller to maintain the rover's straight-line path within a corridor using laser data feedback.
Functionality: It adjusts linear and angular velocities based on laser scan inputs to ensure the rover stays aligned and avoids collisions.
Features:
PD-controller for precise control.
Dynamic reconfiguration support for real-time parameter tuning.
ROS service integration to start/stop rover movements.
tutorial_odom

Description: Generates a star-shaped path based on user-specified parameters such as the number of edges and angles between them.
Functionality: Computes angular and linear velocities necessary for the rover to trace out a star pattern on the ground.
Features:
Calculation of angular velocities for turning edges.
Dynamic reconfiguration for adjusting path characteristics on-the-fly.
ROS service integration for parameter updates and control.
Package Dependencies:

dynamic_reconfigure: Enables runtime adjustment of node parameters.
message_generation: Facilitates message creation and communication.
roscpp, rospy: ROS libraries for C++ and Python programming interfaces.
std_msgs: Standard ROS messages used for communication.
Configuration Files:

Laser_Params.cfg: Configures parameters for the PD-controller in tutorial_laser.
Odom_Params.cfg: Configures parameters for generating the star-shaped path in tutorial_odom.
Execution:

Each node (tutorial_laser and tutorial_odom) can be launched independently to perform their respective tasks within the robotics environment.
Usage:

Users interact with the package by dynamically adjusting parameters through ROS services and monitoring the rover's behavior via laser scans and path visualization.
