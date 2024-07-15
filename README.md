
# ROS Projects

This repository contains the projects I have done using ROS and the GAZEBO simulator.

## Projects Overview

### Bug0 Algorithm for Robot Navigation in a Maze

This project implements the Bug0 algorithm to autonomously navigate a robot within a maze environment using ROS (Robot Operating System). The ROS node responsible for controlling the robot receives inputs including the maze map and the robot's current position. Using this information, the node computes the next optimal position for the robot based on the Bug0 algorithm. The calculated position is then sent to the robot's controller for execution.

#### Key Features:
- Implementation of the Bug0 algorithm for maze navigation.
- Integration with ROS for communication and control.
- Input handling of maze map and robot position.
- Output of computed next positions to the robot's controller.

### Four-Wheel Rover Gazebo Model

This project focuses on creating a four-wheel rover model within the Gazebo simulator environment. The rover model is developed using the URDF format and incorporates essential components such as a chassis, four wheels, LiDAR sensor, and a camera. The primary objective is to simulate and analyze the behavior and performance of the four-wheel rover in a virtual environment.

#### Key Features:
- Creation of a URDF-based model for a four-wheel rover.
- Integration of components including chassis, wheels, LiDAR sensor, and camera.
- Utilization of Gazebo simulator for realistic virtual environment simulations.
- Exploration of rover behaviors and functionalities through simulated scenarios.

#### Usage:
- Use the provided URDF model to simulate various navigation and sensing tasks.
- Customize and extend the model for specific research or educational purposes in robotics.

### task_5_mr

This ROS package encompasses two main nodes designed for controlling a rover in a corridor and generating a star-shaped path. It leverages dynamic reconfiguration and ROS services for parameter updates.

#### Nodes:

1. **tutorial_laser**

    - **Description**: Implements a PD-controller to maintain the rover's straight-line path within a corridor using laser data feedback.
    - **Functionality**: Adjusts linear and angular velocities based on laser scan inputs to ensure the rover stays aligned and avoids collisions.
    - **Features**:
      - PD-controller for precise control.
      - Dynamic reconfiguration support for real-time parameter tuning.
      - ROS service integration to start/stop rover movements.

2. **tutorial_odom**

    - **Description**: Generates a star-shaped path based on user-specified parameters such as the number of edges and angles between them.
    - **Functionality**: Computes angular and linear velocities necessary for the rover to trace out a star pattern on the ground.
    - **Features**:
      - Calculation of angular velocities for turning edges.
      - Dynamic reconfiguration for adjusting path characteristics on-the-fly.
      - ROS service integration for parameter updates and control.

#### Package Dependencies:
- `dynamic_reconfigure`: Enables runtime adjustment of node parameters.
- `message_generation`: Facilitates message creation and communication.
- `roscpp`, `rospy`: ROS libraries for C++ and Python programming interfaces.
- `std_msgs`: Standard ROS messages used for communication.

#### Configuration Files:
- **Laser_Params.cfg**: Configures parameters for the PD-controller in `tutorial_laser`.
- **Odom_Params.cfg**: Configures parameters for generating the star-shaped path in `tutorial_odom`.

#### Execution:
Each node (`tutorial_laser` and `tutorial_odom`) can be launched independently to perform their respective tasks within the robotics environment.

#### Usage:
- Users interact with the package by dynamically adjusting parameters through ROS services and monitoring the rover's behavior via laser scans and path visualization.

## Getting Started

### Prerequisites
- ROS (Robot Operating System) installed on your machine.
- Gazebo simulator installed.
- Basic knowledge of ROS, Gazebo, and URDF.

### Installation
1. Clone this repository:
   ```sh
   git clone https://github.com/your_username/ROS_Projects.git
   ```
2. Navigate to the project directory:
   ```sh
   cd ROS_Projects
   ```
3. Build the workspace:
   ```sh
   catkin_make
   ```

### Running the Projects
#### Bug0 Algorithm
1. Launch the ROS node for Bug0 algorithm:
   ```sh
   roslaunch bug0_algorithm bug0.launch
   ```
2. Provide the maze map and initial robot position as inputs.

#### Four-Wheel Rover Model
1. Launch the Gazebo simulator with the rover model:
   ```sh
   roslaunch four_wheel_rover rover.launch
   ```

#### task_5_mr
1. Launch the `tutorial_laser` node:
   ```sh
   rosrun task_5_mr tutorial_laser
   ```
2. Launch the `tutorial_odom` node:
   ```sh
   rosrun task_5_mr tutorial_odom
   ```

---

