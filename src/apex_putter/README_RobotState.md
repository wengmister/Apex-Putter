# RobotState and TestRobotState Node README

## Overview

This README provides an in-depth explanation of the `RobotState` class and the `TestRobotState` node. The `RobotState` class is designed to interface with ROS 2 and MoveIt to perform forward and inverse kinematics computations for a robot manipulator. The `TestRobotState` node is a ROS 2 node that tests the functionalities of the `RobotState` class and can be modified for future iterations.

## Table of Contents

- [RobotState Class](#robotstate-class)
  - [Features](#features)
  - [Dependencies](#dependencies)
- [TestRobotState Node](#testrobotstate-node)
  - [Functionality](#functionality)
  - [Dependencies](#dependencies-1)
- [Running the Test Node](#running-the-test-node)
  - [Launching MoveIt](#launching-moveit)
  - [Launching the Test Node](#executing-the-test-node)
- [Understanding the Output](#understanding-the-test-output)
- [Structure](#code-structure)
  - [robot_state.py](#robot_statepy)
  - [test_robot_state.py](#test_robot_statepy)
- [Troubleshooting](#troubleshooting)

## RobotState Class

The `RobotState` class encapsulates functionalities to interact with a robot's state in ROS 2, particularly focusing on:

- Retrieving the current joint states.
- Retrieving the current end-effector pose.
- Computing inverse kinematics (IK) to find joint configurations for a desired end-effector pose.
- Computing forward kinematics (FK) to find the end-effector pose for a given joint configuration.

### Features

- **Joint State Retrieval**: Subscribes to the robot's joint state topic to get real-time joint positions.
- **End-Effector Pose Retrieval**: Uses the TF2 transformation library to get the current pose of the end effector relative to the base frame.
- **Inverse Kinematics**: Interfaces with MoveIt's `compute_ik` service to compute joint configurations for a desired pose.
- **Forward Kinematics**: Interfaces with MoveIt's `compute_fk` service to compute the end-effector pose from given joint configurations.

### Dependencies

- ROS 2
- MoveIt 2.
- Python packages:
  - `rclpy`
  - `geometry_msgs`
  - `sensor_msgs`
  - `moveit_msgs`
  - `tf2_ros`
  - `std_msgs`

## TestRobotState Node

The `TestRobotState` node is a ROS 2 node designed to test and demonstrate the functionalities of the `RobotState` class. It performs the following actions:

- Retrieves and logs the current joint states.
- Retrieves and logs the current end-effector pose.
- Performs inverse kinematics for a specified desired pose and logs the resulting joint states.
- Performs forward kinematics for a specified joint configuration and logs the resulting end-effector pose.

### Functionality

1. **Initialization**: The node initializes an instance of the `RobotState` class with the robot's parameters.
2. **Scheduling Tests**: After a sleeping (forcing a delay), it starts the tests to ensure all necessary services and topics are available.
3. **Running Tests**:
   - Retrieves and logs the current joint state.
   - Retrieves and logs the current end-effector pose.
   - Performs IK for a desired pose and logs the resulting joint state.
   - Performs FK for a given joint state and logs the resulting end-effector pose.
4. **Shutdown**: After the tests are completed, the node signals completion and terminates. This 
5. **IMPORTANT**: Running this node **will** make your computer take off!

### Dependencies

- All dependencies listed under the `RobotState` class.
- The test node must run in an environment where MoveIt is running and configured for the robot.
- Please ensure the franka workspace is set up on your computer for MoveIt

## Running the Test Node

### Launching MoveIt

- Ensure that you have the franka workspace set up on your computer
- Open a new terminal
- cd into your franka workspace and run `colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF`
- Next, source your workspace
- Run `ros2 launch franka_fer_moveit_config demo.launch.py`

### Test Node
<!-- - Open a new terminal (2nd Terminal)
- cd into your workspace and run `colcon build --packages-select motion_planner`
- Next, source your workspace
- Run `ros2 launch motion_planner test_robot_state.launch.py` -->

## Understanding the Output

Upon running the test node, you should see output similar to:


[INFO] [test_robot_state]: Current Joint State:
sensor_msgs.msg.JointState(...)

[INFO] [test_robot_state]: Current End-Effector Pose:
geometry_msgs.msg.PoseStamped(...)

[INFO] [test_robot_state]: IK Result (Joint State):
sensor_msgs.msg.JointState(...)

[INFO] [test_robot_state]: FK Result (End-Effector Pose):
geometry_msgs.msg.PoseStamped(...)

[INFO] [test_robot_state]: Tests completed.


- **Current Joint State**: Shows the robot's current joint positions.
- **Current End-Effector Pose**: Displays the current pose of the end effector relative to the base frame.
- **IK Result**: Shows the joint configuration computed to reach the desired end-effector pose.
- **FK Result**: Displays the end-effector pose computed from the specified joint configuration.
- **Tests completed**: Indicates that all tests have run successfully.

## Code Structure

### `robot_state.py`

This file contains the `RobotState` class.

- **Initialization**:
  - Sets up subscribers and clients.
  - Subscribes to the joint state topic.
  - Initializes TF2 listeners.
- **Methods**:
  - `get_current_joint_state()`: Returns the latest joint state.
  - `get_current_end_effector_pose()`: Asynchronously retrieves the current end-effector pose.
  - `compute_inverse_kinematics(pose)`: Asynchronously computes IK for a given pose.
  - `compute_forward_kinematics(joint_state)`: Asynchronously computes FK for a given joint state.

### `test_robot_state.py`

This file contains the `TestRobotState` node.

- **Initialization**:
  - Creates an instance of the `RobotState` class.
  - Sets up a timer to start tests after a delay.
- **Methods**:
  - `run_tests()`: Schedules the asynchronous `_run_tests()` method.
  - `_run_tests()`: An asynchronous method that performs the tests.
- **Main Function**:
  - Initializes ROS 2 and the executor.
  - Spins the executor until the tests are completed.
  - Handles shutdown gracefully.

## Troubleshooting

- **Services Not Available**: If you encounter errors related to services not being available, ensure that MoveIt is running and that the `compute_ik` and `compute_fk` services are active.
  - **Check available services**:
    ```bash
    ros2 service list
    ```
- **TF Transform Issues**: If the node cannot retrieve the end-effector pose, ensure that the TF2 broadcasters are running and that the frames (`base`, `fer_link8`) are correct.
  - **Check TF tree**:
    ```bash
    ros2 run tf2_tools view_frames.py
    ```
- **Incorrect Frame or Joint Names**: Verify that the frame names and joint names in your code match those defined in your robot's URDF and MoveIt configuration.
- **Import Errors**: Ensure that the `robot_state.py` and `test_robot_state.py` files are in the correct package and that the package is built and sourced properly.
