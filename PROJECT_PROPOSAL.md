# Robotic Mini Golf Project Proposal

## Group Name and Number
* Group Name: Apex Putter
* Group number:  #3
* Names: Sayantani Bhattacharya, Andrew Kwolek, Jueun Kwon, Kyle Puckett, Zhengyang Kris Weng

## Description of the project
* This project aims to develop a robotic mini-golf system where a Franka Emika robotic arm is equipped to perform precise putting actions. Using a vision system for ball and hole detection, motion planning, and optimized force control, the robot will be able to execute a hole-in-one shot from a predetermined position, with future extensions for adaptive shots and challenging conditions.

## Goals

### Core Goal
* Successfully execute a hole-in-one putt from a predetermined starting position using the Franka arm (straight trajectory from predetermined starting position)
* Achieve consistent ball trajectory control.
* Ensure club face alignment.
* Implement vision system to detect ball and hole positions
* Calculate and execute appropriate putting force and angle.
* Design an appropriate club attachment to hold the club with the Franka arm.

### Fallback Goal
* Successfully putt the ball in a straight line towards a target from a fixed position
* Constraints and simplifications:
	* Use a fixed starting position for initial development and straight trajectory.
	* Place April Tags around the putting surface for precise position reference
	* Use a simplified flat putting surface.
	* Begin with constant putting force for initial testing.

### Stretch Goal
* Achieve hole-in-one from any arbitrary starting position.
* Object detection using segmentation (to recognize ball and hole)
* Handle varying green slopes and uneven conditions.
* Implement reinforcement learning in simulation to optimize putting strategy based on putting angle and putting velocity.
	* Real-time adjustment of putting velocity and angle based on distance and terrain.

## Robot and Equipment

### Robot
* Franka Emika Robot (FER)

### Additional Equipment Required
1. Custom end-effector for golf putter attachment.
2. Mini golf putter (junior size for appropriate weight — average putter weighs 0.4 to 0.89 kg depending on size)
3. Practice putting green turf
4. Practice golf balls (real and plastic golf balls for testing)
5. April Tags and mounting materials
6. RealSense camera for vision system or equivalent camera for vision system

## High level overview

### System Architecture
The project is divided into these main subsystems:

1. Vision System
	* Ball and hole detection (core goal)
	* April Tag position tracking (fallback goal)
	* Surface plane estimation (stretch goal)

2. Motion Planning
	* Inverse kinematics for putting positions (stretch goal)
	* Trajectory generation (core/fallback goal)
	* Collision avoidance (stretch goal) 

3. Putting Control
	* Club face alignment (core goal)
	* Force control (stretch goal)
	* Impact point optimization (stretch goal)

4. Learning System (Stretch Goal)
	* Simulation environment
	* Reinforcement learning implementation
	* Sim-to-real transfer

5. Application 
	* Integration of all other modules
    * Demo tasks

### Block diagram
![image](https://github.com/user-attachments/assets/f60cffde-aa89-43d1-8a44-8290f7d7a1c9)


### Third-Party Packages
* MoveIt 2 for kinematic motion planning
* OpenCV for vision processing
* AprilTag ROS 2 package
* [Gazebo/Coppelia for simulation]
* [Stable Baselines3 for RL implementation]

### Things to be Built
* 3D attachment between the club and the Franka arm end effector.

## Team Member Roles
* Member 1: Mechanical to Simulation - Creating URDF and integration into end-effector.
	* Primary: Jueun
	* Secondary: Kris
* Member 2: Vision System - AprilTag and extrinsic calibration [and image segmentation + distance sensing as stretch]
	* Primary: Kris
	* Secondary: Jueun
* Member 3: Motion Planning and Control - Task trajectory generation and perform fun motions.
	* Primary: Andrew
	* Secondary: Sayantani
* Member 4: Sample Task - Simulation of the franka arm putting the ball.
	* Primary: Sayantani
	* Secondary: Kyle
* Member 5: Application and Integration - Create ROS packages for demo tasks, and physical system integration of robot arm and putter
	* Primary: Kyle
	* Secondary: Andrew
* Each member will participate in the integrated testing and contribute to their subsystems' documentation. Weekly integration meetings will ensure system cohesion and progress tracking.
