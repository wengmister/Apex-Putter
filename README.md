# Apex-Putter
Project repo for ME495 Final Project Group 3, 2024 Fall

## Authors:
- Zhengyang Kris Weng
- Andrew Kwolek
- Kyle Puckett
- Jueun Kwon
- Sayantani Bhattacharya 



## Overview
Miniature golf, even in its simplest form on a flat surface, presents an intriguing challenge for robotic systems. In this setup, a Franka Panda robot arm attempts the fundamental task of golf putting - striking a ball with just the right force to reach the hole. While humans develop this skill through intuition and practice, programming a robot to achieve the same feat requires precise control of position, force, and timing.

Using a robotic arm for this seemingly simple task allows researchers to explore fundamental questions in robotics and control theory. The Franka Panda must execute precise movements to control both the direction and power of each putt, translating mathematical models of physics into real-world actions. This intersection of robotics and sports not only advances our understanding of robot control but also provides an engaging demonstration of how robots can perform tasks that humans often take for granted.

## Mechanical System
Main components of apex-putter:
- Vision
    - RealSense D435i RGBD camera
- Franka FER Arm
    - Custom putter end-effector
        - Putter proximal end
        - Putter distal end
    - Custom Apriltag base tag choke
- Turf
    - Target apriltag pyramid

## ROS2 Package
Main ROS2 Nodes:
- Vision
    - YOLO based ball identification
        - ... and ball radius compensation
    - Apriltag based robot base transformation
    - Kabsch algorithm based robot base registration
    - Tf broadcasting
- Motion
    - Target vector calculation
    - Target pose calculation
        - Putt face compensation
        - Putter compensation
    - MoveIt! motion callbacks

## Sample Usage
0. a) Follow `DEPENDENCIES.md` to setup your environment b) Source environment with `source_installs_zkw.sh` - please update with your paths c) Start Franka on station
1. `ros2 launch apex_putter demo.launch.py`
2. `ros2 service call /home_robot std_srvs/srv/Empty`
3. `ros2 service call /ready std_srvs/srv/Empty`
4. `ros2 service call /putt std_srvs/srv/Empty`

## Demo Videos
