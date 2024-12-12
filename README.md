# Apex-Putter
Project repo for Group 3's Final Project for the 2024 Fall installment of Northweestern's ME495 Course.

## Authors:
- Zhengyang Kris Weng
- Andrew Kwolek
- Kyle Puckett
- Jueun Kwon
- Sayantani Bhattacharya 



## Overview
As kids, we thought robots were the coolest things ever. We viewed them as the future that can and would do any task you gave it. We thought robots wouldn't have limits. Also in our naive ways, many kids believed mini golf was the dominant display of sportsmanship, control, and the overall "cool factor." A blast from the past, for our final project for ME 495, we've decided to integrate these naive views into one project: Robotic Mini Golf.

Mini golf, a flat surface, presents an intriguing challenge for robotic systems. While kids develop mini golf skills through intuition, practice, and a little bit of healthy competition, programming a robot to achieve the same feat requires precision control of position and timing. A young kid needs a putter, a ball, a hole, and probably a bet and competition from their family/member to make a hole-in-one. They can just "eyeball."

A Franka Robot? A lot more complex. It may not feel competition, but sure does feel the pressure from a badly planned cartesian path. In this setup, a Franka Panda robot arm attempts the fundamental task of mini golf - putting a ball with just enough force to reach the hole. 

Our group uses vision integration via April Tag Detection, motion planning arm and joint positions, as well as cartesian paths, and just enough bit physics to prove that robots can play the simplest children's game (or "perfect" first date), but it's much more difficult than one would think. A whole lot of math and 3D printed parts later, we give you...Apex Putter!

![clown](https://media1.giphy.com/media/dVLAuRFgEsjCw/200w.gif?cid=6c09b952rgld8zt5pfe67ng0yz2zz934dsd5us8qnwjw3roz&ep=v1_gifs_search&rid=200w.gif&ct=g)

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
0. Follow `DEPENDENCIES.md` to setup your environment b) Source environment with `source_installs_zkw.sh` - please update with your paths c) Start Franka on station
1. `ros2 launch apex_putter demo.launch.py`
2. `ros2 service call /home_robot std_srvs/srv/Empty`
3. `ros2 service call /ready std_srvs/srv/Empty`
4. `ros2 service call /putt std_srvs/srv/Empty`

## Demo Videos
RVIZ:

https://github.com/user-attachments/assets/c37d6d04-b3b5-417b-8256-89fccbd9d821

Live Demonstrations:

https://github.com/user-attachments/assets/ca9eafb5-67a2-44d8-abbf-b2a758cc1057

