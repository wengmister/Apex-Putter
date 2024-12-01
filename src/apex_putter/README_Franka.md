# How to use the Franka arm

Detailed instruction can be found on the couse website: https://nu-msr.github.io/ros_notes/ros2/franka.html

## Source
* Instead of using `source /path/to/franka_workspace/setup.bash`, add the `source /path/to/franka_workspace/setup.bash` source command to your `~/.bashrc` file so that it automatically sources the Franka workspace every time you open a new terminal session.
* After modifying ~/.bashrc, you need to run source ~/.bashrc or restart your terminal for the changes to take effect.


## Connecting to the Franka robot
* https://panda0.robot
    * username: student
    * password: robotics!

## Robot State
* white - can move freely
* blue - can move with command

## Using the Franka arm
* ssh student@station.robot

* Terminal 1:
    * `ros2 launch franka_fer_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot`

* Terminal 2:
    * `export ROS-DOMAIN_ID=0`
    * `source install/setup.bash`
    *  `ros2 launch franka_fer-moveit_config moveit_rviz.launch.py` - this opens rvis

* In RVIZ:
    * Under MotionPlanning, go to planning to move the franka
    * if you plan to move the arm beyond workspace, it will plan but will not execute

* If something is wrong:
    * kill ssh file
    * deactivate and reactivate
    * or might have to stop it and manually move it

* WHen you are done:
    * PLan a path back to ready position
    * press the e-stop
    * go to the web interface and lock the robot
    * onlt leave the robot unattended when it is in yellow state