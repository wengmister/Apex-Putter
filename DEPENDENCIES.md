# Python dependency setup in ROS
This library depends on ultralytics, a machine learning package to perform YOLO image segmentation. This needs to be setup externally in a separate ros package acting as a fake `virtual environment` to be sourced.

Since ROS2 doesn't work with normal venv, and pip install doesn't work with system managed site-packages, this is a workaround.

## Instructions
1. Create a new ament python pkg, e.g.
`ros2 pkg create --build-type ament_python --license MIT --maintainer-name "Zhengyang Kris Weng" --maintainer-email "wengmister@gmail.com" --description "Venv for ROS2" venv_ros`

2. `colcon build`

3. `. install/setup.bash`

4. `PYTHONUSERBASE="$(ros2 pkg prefix venv_ros)" pip3 install --user --break-system-packages ultralytics` ... and other packages needed

We will need to source this pkg install location to use the dependant packages.

See https://nu-msr.github.io/ros_notes/ros2/colcon.html#pip_ros for more details.
