# Golf Club URDF

* I followed directions from `https://wiki.ros.org/sw_urdf_exporter` to convert a solidworks assembly file to a urdf file. 
* The original urdf downloaded from solidworks was was given in ROS1, and I manually changed to be in ROS2 format.
* Terminal:
    * cd ~/ws/final_project
    * colcon build
    * source install/setup.bash
    * `ros2 launch golf_club display.launch.xml`
    * smt is wrong.... maybe i change it back to the original ros1 file and use a ros1 to ros2 to bridge?