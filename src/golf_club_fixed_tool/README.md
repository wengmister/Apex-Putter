# URDF

* I followed directions from `https://wiki.ros.org/sw_urdf_exporter` to convert a solidworks assembly file to a urdf file. 
* The original urdf downloaded from solidworks was was given in ROS1, and I manually changed to be in ROS2 format utilizing `https://github.com/xiaoming-sun6/sw2urdf_ros2`.


#### Terminal

* Go into the Apex-Putter folder.
* `colcon build --packages-select golf_club_fixed_tool`
* `source install/setup.bash`
* `ros2 launch golf_club_fixed_tool launch.py`

#### TF location
* Relative to the origin in the solidworks file, the putting impact happens at, `x=0.009652, y=-0.043688, z=0.054864`.
* So we refine the origin to that point so the tf is located at the correct putting impact point.
 