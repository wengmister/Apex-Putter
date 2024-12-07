# Using franka with Apex_Putter

* 'source /opt/ros/jazzy/setup.bash'

<Terminal 1>
* 'cd ~/ws/final_project/src/Apex-Putter/src/franka'

Build the franka workspace:
* 'colcon clean workspace'
* 'rosdep install --from-paths src -y --ignore-src -r'
* 'colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF'

Source the franka workspace:
* 'nano ~/.bashrc'
* add 'source ~/ws/final_project/src/Apex-Putter/src/franka/install/setup.bash' to the end
* 'source ~/.bashrc'

Run the demo file:
* 'ros2 launch franka_fer_moveit_config demo.launch.py'
