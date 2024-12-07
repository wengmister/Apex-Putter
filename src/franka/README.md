# Using franka with Apex_Putter

#### **Original Joint to Tool Origin**
```yaml
src/'Apex-Putter>src>franka>src>franka_description>robots>fer' was modified so that 

joint8:
  kinematic:
    x: 0
    y: 0
    z: 0.107
    roll: 0
    pitch: 0
    yaw: 0

was changed to include the golf club length. 

The new joint8 length is:

# Joint to tool origin
joint8:
  kinematic:
    x: 0
    y: 0
    z: 0.68739
    roll: 0
    pitch: 0
    yaw: 0

<Terminal 1>
* 'source /opt/ros/jazzy/setup.bash'

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