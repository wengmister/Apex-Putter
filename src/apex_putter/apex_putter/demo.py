import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import math
import tf2_ros

from apex_putter.MotionPlanningInterface import MotionPlanningInterface
import apex_putter.transform_operations as transOps

from tf_transformations import quaternion_from_euler

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')

        # Parameter to toggle simulation vs real-world
        self.declare_parameter('simulation_mode', True)
        self.use_simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value

        # Declare parameters for frames
        self.declare_parameter('ball_tag_frame', 'ball')
        self.declare_parameter('hole_tag_frame', 'tag_15')
        self.declare_parameter('base_frame', 'robot_base_frame')
        self.declare_parameter('camera_frame', 'camera_link')

        self.ball_tag_frame = self.get_parameter('ball_tag_frame').get_parameter_value().string_value
        self.hole_tag_frame = self.get_parameter('hole_tag_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        # Known offsets:
        self.putter_length = 22.85 * 0.0254  # About 0.58 m
        self.putter_offset = 0.18 * 0.0254  # About 0.00457 m

        # Dimentions:(in m)
        self.puttface_dim = [0.1,0.02,0.028]
        self.goal_ball_radius = 2.03
        # testing vals.
        self.putface_ee_transform =  np.array([
                                                [0.7071, -0.7071, 0, 1],
                                                [0.7071, 0.7071, 0, 0.44454056],
                                                [0, 0, 1, 0.66401457],
                                                [0, 0, 0, 1]])

        # Motion planning interface
        self.MPI = MotionPlanningInterface(
            node=self,
            base_frame=self.base_frame,
            end_effector_frame='fer_link8'
        )

        self.hole_position = None
        self.ball_position = None
        self.v_h2b = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Services
        self.ready_srv = self.create_service(Empty, 'ready', self.ready_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.home_srv = self.create_service(Empty, 'home_robot', self.home_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.putt_srv = self.create_service(Empty, 'putt', self.putt_callback, callback_group=MutuallyExclusiveCallbackGroup())

        # Timer for optional tasks
        # self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("DemoNode initialized. Use '/simulate' or '/real_putt'.")

    async def home_callback(self, request, response):
        """Make the robot go to home pose"""
        self.get_logger().info("Home requested.")
        await self.MPI.move_arm_joints(joint_values=[0.0, -0.4, 0.0, -1.6, 0.0, 1.57, 0.0])
        return response
    
    def look_up_ball_in_base_frame(self):
        """Look up the ball position in the base frame"""
        self.get_logger().info("Looking up ball position in base frame.")
        try:
            transform_base_ball = self.tf_buffer.lookup_transform(self.base_frame, self.ball_tag_frame, rclpy.time.Time())
            self.get_logger().info(f"Transform from {self.ball_tag_frame} to {self.base_frame}: {transform_base_ball}")
            htm_base_ball = transOps.transform_to_htm(transform_base_ball.transform)
            self.ball_position = htm_base_ball[:3, 3]
            self.get_logger().info(f"Ball position: {self.ball_position}")
        except Exception as e:
            self.get_logger().error(f"Failed to look up ball position: {e}")

    def look_up_hole_in_base_frame(self):
        """Look up the hole position in the base frame"""
        self.get_logger().info("Looking up hole position in base frame.")
        try:
            transform_base_hole = self.tf_buffer.lookup_transform(self.base_frame, self.hole_tag_frame, rclpy.time.Time())
            self.get_logger().info(f"Transform from {self.hole_tag_frame} to {self.base_frame}: {transform_base_hole}")
            htm_base_hole = transOps.transform_to_htm(transform_base_hole.transform)
            self.hole_position = htm_base_hole[:3, 3]
            self.get_logger().info(f"Hole position: {self.hole_position}")
        except Exception as e:
            self.get_logger().error(f"Failed to look up hole position: {e}")

    def calculate_hole_to_ball_vector(self):
        """Calculate the vector from the hole to the ball"""
        self.look_up_ball_in_base_frame()
        self.look_up_hole_in_base_frame()
        self.hole_position[2] = self.ball_position[2] # flatten the hole position on z-axis
        return self.ball_position - self.hole_position # vector: hole to ball

    def calculate_ball_to_puttface_vector(self, v_h2b, putting_distance = 0.1):
        '''
            Calculate the vector from the centre of ball to the centre of puttface.

            Args: 
                v_h2b = vector from hole to ball.
                putting_distance (Optional) : the distance in direction of 
                                    putting where the robot would be positioned to start the swing action.

            Returns: The pose of the putterface to start the swing.
        '''        
        dx = v_h2b[0]
        dy = v_h2b[1]
        dz = v_h2b[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        scaling_factor = (putting_distance)/ distance         
        # Coordinates of the ball center
        x_r = dx * scaling_factor + (self.puttface_dim[0]/2)
        y_r = dy * scaling_factor + (self.puttface_dim[1]/2)
        z_r = dz * scaling_factor + (self.puttface_dim[2]/2)
        v_b2p = [x_r,y_r,z_r]
        return v_b2p
    
    def calculate_putting_orientation(self, v_b2p):
        """ 
        Caculating the unit vector (Quaternion) in the direction opposite to ball to puttface vector.
        """        
        norm_ang = np.linalg.norm(v_b2p)
        if norm_ang == 0:
            raise ValueError("Ball to putter vector has no direction.")
        unit_vector = v_b2p / norm_ang
        # Reverse direction
        opposite_vector = -unit_vector
        putting_angle = -norm_ang

        # Calculate quaternion components : need confirm the formula.
        w = np.cos(putting_angle / 2)
        x = unit_vector[0] * np.sin(putting_angle / 2)
        y = unit_vector[1] * np.sin(putting_angle / 2)
        z = unit_vector[2] * np.sin(putting_angle / 2)
        orientation = Quaternion(x=x, y=y, z=z, w=w)
        return orientation
    
    async def ready_callback(self, request, response):
        """Prepare the robot for putting"""
        self.get_logger().info("Ready requested.")
        self.get_logger().info("=============================================================")

        self.v_h2b = self.calculate_hole_to_ball_vector()
        self.v_b2p = self.calculate_ball_to_puttface_vector(self.v_h2b)   
        pose_wrt_putface = np.zeros(3)
        pose_wrt_putface[0] = self.ball_position[0] + self.v_b2p[0]
        pose_wrt_putface[1] = self.ball_position[1] + self.v_b2p[1]
        pose_wrt_putface[2]= self.ball_position[2] + self.v_b2p[2]

        # Puttface to ee transform
        pose_wrt_putface_homogeneous = np.append(pose_wrt_putface, 1).reshape(1, 4)
        pose_wrt_ee = pose_wrt_putface_homogeneous * self.putface_ee_transform

        orientation = self.calculate_putting_orientation(self.v_b2p)
        ready_pose = Pose()
        ready_pose.position.x = pose_wrt_ee[0]
        ready_pose.position.y = pose_wrt_ee[1]
        ready_pose.position.z = pose_wrt_ee[2]
        ready_pose.orientation = orientation
        await self.MPI.move_arm_pose(ready_pose, max_velocity_scaling_factor=0.5, max_acceleration_scaling_factor=0.5)
        return response
    
    async def putt_callback(self, request, response):
        """Putt the ball"""
        self.get_logger().info("Putt requested.")
        self.get_logger().info("=============================================================")

        putt_pose = Pose()
        putt_pose.position.x = self.ball_position[0] - 0.2 * self.v_h2b[0]
        putt_pose.position.y = self.ball_position[1] - 0.2 * self.v_h2b[1]
        putt_pose.position.z = self.ball_position[2]
        # make oritentation vertical downwards
        putt_pose.orientation = Quaternion(x=0.92, y=-0.38, z=0.00035, w=0.0004)
        await self.MPI.move_arm_pose(putt_pose, max_velocity_scaling_factor=0.8, max_acceleration_scaling_factor=0.8)
        return response
    
        
    # def offset_ball_position(self, z):
    #     """Offset the ball position by z"""
    #     self.ball_position[2] += z

def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
