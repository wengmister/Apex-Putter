import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion, TransformStamped
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import math
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import time

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
        self.putface_ee_transform =  np.array([ [1, 0, 0, 0],
                                                [0, 1, 0, 0],
                                                [0, 0, 1, -0.53],
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

        # Dynamic transform brodcaster for putter-face.
        self.tf_dynamic_broadcaster = TransformBroadcaster(self)
        # If not looked up
        self.transform_base_ee = TransformStamped()
        self.transform_base_ee.header.stamp = self.get_clock().now().to_msg()

        # Services
        self.ready_srv = self.create_service(Empty, 'ready', self.ready_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.home_srv = self.create_service(Empty, 'home_robot', self.home_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.putt_srv = self.create_service(Empty, 'putt', self.putt_callback, callback_group=MutuallyExclusiveCallbackGroup())

        # Timer for optional tasks
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("DemoNode initialized. Use '/simulate' or '/real_putt'.")

    async def home_callback(self, request, response):
        """Make the robot go to home pose"""
        self.get_logger().info("Home requested.")
        await self.MPI.move_arm_joints(joint_values=[0.0, -0.4, 0.0, -1.6, 0.0, 1.57, 0.0])
        return response
    
    def dynamic_brodcaster(self,transform_base_ee: TransformStamped):
        T_be = transOps.transform_to_htm(transform_base_ee.transform)
        T_ep = self.putface_ee_transform
        T_pb = np.dot(np.linalg.inv(T_ep),np.linalg.inv(T_be))
        T = transOps.htm_to_transform(T_pb)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'fer_link8'
        t.child_frame_id = 'putt_face'
        t.transform = T    
        self.tf_dynamic_broadcaster.sendTransform(t)

    def look_up_fer8_frame(self):
        """Look up the ee position in the base frame"""
        self.get_logger().info("Looking up ball position in base frame.")
        try:
            self.transform_base_ee = self.tf_buffer.lookup_transform(self.base_frame, 'fer_link8', rclpy.time.Time())
            self.get_logger().info(f"Transform from 'fer_link8' to {self.base_frame}: {self.transform_base_ee}")
            # self.dynamic_brodcaster(transform_base_ee)
        except Exception as e:
            self.get_logger().error(f"Failed to look up the end-effector transform: {e}")

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
            self.ball_position = [0.60659744, -0.04259332,  0.05297366]
            self.get_logger().error(f"Failed to look up ball position, setting fallback value: {e}")
            self.get_logger().info(f"Fallback Ball position: {self.ball_position}")

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
            self.hole_position = [0.72347078, 0.29201838, 0.05362293]
            self.get_logger().error(f"Failed to look up hole position, setting fallback value: {e}")
            self.get_logger().info(f"Fallback Hole position: {self.hole_position}")
    
    def calculate_hole_to_ball_vector(self):
        """Calculate the vector from the hole to the ball"""
        self.look_up_ball_in_base_frame()
        self.look_up_hole_in_base_frame()
        self.hole_position[2] = self.ball_position[2] # flatten the hole position on z-axis
        return [b - h for b, h in zip(self.ball_position, self.hole_position)]  # vector: hole to ball

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
        pose_wrt_putface[2]= self.ball_position[2] + self.v_b2p[2] + 0.53  #translated to make it wrt ee

        # Puttface to ee transform
        pose_wrt_putface_homogeneous = np.append(pose_wrt_putface, 1).reshape(1, 4)
        self.get_logger().info(f"\n ========================== self.putface_ee_transform { self.putface_ee_transform}===================================")

        pose_wrt_ee = pose_wrt_putface
        # pose_wrt_ee = np.dot(pose_wrt_putface_homogeneous, self.putface_ee_transform)
        self.get_logger().info(f"==========================pose_wrt_ee {pose_wrt_ee} and z is {pose_wrt_ee[2]} ===================================")

        orientation = self.calculate_putting_orientation(self.v_b2p)
        ready_pose = Pose()
        # to test
        # pose_wrt_ee = [0.5,0,0.590]
        ready_pose.position.x = pose_wrt_ee[0]
        ready_pose.position.y = pose_wrt_ee[1]
        ready_pose.position.z = pose_wrt_ee[2]
        # ready_pose.orientation = orientation

        # keeping orientation constant.
        ready_pose.orientation = self.transform_base_ee.transform.rotation
        self.get_logger().info(f"================== orientation: {ready_pose.orientation} ===========================================")

        self.get_logger().info("================== Await mpi starts ===========================================")
        # sleep to prevent oscillation btw quat angles - need to understand
        time.sleep(3)
        await self.MPI.move_arm_pose(ready_pose, max_velocity_scaling_factor=0.5, max_acceleration_scaling_factor=0.5)
        return response
    
    async def putt_callback(self, request, response):
        """Putt the ball"""
        self.get_logger().info("Putt requested.")
        self.get_logger().info("=============================================================")
        
        # Swinging till a distance of 0.2, towards the ball.
        putt_pose_wrt_pf = np.zeros(3)
        putt_pose_wrt_pf[0] = self.ball_position[0] - 0.2 * self.v_h2b[0]
        putt_pose_wrt_pf[1] = self.ball_position[1] - 0.2 * self.v_h2b[1]
        putt_pose_wrt_pf[2] = self.ball_position[2]

        # Puttface to ee transform.
        pose_homogeneous = np.append(putt_pose_wrt_pf, 1).reshape(1, 4)
        putt_pose_wrt_ee = np.dot(pose_homogeneous, self.putface_ee_transform)

        putt_pose = Pose()
        putt_pose.position.x = putt_pose_wrt_ee[0,0]
        putt_pose.position.y = putt_pose_wrt_ee[0,1]
        putt_pose.position.z = putt_pose_wrt_ee[0,2]

        orientation = self.calculate_putting_orientation(self.v_b2p)
        putt_pose.orientation = orientation
        await self.MPI.move_arm_pose(putt_pose, max_velocity_scaling_factor=0.8, max_acceleration_scaling_factor=0.8)
        return response
    
    def timer_callback(self):
        self.dynamic_brodcaster(self.transform_base_ee)

def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
