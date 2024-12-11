import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from apex_putter.MotionPlanningInterface import MotionPlanningInterface
import numpy as np
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from std_srvs.srv import Trigger
import tf_transformations

class Calibrator(Node):
    def __init__(self):
        super().__init__('calibrator')
        
        # Declare parameters for frames
        self.declare_parameter('robot_base_frame', 'base')
        self.declare_parameter('robot_ee_frame', 'fer_link8_extended')
        self.declare_parameter('camera_base_frame', 'camera_color_optical_frame')
        self.declare_parameter('camera_target_frame', 'tag_36')
        
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.robot_ee_frame = self.get_parameter('robot_ee_frame').get_parameter_value().string_value
        self.camera_base_frame = self.get_parameter('camera_base_frame').get_parameter_value().string_value
        self.camera_target_frame = self.get_parameter('camera_target_frame').get_parameter_value().string_value
        
        self.robot_points = []
        self.camera_points = []
        
        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Motion planning interface
        self.MPI = MotionPlanningInterface(
            node=self,
            base_frame=self.robot_base_frame,
            end_effector_frame='fer_link8'
        )
        
        # Services
        self.move_service = self.create_service(
            Trigger,
            'move_to_next_pose',
            self.move_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.save_data_service = self.create_service(
            Trigger,
            'save_current_pose',
            self.save_data_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.calibrate_service = self.create_service(
            Trigger,
            'calibrate',
            self.calibrate_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Add new service for getting transform
        self.get_transform_service = self.create_service(
            Trigger,
            'get_calibrated_transform',
            self.get_transform_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Add tf broadcaster for publishing calibration transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.calibration_transform = None
        
        self.pose_list = None
        self.generate_all_poses()
        self.pose_counter = 0
        self.broadcast_extension()
        self.get_logger().info("Calibrator node initialized")

    def broadcast_extension(self):
        """Broadcast the extension transform"""
        extension_transform = TransformStamped()
        extension_transform.header.stamp = self.get_clock().now().to_msg()
        extension_transform.header.frame_id = "fer_link8"
        extension_transform.child_frame_id = "fer_link8_extended"
        extension_transform.transform.translation.x = 0.0310
        extension_transform.transform.translation.z = 0.0505
        self.static_tf_broadcaster.sendTransform(extension_transform)


    async def move_callback(self, request, response):
        """Async service callback for moving to the next pose."""
        if self.pose_counter >= len(self.pose_list):
            response.success = False
            response.message = "All poses visited"
            return response
            
        current_pose = self.pose_list[self.pose_counter]
        success = await self.MPI.move_arm_pose(current_pose, max_velocity_scaling_factor=0.25, max_acceleration_scaling_factor=0.2)
        
        if success:
            self.pose_counter += 1
            response.success = True
            response.message = f"Moved to pose {self.pose_counter}/{len(self.pose_list)}"
        else:
            response.success = False
            response.message = "Failed to move to pose"
            
        return response

    def generate_all_poses(self):
        """Generate 18 poses in a grid pattern with specified ranges"""
        # Generate poses by linspacing in x = [0.4, 0.6], y = [-0.4, 0.4], z = [0.2, 0.5]
        # Create 18 poses by adjusting the number of points in each dimension
        x = np.linspace(0.4, 0.55, 2)  # 3 points
        y = np.linspace(-0.45, 0.4, 3)  # 3 points
        z = np.linspace(0.3, 0.5, 2)  # 2 points to get 3*3*2 = 18 poses
        
        self.pose_list = []
        for xi in x:
            for yi in y:
                for zi in z:
                    self.pose_list.append(self.generate_pose(xi, yi, zi))

    def generate_pose(self, x, y, z):
        """Generate a pose with position and orientation (180° rotation about z-axis)"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # Set orientation to pointing downward with 180° rotation about z-axis
        # This is equivalent to rotating the end-effector 180° about its z-axis
        # from the downward-pointing position
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0  # Negative y to rotate 180° about z
        pose.orientation.z = 0.2
        pose.orientation.w = 0.0
        
        return pose

    def save_data_callback(self, request, response):
        try:
            # Get robot end-effector position
            robot_transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.robot_ee_frame,
                rclpy.time.Time()
            )
            
            # Get camera target position
            camera_transform = self.tf_buffer.lookup_transform(
                self.camera_base_frame,
                self.camera_target_frame,
                rclpy.time.Time()
            )
            
            # Extract positions
            robot_point = np.array([
                robot_transform.transform.translation.x,
                robot_transform.transform.translation.y,
                robot_transform.transform.translation.z
            ])
            
            camera_point = np.array([
                camera_transform.transform.translation.x,
                camera_transform.transform.translation.y,
                camera_transform.transform.translation.z
            ])
            
            self.robot_points.append(robot_point)
            self.camera_points.append(camera_point)
            
            response.success = True
            response.message = f"Saved point pair {len(self.robot_points)}"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to save points: {str(e)}"
            
        return response

    def calibrate_callback(self, request, response):
        if len(self.robot_points) < 3:
            response.success = False
            response.message = "Need at least 3 point pairs for calibration"
            return response
            
        try:
            R, t = self.calibrate()
            response.success = True
            response.message = f"Calibration successful. R:\n{R}\nt:\n{t}"
        except Exception as e:
            response.success = False
            response.message = f"Calibration failed: {str(e)}"
            
        return response

    def calibrate(self):
        # Convert points to numpy arrays
        P = np.array(self.robot_points)
        Q = np.array(self.camera_points)
        
        # Center the points
        p_centroid = np.mean(P, axis=0)
        q_centroid = np.mean(Q, axis=0)
        
        P_centered = P - p_centroid
        Q_centered = Q - q_centroid
        
        # Calculate covariance matrix
        H = P_centered.T @ Q_centered
        
        # SVD decomposition
        U, S, Vt = np.linalg.svd(H)
        
        # Calculate rotation matrix
        R = Vt.T @ U.T
        
        # Ensure right-handed coordinate system
        if np.linalg.det(R) < 0:
            Vt[-1,:] *= -1
            R = Vt.T @ U.T
        
        # Calculate translation
        t = q_centroid - R @ p_centroid
        
        return R, t
    
    def matrix_to_transform(self, R, t):
        """Convert rotation matrix and translation vector to TransformStamped message"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.camera_base_frame
        transform.child_frame_id = "calibrated_base"
        
        # Set translation
        transform.transform.translation.x = float(t[0])
        transform.transform.translation.y = float(t[1])
        transform.transform.translation.z = float(t[2])
        
        # Convert rotation matrix to quaternion
        q = tf_transformations.quaternion_from_matrix(
            np.vstack([np.hstack([R, np.zeros((3, 1))]),
                      [0, 0, 0, 1]])
        )
        
        transform.transform.rotation.x = float(q[0])
        transform.transform.rotation.y = float(q[1])
        transform.transform.rotation.z = float(q[2])
        transform.transform.rotation.w = float(q[3])
        
        return transform

    def calibrate_callback(self, request, response):
        if len(self.robot_points) < 3:
            response.success = False
            response.message = "Need at least 3 point pairs for calibration"
            return response
            
        try:
            R, t = self.calibrate()
            # Convert to transform and store it
            self.calibration_transform = self.matrix_to_transform(R, t)
            
            # Start broadcasting the transform
            self.create_timer(0.1, self.broadcast_transform)  # 10Hz broadcast rate
            
            response.success = True
            response.message = f"Calibration successful. R:\n{R}\nt:\n{t}"
        except Exception as e:
            response.success = False
            response.message = f"Calibration failed: {str(e)}"
            
        return response

    def broadcast_transform(self):
        """Timer callback to broadcast the calibration transform"""
        if self.calibration_transform is not None:
            self.calibration_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.calibration_transform)

    def get_transform_callback(self, request, response):
        """Service to get transform from robot_base_tag to calibrated_base"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "robot_base_tag",
                "calibrated_base",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)  # 1 second timeout
            )
            
            response.success = True
            response.message = f"Transform from robot_base_tag to calibrated_base:\n{transform}"
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            response.success = False
            response.message = f"Failed to get transform: {str(e)}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    calibrator = Calibrator()
    rclpy.spin(calibrator)
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()