import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import apex_putter.MotionPlanningInterface as MotionPlanningInterface
import numpy as np
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from std_srvs.srv import Trigger

class Calibrator(Node):
    def __init__(self):
        super().__init__('calibrator')
        
        # Declare parameters for frames
        self.declare_parameter('robot_base_frame', 'base')
        self.declare_parameter('robot_ee_frame', 'fer_link8')
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
            end_effector_frame=self.robot_ee_frame
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
        
        self.pose_list = None
        self.generate_all_poses()
        self.pose_counter = 0

    def move_callback(self, request, response):
        if self.pose_counter >= len(self.pose_list):
            response.success = False
            response.message = "All poses visited"
            return response
            
        current_pose = self.pose_list[self.pose_counter]
        success = self.MPI.move_to_pose(current_pose)
        
        if success:
            self.pose_counter += 1
            response.success = True
            response.message = f"Moved to pose {self.pose_counter}/{len(self.pose_list)}"
        else:
            response.success = False
            response.message = "Failed to move to pose"
            
        return response

    def generate_all_poses(self):
        # Generate poses by linspacing in x = [0.2, 0.5], y = [-0.5, 0.5], z = [0.2, 0.5]
        x = np.linspace(0.2, 0.5, 3)
        y = np.linspace(-0.5, 0.5, 3)
        z = np.linspace(0.2, 0.5, 3)
        
        self.pose_list = []
        for xi in x:
            for yi in y:
                for zi in z:
                    self.pose_list.append(self.generate_pose(xi, yi, zi))

    def generate_pose(self, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        # Set orientation to pointing downward
        pose.orientation.x = 0.0
        pose.orientation.y = 1.0
        pose.orientation.z = 0.0
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

def main(args=None):
    rclpy.init(args=args)
    calibrator = Calibrator()
    rclpy.spin(calibrator)
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()