import cv2
import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from apex_putter_interfaces.msg import Detection2D, DetectionArray


class YoloNode(Node):
    """
    Use Yolo to identify golf balls

    Subscribes
    ----------
    image (sensor_msgs/msg/Image) - The input image

    Publishes
    ---------
    new_image (sensor_msgs/msg/Image) - The image with the detections
    detections (detection_msgs/msg/DetectionArray) - Array of all detection center coordinates

    Parameters
    model (string) - The Yolo model to use: see docs.ultralytics.org for available values. Default is yolo11n.pt
    """
    def __init__(self):
        super().__init__("pose")
        self.bridge = CvBridge()
        self.declare_parameter("model",
                               value="best.pt")
        self.model = YOLO(self.get_parameter("model").get_parameter_value().string_value)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.yolo_callback, 10)
        self.image_pub = self.create_publisher(Image, 'image_yolo', 10)
        self.detections_pub = self.create_publisher(DetectionArray, 'ball_detections', 10)

    def yolo_callback(self, image):
        """Identify all the objects in the scene"""
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        # Run the model
        results = self.model(cv_image)

        # Create detecgtion message
        detections_msg = DetectionArray()

        # Process the results (draw bounding boxes on the image)
        for result in results:
            cv_image = result.plot()

            # Check if any of the detected objects is a person
            for box in result.boxes:
                x, y, w, h = box.xywh[0]  # center x, center y, width, height\
                self.get_logger().debug(f"Detected object at ({x}, {y}) with width {w} and height {h}")
                center_x = int(x)
                center_y = int(y)

                # Create detection message and add to array
                detection = Detection2D()
                detection.x = center_x
                detection.y = center_y
                detections_msg.detections.append(detection)
                
                # Draw red dot (circle) at center
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)  # -1 fills the circle
        
        self.detections_pub.publish(detections_msg)

        new_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        # publish
        self.image_pub.publish(new_msg)

def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()