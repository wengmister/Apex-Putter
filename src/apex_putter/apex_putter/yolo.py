import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class YoloNode(Node):
    """
    Use Yolo to identify scene objects

    Subscribes
    ----------
    image (sensor_msgs/msg/Image) - The input image

    Publishes
    ---------
    new_image (sensor_msgs/msg/Image) - The image with the detections

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
        self.pub = self.create_publisher(Image, 'image_yolo', 10)

    def yolo_callback(self, image):
        """Identify all the objects in the scene"""
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        # Run the model
        results = self.model(cv_image)
        # Get the result and draw it on an OpenCV image
        frame = results[0].plot()
        new_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # publish
        self.pub.publish(new_msg)

def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()