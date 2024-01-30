import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with the actual camera topic name
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return

        # Process the cv_image (OpenCV) as needed
        # For example, you can perform obstacle avoidance logic here

        # Resize the image to 200x200
        cv_image_resized = cv2.resize(cv_image, (400, 250))

        # Display the resized image (for demonstration purposes)
        cv2.imshow('Camera Image', cv_image_resized)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

