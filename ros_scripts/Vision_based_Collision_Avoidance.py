import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import tensorflow as tf
import time
import numpy as np
from ultralytics import YOLO

class CombinedNode(Node):
    def __init__(self):
        super().__init__('combined_node')

        # Initialize YOLO model
        self.yolo_model = YOLO('best.pt')

        # Image subscription
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.image_bridge = CvBridge()

        # Autonomous movement
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.model = tf.keras.models.load_model('model.hdf5')
        self.cmd_vel_msg = Twist()
        self.obstacle_detected = False

    def image_callback(self, msg):
        try:
            cv_image = self.image_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return

        # Object Detection using YOLO
        results = self.yolo_model.predict(cv_image, classes=[0, 2])
        img = results[0].plot()

        # Show Results
        img_resized = cv2.resize(img, (400, 250))
        cv2.imshow('Detected object frame', img_resized)
        cv2.waitKey(1)

    def laser_callback(self, msg):
        # Extract a subset of laser range data and replace infinite values
        laser_data = np.array(msg.ranges[:60] + msg.ranges[-60:])
        laser_data[np.isinf(laser_data)] = 3.5

        # Use your pre-trained model to make predictions based on the processed laser data
        predictions = self.model.predict(np.expand_dims(laser_data, axis=0))

        # Check if the prediction indicates an obstacle (e.g., value > 0.5)
        if np.argmax(predictions) != 0:
            # Obstacle detected, change direction and continue moving
            self.cmd_vel_msg.linear.x = -0.3  # Continue moving forward
            self.cmd_vel_msg.angular.z = 0.3  # Rotate to avoid the obstacle
            self.obstacle_detected = True
        else:
            # No obstacle detected, continue moving forward
            self.cmd_vel_msg.linear.x = 0.5  # Adjust the linear speed as needed
            self.cmd_vel_msg.angular.z = 0.0
            self.obstacle_detected = False

        # Publish the computed velocity command
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)

    def move_forward(self):
        while rclpy.ok():
            if not self.obstacle_detected:
                self.cmd_vel_msg.linear.x = 0.2  # Continue moving forward
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                self.get_logger().info('Moving forward...')
            rclpy.spin_once(self)
        self.stop()

    def stop(self):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
        self.get_logger().info('Stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = CombinedNode()

    try:
        node.move_forward()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()

