import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1' 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tensorflow as tf
import time
import numpy as np

class AutonomousMovementNode(Node):
    def __init__(self):
        super().__init__('autonomous_movement_node')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.model = tf.keras.models.load_model('model.hdf5')
        self.cmd_vel_msg = Twist()
        self.obstacle_detected = False

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
        self.publisher.publish(self.cmd_vel_msg)

    def move_forward(self):
        while rclpy.ok():
            if not self.obstacle_detected:
                self.cmd_vel_msg.linear.x = 0.2  # Continue moving forward
                self.cmd_vel_msg.angular.z = 0.0
                self.publisher.publish(self.cmd_vel_msg)
                self.get_logger().info('Moving forward...')
            rclpy.spin_once(self)
        self.stop()

    def stop(self):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.publisher.publish(self.cmd_vel_msg)
        self.get_logger().info('Stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMovementNode()

    try:
        node.move_forward()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()

