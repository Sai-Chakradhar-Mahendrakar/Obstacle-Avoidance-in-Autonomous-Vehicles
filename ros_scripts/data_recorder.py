import rclpy
from rclpy.node import Node
import csv
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

filename = "data.csv"
csvfile = open(filename, "w")
csvwriter = csv.writer(csvfile)
dataset = []

class DataRecorder(Node):

    def _init_(self):
        super()._init_('data_recorder')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        self.ts = ApproximateTimeSynchronizer([self.scan_sub, self.cmd_sub], 10, 0.1)
        self.ts.registerCallback(self.data_callback)

    def scan_callback(self, msg):
        self.laser_data = msg.ranges[:60] + msg.ranges[-60:]

    def cmd_callback(self, msg):
        self.twist_data = (msg.linear.x, msg.angular.z)

    def data_callback(self, laser, twist):
        data = self.laser_data + self.twist_data
        dataset.append(data)

def main(args=None):
    rclpy.init(args=args)
    data_recorder = DataRecorder()

    counter = 0
    rate = data_recorder.create_rate(500)
    while rclpy.ok():
        rate.sleep()
        counter += 1

        if counter > 1e5:
            break

    csvfile.close()
    data_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()