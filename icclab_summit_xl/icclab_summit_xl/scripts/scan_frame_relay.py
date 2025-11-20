#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameRelay(Node):
    def __init__(self):
        super().__init__('scan_frame_relay')
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        
    def callback(self, msg):
        msg.header.frame_id = 'front_laser_base_link'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanFrameRelay()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
