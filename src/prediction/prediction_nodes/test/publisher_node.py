#!/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from autoware_auto_msgs.msg import TrackedObjects

import random

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = rclpy.qos.QoSProfile(durability='TRANSIENT_LOCAL', reliability='RELIABLE')
        self._publisher = self.create_publisher(TrackedObjects, 'tracked_objects', qos_profile)
        self.timer_period = 1 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = TrackedObjects()
        msg.header.stamp.sec = int(self.i * self.timer_period)
        msg.header.stamp.nanosec = random.randint(0, 4294967295)
        self._publisher.publish(msg)
        self.get_logger().info(f'Publishing "{self.i}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
