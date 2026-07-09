#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from array import array

class StaticJointPublisher(Node):
    """A ROS2 node that publishes static joint states to solve Exercise 2.1."""
    def __init__(self):
        super().__init__('static_joint_publisher')

	# you can put code here


def main(args=None):
    rclpy.init(args=args)
    node = StaticJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
