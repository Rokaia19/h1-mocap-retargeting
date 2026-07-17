#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from array import array
from std_msgs.msg import Float64

joint_cmd_topics = [
    "/h1/left_hip_yaw_joint/cmd_pos",
    "/h1/left_hip_pitch_joint/cmd_pos",
    "/h1/left_hip_roll_joint/cmd_pos",
    "/h1/left_knee_joint/cmd_pos",
    "/h1/left_ankle_pitch_joint/cmd_pos",
    "/h1/left_ankle_roll_joint/cmd_pos",
    "/h1/right_hip_yaw_joint/cmd_pos",
    "/h1/right_hip_pitch_joint/cmd_pos",
    "/h1/right_hip_roll_joint/cmd_pos",
    "/h1/right_knee_joint/cmd_pos",
    "/h1/right_ankle_pitch_joint/cmd_pos",
    "/h1/right_ankle_roll_joint/cmd_pos",
    "/h1/torso_joint/cmd_pos",
    "/h1/left_shoulder_pitch_joint/cmd_pos",
    "/h1/left_shoulder_roll_joint/cmd_pos",
    "/h1/left_shoulder_yaw_joint/cmd_pos",
    "/h1/left_elbow_joint/cmd_pos",
    "/h1/left_wrist_roll_joint/cmd_pos",
    "/h1/left_wrist_pitch_joint/cmd_pos",
    "/h1/left_wrist_yaw_joint/cmd_pos",
    "/h1/right_shoulder_pitch_joint/cmd_pos",
    "/h1/right_shoulder_roll_joint/cmd_pos",
    "/h1/right_shoulder_yaw_joint/cmd_pos",
    "/h1/right_elbow_joint/cmd_pos",
    "/h1/right_wrist_roll_joint/cmd_pos",
    "/h1/right_wrist_pitch_joint/cmd_pos",
    "/h1/right_wrist_yaw_joint/cmd_pos"
]

class StaticJointPublisher(Node):
    """A ROS2 node that publishes static joint states to solve Exercise 2.1."""
    def __init__(self):
        super().__init__('static_joint_publisher')
        self.pub = []
        for topic in joint_cmd_topics:
            self.pub.append(self.create_publisher(Float64, topic, 10))
            
        self.declare_parameter('mode', 'upper_body')
        
        mode = self.get_parameter('mode').get_parameter_value().string_value
        
        modes = {
            'upper_body': self.upper_body_angles_pub,
        }
        
        if mode in modes:
            modes[mode]
        else:
            self.get_logger().error(f"Unknown mode: '{mode}'. Choose from: {list(modes.keys())}")

	# you can put code here
    def upper_body_angles_pub(self):
        print("Setting all angles of the upper body to 0.3 rad, all other at zero")
        """# Splice the joind_cmd_topics into upper_body_cmd_topics
        self.upper_body_cmd_topics = joint_cmd_topics[12:]
        for pub in joint_cmd_topics[:13]:
            pub.publish(0.0)
        for pub in self.upper_body_cmd_topics:
            pub.publish(0.3)    """
        pass
    

def main(args=None):
    rclpy.init(args=args)
    node = StaticJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
