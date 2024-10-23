"""
TU Delft Swarm Lab - Human Swarm Interaction Project
Author: Alexander James Becoy @alexanderjamesbecoy
Date: 2024-10-23
"""

import rclpy
from rclpy.node import Node

from topic_interface.msg import ControllerCommand
from human_swarm_interaction_interfaces.msg import PoseKeypointsStamped

from swarm_operation.helper_classes import SwarmController
import numpy as np

class WristControllerNode(Node):
    """ TODO: Single drone control
    [ ] At startup, move singular drone to center of cage
    [ ] position control := Kp(x_des - x; y_des - y)
    """

    def __init__(self):
        super().__init__('wrist_controller_node')
        self.initialize_keypoints_subscriber()
        # self.initialize_controller_publisher()
        self.get_logger().info('Wrist Controller Node Initialized')

    def keypoints_callback(self, msg):
        keypoints = msg.keypoints
        self.get_logger().info('Received {} keypoints'.format(len(keypoints)))
        # self.publish_controller_command()

    def initialize_keypoints_subscriber(self):
        self.declare_parameter('keypoints_topic_name', '/keypoints')
        self.declare_parameter('keypoints_topic_queue_size', 10)

        keypoints_topic_name = self.get_parameter('keypoints_topic_name').value
        keypoints_topic_queue_size = self.get_parameter('keypoints_topic_queue_size').value
        self.keypoints_subscriber = self.create_subscription(
            PoseKeypointsStamped,
            keypoints_topic_name,
            self.keypoints_callback,
            keypoints_topic_queue_size
        )
        self.get_logger().info('Subscribed to topic: {} with queue size: {}'.format(
            keypoints_topic_name, keypoints_topic_queue_size))
        

def main(args=None):
    rclpy.init(args=args)
    wrist_controller_node = WristControllerNode()
    rclpy.spin(wrist_controller_node)
    wrist_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
