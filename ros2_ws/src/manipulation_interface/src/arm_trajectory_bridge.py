#!/usr/bin/env python3
"""
Simple bridge: subscribe to JointTrajectory messages and forward them
to the configured arm controller command topic.

This allows MoveIt2 to publish planned trajectories (to e.g. /arm_trajectory)
and have this node forward them to `/arm_controller/command` which the simulator
or ros2_control controller can consume.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class ArmTrajectoryBridge(Node):
    def __init__(self):
        super().__init__('arm_trajectory_bridge')
        self.declare_parameter('input_topic', 'arm_trajectory')
        self.declare_parameter('output_topic', 'arm_controller/command')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(JointTrajectory, output_topic, 10)
        self.sub = self.create_subscription(JointTrajectory, input_topic, self.cb, 10)

        self.get_logger().info(f'Bridge forwarding JointTrajectory: {input_topic} -> {output_topic}')

    def cb(self, msg: JointTrajectory):
        # Optionally validate/modify trajectory here
        self.get_logger().debug(f'Received trajectory for {len(msg.joint_names)} joints, forwarding')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTrajectoryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
