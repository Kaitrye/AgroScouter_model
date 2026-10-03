#!/usr/bin/env python3
"""Heartbeat and timeout for velocity commands, using simulation time."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CommandRelay(Node):
    def __init__(self):
        super().__init__('robot_command_relay')
        self.declare_parameter('timeout', 0.5)
        self.last_stamp = None
        self.command = Twist()
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.receive, 10)
        self.create_timer(0.05, self.tick)

    def receive(self, message):
        self.command = message
        self.last_stamp = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        timeout = self.get_parameter('timeout').value
        age = None if self.last_stamp is None else (now - self.last_stamp).nanoseconds * 1e-9
        self.publisher.publish(self.command if age is not None and 0 <= age <= timeout else Twist())


def main():
    rclpy.init()
    node = CommandRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
