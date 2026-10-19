#!/usr/bin/env python3
"""Expose motor commands and virtual track encoders through ROS."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64, Int32MultiArray, UInt32MultiArray


class MotorInterface(Node):
    def __init__(self):
        super().__init__('robot_motor_interface')
        self.declare_parameter('max_motor_speed_m_s', 0.4)
        self.declare_parameter('tracks_separation_m', 0.81)
        self.declare_parameter('ticks_per_meter', 1000.0)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('track_speed_timeout_s', 0.8)
        self.speed = float(self.get_parameter('max_motor_speed_m_s').value)
        self.separation = float(self.get_parameter('tracks_separation_m').value)
        self.ticks_per_meter = float(self.get_parameter('ticks_per_meter').value)
        self.timeout = float(self.get_parameter('command_timeout_s').value)
        self.track_timeout = float(self.get_parameter('track_speed_timeout_s').value)
        if min(self.speed, self.separation, self.ticks_per_meter,
               self.timeout, self.track_timeout) <= 0:
            raise ValueError('Motor interface parameters must be positive')

        self.velocity_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.encoder_pub = self.create_publisher(UInt32MultiArray, '/encoders', 10)
        self.create_subscription(Int32MultiArray, '/motors', self.on_motors, 10)
        self.create_subscription(
            Float64, '/robot/left_track/speed',
            lambda message: self.on_track_speed(0, message), 10
        )
        self.create_subscription(
            Float64, '/robot/right_track/speed',
            lambda message: self.on_track_speed(1, message), 10
        )
        self.last_command = None
        self.command = Twist()
        self.track_speeds = [0.0, 0.0]
        self.track_stamps = [None, None]
        self.encoder_ticks = [0.0, 0.0]
        self.last_update = self.get_clock().now()
        self.create_timer(0.02, self.publish_state)

    def on_motors(self, message):
        if len(message.data) != 2:
            self.get_logger().warning('/motors requires exactly two values: [left, right]')
            return
        left, right = (max(-100, min(100, int(value))) for value in message.data)
        left_speed = self.speed * left / 100.0
        right_speed = self.speed * right / 100.0
        self.command = Twist()
        self.command.linear.x = (left_speed + right_speed) / 2.0
        self.command.angular.z = (right_speed - left_speed) / self.separation
        self.last_command = self.get_clock().now()

    def accumulate_until(self, now):
        if now < self.last_update:
            self.last_update = now
            self.track_stamps = [None, None]
            self.encoder_ticks = [0.0, 0.0]
            return
        for side, stamp in enumerate(self.track_stamps):
            if stamp is None:
                continue
            active_start = max(self.last_update.nanoseconds, stamp.nanoseconds)
            active_end = min(
                now.nanoseconds,
                stamp.nanoseconds + round(self.track_timeout * 1e9),
            )
            if active_end > active_start:
                seconds = (active_end - active_start) * 1e-9
                self.encoder_ticks[side] += (
                    self.track_speeds[side] * seconds * self.ticks_per_meter
                )
        self.last_update = now

    def on_track_speed(self, side, message):
        if not math.isfinite(message.data):
            self.get_logger().warning('Ignoring non-finite track speed')
            return
        now = self.get_clock().now()
        self.accumulate_until(now)
        self.track_speeds[side] = message.data
        self.track_stamps[side] = now

    def publish_state(self):
        now = self.get_clock().now()
        self.accumulate_until(now)

        age = None if self.last_command is None else (now - self.last_command).nanoseconds * 1e-9
        self.velocity_pub.publish(
            self.command if age is not None and 0 <= age <= self.timeout else Twist()
        )
        encoders = UInt32MultiArray()
        encoders.data = [round(ticks) % (1 << 32) for ticks in self.encoder_ticks]
        self.encoder_pub.publish(encoders)


def main():
    rclpy.init()
    node = MotorInterface()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.velocity_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
