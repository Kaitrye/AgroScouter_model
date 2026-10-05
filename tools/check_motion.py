#!/usr/bin/env python3
"""Run a short simulation-only motion / camera check after starting sim.launch.py."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, Imu
from rclpy.qos import qos_profile_sensor_data


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class MotionCheck(Node):
    def __init__(self):
        super().__init__('robot_sim_motion_check')
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        self.latest=None;self.images=0;self.scans=0;self.imus=0;self.start_time=None;self.forward_end=None;self.turn_start=None;self.turn_end=None
        self.initial=None;self.result=None;self.started_wall=time.monotonic()
        self.create_subscription(Odometry,'/ground_truth/odom',self.receive,10)
        self.create_subscription(Image,'/camera/image_raw',self.image,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.create_subscription(Imu,'/imu/data',self.imu,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)

    def receive(self,msg):
        self.latest=msg

    def image(self,msg):
        if msg.width>0 and msg.height>0 and len(msg.data)>0:
            self.images+=1

    def scan(self,msg):
        if len(msg.ranges)>0 and msg.range_max>msg.range_min:
            self.scans+=1

    def imu(self,msg):
        q=msg.orientation
        if msg.header.frame_id=='imu_link' and sum(v*v for v in (q.x,q.y,q.z,q.w))>.5:
            self.imus+=1

    def pose(self):
        p=self.latest.pose.pose
        return (p.position.x,p.position.y,yaw(p.orientation))

    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        cmd=Twist()
        if self.latest is None:
            self.publisher.publish(cmd)
            return
        if self.start_time is None:
            self.start_time=now;self.initial=self.pose()
        elapsed=now-self.start_time
        if elapsed<1:
            pass
        elif elapsed<5:
            cmd.linear.x=.15
        elif elapsed<6:
            self.forward_end=self.pose()
        elif elapsed<10:
            if self.turn_start is None:self.turn_start=self.pose()
            cmd.angular.z=.3
        elif elapsed<11:
            self.turn_end=self.pose()
        elif elapsed>=13:
            end=self.pose()
            distance=math.hypot(self.forward_end[0]-self.initial[0],self.forward_end[1]-self.initial[1])
            angle=abs(math.atan2(math.sin(self.turn_end[2]-self.turn_start[2]),math.cos(self.turn_end[2]-self.turn_start[2])))
            stopped_drift=math.hypot(end[0]-self.turn_end[0],end[1]-self.turn_end[1])
            stopped_yaw=abs(math.atan2(math.sin(end[2]-self.turn_end[2]),math.cos(end[2]-self.turn_end[2])))
            forward_x=self.forward_end[0]-self.initial[0]
            self.result=distance>.03 and forward_x>.02 and angle>.05 and stopped_drift<.05 and stopped_yaw<.1 and self.images>0 and self.scans>0 and self.imus>0
            self.get_logger().info(f'Forward: {distance:.3f} m, dX={forward_x:.3f}; turn: {angle:.3f} rad; stop drift: {stopped_drift:.3f} m / {stopped_yaw:.3f} rad; camera frames: {self.images}; lidar scans: {self.scans}; IMU messages: {self.imus}; PASS={self.result}')
        self.publisher.publish(cmd)


def main():
    rclpy.init();node=MotionCheck()
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-node.started_wall>90:
                node.get_logger().error('Timeout: check that simulation is running, unpaused, and ROS bridges are active.')
                node.result=False
    except KeyboardInterrupt:
        node.result=False
    finally:
        if rclpy.ok():node.publisher.publish(Twist())
        passed=node.result is True;node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__=='__main__':
    main()
