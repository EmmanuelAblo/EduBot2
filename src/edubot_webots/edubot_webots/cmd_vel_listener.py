#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from controller import Robot
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
import tf_transformations

import math
import numpy as np

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')

        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        self.get_logger().info("Conecté au robot Edubot2 dans Webots !")

        # ROS Publishers
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Motors
        self.left_motor = self.robot.getDevice('left_wheel_joint')
        self.right_motor = self.robot.getDevice('right_wheel_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Encoders
        self.left_encoder = self.robot.getDevice('left_wheel_joint_sensor')
        self.right_encoder = self.robot.getDevice('right_wheel_joint_sensor')
        self.left_encoder.enable(self.time_step)
        self.right_encoder.enable(self.time_step)

        self.prev_left = 0.0
        self.prev_right = 0.0

        # Physique
        self.wheel_radius = 0.0762
        self.wheel_base = 0.413  # pris du URDF → ≈ 0.387 m

        # LiDAR
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(self.time_step)
        self.lidar.enablePointCloud()
        self.lidar_resolution = self.lidar.getHorizontalResolution()
        self.lidar_fov = self.lidar.getFov()

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        left_speed = (linear - angular * self.wheel_base / 2.0) / self.wheel_radius
        right_speed = (linear + angular * self.wheel_base / 2.0) / self.wheel_radius
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def wait_for_encoder_values(self):
        while math.isnan(self.left_encoder.getValue()) or math.isnan(self.right_encoder.getValue()):
            self.robot.step(self.time_step)

        self.prev_left = self.left_encoder.getValue()
        self.prev_right = self.right_encoder.getValue()
        self.get_logger().info("Encodeurs initialisés.")


    def publish_scan(self):
        ranges = list(self.lidar.getRangeImage())
        if len(ranges) == self.lidar_resolution:
            ranges.append(ranges[-1])

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = -self.lidar_fov / 2.0
        msg.angle_max = self.lidar_fov / 2.0
        msg.angle_increment = self.lidar_fov / self.lidar_resolution
        msg.time_increment = (self.time_step / 1000.0) / (self.lidar_resolution + 1)
        msg.scan_time = self.time_step / 1000.0
        msg.range_min = self.lidar.getMinRange()
        msg.range_max = self.lidar.getMaxRange()
        msg.ranges = ranges
        self.scan_publisher.publish(msg)

    def update_odometry(self):
        left_pos = self.left_encoder.getValue()
        right_pos = self.right_encoder.getValue()

        delta_left = (left_pos - self.prev_left) * self.wheel_radius
        delta_right = (right_pos - self.prev_right) * self.wheel_radius
        self.prev_left = left_pos
        self.prev_right = right_pos

        delta_theta = (delta_right - delta_left) / self.wheel_base
        delta_s = (delta_right + delta_left) / 2.0

        theta_old = self.theta
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # normalisation

        self.x += delta_s * math.cos(theta_old)
        self.y += delta_s * math.sin(theta_old)


        # TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        dt = self.time_step / 1000.0
        odom.twist.twist.linear.x = delta_s / dt
        odom.twist.twist.angular.z = delta_theta / dt

        self.odom_pub.publish(odom)
        
    def spin(self):
        self.wait_for_encoder_values()

        while self.robot.step(self.time_step) != -1:
            # /clock
            sim_time = self.robot.getTime()
            clock_msg = Clock()
            clock_msg.clock.sec = int(sim_time)
            clock_msg.clock.nanosec = int((sim_time - int(sim_time)) * 1e9)
            self.clock_pub.publish(clock_msg)

            self.update_odometry()
            self.publish_scan()
            rclpy.spin_once(self, timeout_sec=0.0)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
