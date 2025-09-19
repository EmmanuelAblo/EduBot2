#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from pymodbus.client import ModbusTcpClient
from pymodbus.payload import BinaryPayloadDecoder , BinaryPayloadBuilder
from pymodbus.constants import Endian
from sensor_msgs.msg import Joy

class JoyDriver(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            1)
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.twist_callback,
            1)

        self.pubish_zero = False
        self.dead_man_switch = 0

    def twist_callback(self, msg ):
        if self.dead_man_switch == 0:
            self.publisher.publish(msg)

    def joy_callback(self, msg ):
        rightleft_anlog = msg.axes[0] 
        updown_analog= msg.axes[1]

        updown = msg.axes[7]
        rightleft = msg.axes[6]

        if abs(rightleft_anlog) > 0.2:
            rightleft = rightleft_anlog
        if abs(updown_analog) > 0.2:
            updown = updown_analog

        turbo = msg.buttons[6]
        slow = msg.buttons[5]
        self.dead_man_switch = msg.buttons[7]
        cmd = Twist()
        if self.dead_man_switch == 1:
            self.pubish_zero = True
            v = 0.4
            w = 0.6
            if turbo == 1:
                v = 0.8
                w = 1
            elif slow:
                v = 0.2
                w = 0.5
            cmd.linear.x =  v * updown
            cmd.angular.z =  w *rightleft
            self.publisher.publish(cmd)
        else:
            if self.pubish_zero:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.publisher.publish(cmd)
                self.pubish_zero = False
        


def main(args=None):
    rclpy.init(args=args)
    node = JoyDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
