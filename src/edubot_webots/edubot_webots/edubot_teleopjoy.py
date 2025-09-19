#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import time

class webots_controller(Node):
    def __init__(self):
        super().__init__('webots_controller')

        # Paramètres de vitesse
        self.MAX_LIN = 0.2
        self.MAX_ANGULAR = 0.4
        self.MAX_LIN_TURBO = 0.5
        self.MAX_ANGULAR_TURBO = 0.7
        self.DEAD_ZONE = 0.07

        # Boutons
        self.R2_BUTTON = 7
        self.L2_BUTTON = 6
        self.UP_ARROW = 13
        self.DOWN_ARROW = 14
        self.RIGHT_ARROW = 16
        self.LEFT_ARROW = 15
        self.FORWARD_AXIS = 1
        self.ROTATION_AXIS = 0

        # Initialisation Pygame
        pygame.init()
        pygame.joystick.init()
        time.sleep(0.2)

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("Aucune manette connectée.")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Manette connectée : {self.joystick.get_name()}")

        # Publisher ROS 2 sur /cmd_vel
        self.edubot_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.control_loop)
        self.get_logger().info("teleop_joy has been started")

    def control_loop(self):
        pygame.event.pump()

        Vx, Wz = 0.0, 0.0
        r2 = self.joystick.get_button(self.R2_BUTTON)
        l2 = self.joystick.get_button(self.L2_BUTTON)
        up = self.joystick.get_button(self.UP_ARROW)
        down = self.joystick.get_button(self.DOWN_ARROW)
        right = self.joystick.get_button(self.RIGHT_ARROW)
        left = self.joystick.get_button(self.LEFT_ARROW)

        forward_axis = self.joystick.get_axis(self.FORWARD_AXIS)
        rotation_axis = self.joystick.get_axis(self.ROTATION_AXIS)

        # Déplacement avec flèches
        if up and r2:
            Vx = self.MAX_LIN_TURBO if l2 else self.MAX_LIN
        elif down and r2:
            Vx = -self.MAX_LIN_TURBO if l2 else -self.MAX_LIN
        elif right and r2:
            Wz = -self.MAX_ANGULAR_TURBO if l2 else -self.MAX_ANGULAR
        elif left and r2:
            Wz = self.MAX_ANGULAR_TURBO if l2 else self.MAX_ANGULAR
        elif r2:
            if abs(forward_axis) > self.DEAD_ZONE:
                Vx = -forward_axis * (self.MAX_LIN_TURBO if l2 else self.MAX_LIN)
            if abs(rotation_axis) > self.DEAD_ZONE:
                Wz = -rotation_axis * (self.MAX_ANGULAR_TURBO if l2 else self.MAX_ANGULAR)

        msg = Twist()
        msg.linear.x = Vx
        msg.angular.z = Wz
        self.edubot_pub.publish(msg)

        # self.get_logger().info(f"Cmd ROS envoyée - Vx: {Vx:.2f}, Wz: {Wz:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = webots_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
