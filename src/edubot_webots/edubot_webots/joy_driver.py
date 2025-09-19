#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Contrôleur manette prêt.")

    def joy_callback(self, msg):
        # Indices des boutons
        BUTTON_R2 = 7  # sécurité
        BUTTON_L2 = 6  # turbo

        # Vérification des boutons
        r2_pressed = msg.buttons[BUTTON_R2] if len(msg.buttons) > BUTTON_R2 else 0
        l2_pressed = msg.buttons[BUTTON_L2] if len(msg.buttons) > BUTTON_L2 else 0

        # Axes analogiques
        axis_left_right = msg.axes[0] if len(msg.axes) > 0 else 0.0
        axis_up_down = msg.axes[1] if len(msg.axes) > 1 else 0.0

        # Flèches directionnelles (boutons 13, 14, 15, 16)
        dpad_left = msg.buttons[15] if len(msg.buttons) > 13 else 0
        dpad_right = msg.buttons[16] if len(msg.buttons) > 14 else 0
        dpad_up = msg.buttons[13] if len(msg.buttons) > 15 else 0
        dpad_down = msg.buttons[14] if len(msg.buttons) > 16 else 0

        twist = Twist()

        if r2_pressed:
            # Combinaison des entrées analogiques et des flèches directionnelles
            linear = axis_up_down + (dpad_up - dpad_down) 
            angular = axis_left_right + (dpad_left - dpad_right)

            # Vitesse normale
            speed = 0.2
            turn_speed = 0.2

            # Mode turbo
            if l2_pressed:
                speed *= 2.0
                turn_speed *= 2.0

            twist.linear.x = speed * linear
            twist.angular.z = turn_speed * angular
        else:
            # R2 non pressé => arrêt d’urgence
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
