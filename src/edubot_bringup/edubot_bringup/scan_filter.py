#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.get_logger().info("Filtre actif : extraction des 180° à l'avant du robot (arrière du lidar_link)")

    def callback(self, msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = msg.ranges
        intensities = msg.intensities
        total_points = len(ranges)

        # On veut 90° à 270° dans le repère du LiDAR (avant du robot)
        start_angle = math.pi / 2   # 90°
        end_angle = 3 * math.pi / 2 # 270°

        # Calcul des indices correspondants
        start_index = int((start_angle - angle_min) / angle_inc) % total_points
        end_index = int((end_angle - angle_min) / angle_inc) % total_points

        # Correction pour éviter dépassement
        start_index = max(0, start_index)
        end_index = min(total_points - 1, end_index)

        # Gestion de la plage continue (boucle de -180° à 180°)
        if start_index <= end_index:
            new_ranges = ranges[start_index:end_index + 1]
            new_intensities = intensities[start_index:end_index + 1] if intensities else []
        else:
            new_ranges = ranges[start_index:] + ranges[:end_index + 1]
            new_intensities = (intensities[start_index:] + intensities[:end_index + 1]) if intensities else []

        # Ajustement de angle_increment pour refléter la nouvelle plage
        expected_angle_range = end_angle - start_angle if end_angle > start_angle else (2 * math.pi + end_angle - start_angle)
        new_angle_increment = expected_angle_range / len(new_ranges) if len(new_ranges) > 0 else angle_inc

        # Création du nouveau message
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.angle_min = start_angle
        new_msg.angle_max = end_angle
        new_msg.angle_increment = new_angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        new_msg.ranges = new_ranges
        new_msg.intensities = new_intensities

        self.pub.publish(new_msg)

def main():
    rclpy.init()
    node = ScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()