#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarTestNode(Node):
    def __init__(self):
        super().__init__('lidar_test_node')
        # Subscriu al topic del LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.subscription  # evita warning de variable no usada
        self.get_logger().info("Lidar Test Node Initialized")

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Distància mínima i angle corresponent
        min_distance = min(ranges)
        min_index = ranges.index(min_distance)
        min_angle = angle_min + min_index * angle_increment

        # Funció per obtenir distància a un angle específic (aproximant a l'índex més proper)
        def distance_at_angle(target_angle):
            index = int(round((target_angle - angle_min) / angle_increment))
            index = max(0, min(index, len(ranges)-1))  # assegura que estigui dins de rang
            return ranges[index]

        dist_0 = distance_at_angle(0.0)
        dist_90 = distance_at_angle(math.pi / 2)
        dist_neg90 = distance_at_angle(-math.pi / 2)

        # Mostrem per pantalla
        self.get_logger().info(f"Dist. mínima: {min_distance:.2f} m a angle {math.degrees(min_angle):.1f}º")
        self.get_logger().info(f"Dist. 0º: {dist_0:.2f} m, 90º: {dist_90:.2f} m, -90º: {dist_neg90:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = LidarTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
