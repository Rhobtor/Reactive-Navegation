#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class ScanToCloud(Node):
    def __init__(self):
        super().__init__('scan_to_cloud')
        self.laser_proj = LaserProjection()
        # Suscribirse al topic 'scan' (ajusta el nombre si es distinto)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        # Publicar en el topic 'cloud'
        self.publisher = self.create_publisher(PointCloud2, 'cloud', 10)
        self.get_logger().info("Nodo scan_to_cloud iniciado")

    def scan_callback(self, msg: LaserScan):
        # Convertir el LaserScan a PointCloud2
        cloud = self.laser_proj.projectLaser(msg)
        self.publisher.publish(cloud)
        self.get_logger().debug("Publicada nube de puntos")

def main(args=None):
    rclpy.init(args=args)
    node = ScanToCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
