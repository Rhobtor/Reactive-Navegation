#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
from rclpy.duration import Duration

class DebugTFNode(Node):
    def __init__(self):
        super().__init__('debug_tf_node')
        # Nos suscribimos al tópico /scan
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Creamos el buffer y listener de tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
    def scan_callback(self, scan_msg: LaserScan):
        # Intentamos obtener la transformación desde 'odom' a 'base_link' para el timestamp del scan
        try:
            # Espera hasta 1 segundo para obtener la transformación
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', scan_msg.header.stamp, Duration(seconds=1.0))
            self.get_logger().info(
                f"Transformación obtenida: traslación: {transform.transform.translation}, rotación: {transform.transform.rotation}")
        except Exception as e:
            self.get_logger().warn(f"Fallo en lookup transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DebugTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
