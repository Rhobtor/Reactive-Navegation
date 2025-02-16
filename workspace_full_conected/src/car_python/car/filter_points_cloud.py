#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class GroundFilterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_ground_filter')
        # Parametro para definir la altura mínima (por ejemplo, 0.2 m)
        self.declare_parameter('min_z', -2)
        self.min_z = self.get_parameter('min_z').value

        # Suscribirse al tópico que publica la nube original (del plugin del LiDAR)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/scan_cloud',
            self.listener_callback,
            10
        )
        # Publicador para la nube filtrada
        self.publisher_ = self.create_publisher(PointCloud2, '/scan_cloud_filtered', 10)
        self.get_logger().info('Nodo de filtrado de suelo iniciado...')

    def listener_callback(self, msg: PointCloud2):
        filtered_points = []
        # Itera sobre los puntos de la nube (asumiendo campos x, y, z)
        for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            x, y, z = point
            if z > self.min_z:
                filtered_points.append([x, y, z])
        # Crear una nueva nube usando la cabecera original
        new_cloud = pc2.create_cloud_xyz32(msg.header, filtered_points)
        self.publisher_.publish(new_cloud)
        self.get_logger().info(f'Publicados {len(filtered_points)} puntos filtrados')

def main(args=None):
    rclpy.init(args=args)
    node = GroundFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
