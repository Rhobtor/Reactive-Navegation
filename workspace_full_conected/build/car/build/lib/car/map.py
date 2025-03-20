#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
import math

class NavigationMapBuilder(Node):
    def __init__(self):
        super().__init__('navigation_map_builder')
        # Subscripción al tópico con nodos filtrados
        self.subscription = self.create_subscription(
            PoseArray,
            'filtered_navigation_nodes',
            self.pose_callback,
            10)
        # Publicador para visualizar el mapa con markers en RViz
        self.marker_pub = self.create_publisher(MarkerArray, 'navigation_map_markers', 10)
        # Publicador para enviar el mapa como PoseArray
        self.map_pose_pub = self.create_publisher(PoseArray, 'navigation_map_points', 10)
        # Lista para almacenar nodos del mapa
        self.map_nodes = []  
        # Umbral para considerar que dos puntos son iguales (en metros)
        self.node_distance_threshold = 2.0

    def pose_callback(self, msg: PoseArray):
        updated = False
        for pose in msg.poses:
            # Si el nuevo punto no está ya cerca de uno almacenado, se añade
            if not self.is_pose_in_map(pose):
                self.map_nodes.append(pose)
                updated = True
        if updated:
            self.publish_map()

    def is_pose_in_map(self, new_pose: Pose):
        for pose in self.map_nodes:
            dx = new_pose.position.x - pose.position.x
            dy = new_pose.position.y - pose.position.y
            if math.hypot(dx, dy) < self.node_distance_threshold:
                return True
        return False

    def publish_map(self):
        # Publicar MarkerArray para visualización en RViz
        marker_array = MarkerArray()
        for i, pose in enumerate(self.map_nodes):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "navigation_map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        
        # Publicar el mapa completo como PoseArray
        map_pose = PoseArray()
        map_pose.header.frame_id = "map"
        map_pose.header.stamp = self.get_clock().now().to_msg()
        map_pose.poses = self.map_nodes
        self.map_pose_pub.publish(map_pose)
        
        self.get_logger().info(f"Mapa actualizado: {len(self.map_nodes)} nodos.")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationMapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
