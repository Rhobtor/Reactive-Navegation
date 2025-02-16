#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point
import yaml

class WaypointMarkerNode(Node):
    def __init__(self):
        super().__init__('waypoint_marker_node')
        self.marker_pub = self.create_publisher(Marker, 'waypoint_marker', 1)
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.waypoint_callback,
            10
        )
        self.get_logger().info("Nodo de marcaje de waypoints iniciado.")

    def save_waypoint(self, point, filename="waypoints.yaml"):
        # Se crea un diccionario con las coordenadas del punto
        data = {'x': point.x, 'y': point.y, 'z': point.z}
        # Se abre (o crea) el archivo y se añade el nuevo punto
        with open(filename, "a") as file:
            yaml.dump([data], file, default_flow_style=False)

    def publish_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "map"  # Asegúrate de que este frame coincida con el de tu mapa
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0  # Si planeas varios, incrementa el ID para cada punto
        marker.type = Marker.SPHERE  # Puedes cambiar el tipo (SPHERE, ARROW, etc.)
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3  # Ajusta el tamaño según tus necesidades
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0  # Rojo para resaltar
        
    def waypoint_callback(self, msg: PoseStamped):
            # Extraer la posición del mensaje PoseStamped
            point = msg.pose.position
            # Publicar el marcador para visualizar en RViz
            self.publish_marker(point)
            # Guardar el waypoint para usos futuros
            self.save_waypoint(point)
            self.get_logger().info(f"Waypoint guardado: [{point.x:.2f}, {point.y:.2f}, {point.z:.2f}]")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()