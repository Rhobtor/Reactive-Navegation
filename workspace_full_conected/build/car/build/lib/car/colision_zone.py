#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker

class VirtualCollisionZone(Node):
    def __init__(self):
        super().__init__('virtual_collision_zone')
        # Parámetro: radio de la zona de colisión virtual (en metros)
        self.declare_parameter("collision_radius", 2.0)
        self.collision_radius = self.get_parameter("collision_radius").value
        
        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.obstacle_callback, 10)
        
        # Publicador de la señal de colisión (Bool)
        self.collision_pub = self.create_publisher(Bool, '/virtual_collision', 10)
        # Publicador de Marker para visualizar la zona de colisión
        self.marker_pub = self.create_publisher(Marker, '/collision_zone_marker', 10)
        
        self.current_pose = None  # Pose actual del robot
        self.obstacle_poses = []  # Lista de obstáculos (PoseArray)

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.check_collision()

    def obstacle_callback(self, msg: PoseArray):
        self.obstacle_poses = msg.poses
        self.check_collision()

    def check_collision(self):
        if self.current_pose is None or not self.obstacle_poses:
            return
        collision = False
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        # Se calcula la distancia de cada obstáculo al robot
        for pose in self.obstacle_poses:
            dx = pose.position.x - robot_x
            dy = pose.position.y - robot_y
            if math.hypot(dx, dy) < self.collision_radius:
                collision = True
                break
        # Publicar el Bool indicando colisión virtual
        bool_msg = Bool()
        bool_msg.data = collision
        self.collision_pub.publish(bool_msg)
        # Publicar el Marker para la zona de colisión
        marker = Marker()
        marker.header.frame_id = "map"  # Usamos base_link para que el marcador se mueva con el robot
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "collision_zone"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        # Colocar el marcador centrado en base_link (0,0,0) y elevarlo ligeramente para verlo en RViz
        marker.pose.position.x = self.current_pose.position.x
        marker.pose.position.y = self.current_pose.position.y
        marker.pose.position.z = 0.01  # Un poco por encima del suelo
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # Escala: diámetro = 2*radio y altura pequeña
        marker.scale.x = 2 * self.collision_radius
        marker.scale.y = 2 * self.collision_radius
        marker.scale.z = 0.02
        # Color: rojo si hay colisión, verde si está libre
        if collision:
            marker.color.r = 1.0
            marker.color.g = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5  # Semitransparente
        self.marker_pub.publish(marker)
        if collision:
            self.get_logger().warn("¡Zona de colisión virtual activada!")
        else:
            self.get_logger().info("Zona segura.")

def main(args=None):
    rclpy.init(args=args)
    node = VirtualCollisionZone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
