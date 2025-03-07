#!/usr/bin/env python3

import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker  # Para publicar marcadores en RViz


class FrontierSelector(Node):
    def __init__(self):
        super().__init__('frontier_selector')
        # Suscripción al topic con los puntos de frontera
        self.frontier_sub = self.create_subscription(
            PoseArray,
            '/frontier_points',
            self.frontier_callback,
            10
        )
        # Publicador del punto objetivo en "goal" (tipo PoseArray)
        self.goal_pub = self.create_publisher(PoseArray, 'goal', 10)
        # Publicador del marcador para visualizar el goal en RViz
        self.goal_marker_pub = self.create_publisher(Marker, 'goal_marker', 10)
        # Suscripción al topic "goal_reached" para recibir señal de que se alcanzó el objetivo
        self.goal_reached_sub = self.create_subscription(
            Bool,
            'goal_reached',
            self.goal_reached_callback,
            10
        )
        
        self.frontier_points = []  # Almacena los puntos de frontera
     
        self.current_goal_set = False  # Bandera para saber si se ha publicado un objetivo

    def frontier_callback(self, msg: PoseArray):
        """
        Callback para manejar los puntos de frontera recibidos.
        """
        self.get_logger().info('Recibidos {} puntos de frontera'.format(len(msg.poses)))
        self.frontier_points = msg.poses

        # Si hay puntos y no se ha publicado ningún objetivo, publica el primero
        if self.frontier_points and not self.current_goal_set:
          
            self.publish_next_goal()

    def goal_reached_callback(self, msg: Bool):
        """
        Callback para manejar la señal de objetivo alcanzado.
        """
        if msg.data:
            self.get_logger().info('Señal de objetivo alcanzado recibida. Publicando el siguiente punto.')
            self.publish_next_goal()

    def publish_next_goal(self):
        """
        Publica el siguiente punto de frontera en el topic 'goal' y envía un marcador para RViz.
        """
        if not self.frontier_points:
            self.get_logger().warn('No hay puntos de frontera disponibles para publicar.')
            return

        # Selecciona un punto aleatorio de los puntos de frontera
        random_goal_pose = random.choice(self.frontier_points)

        # Publica el punto como un mensaje PoseArray
        goal_msg = PoseArray()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'  # Ajusta el frame según tu configuración
        goal_msg.poses.append(random_goal_pose)

        self.get_logger().info('Publicando un punto objetivo aleatorio.')
        self.goal_pub.publish(goal_msg)

        # Publicar un marcador para visualizar el goal en RViz
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = random_goal_pose
        marker.scale.x = 0.3  # Ajusta el tamaño según lo que necesites
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.goal_marker_pub.publish(marker)


        # Marca que hay un objetivo activo
        self.current_goal_set = True




def main(args=None):
    rclpy.init(args=args)
    node = FrontierSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
