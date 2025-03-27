#!/usr/bin/env python3
import rclpy
import random
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

# Umbral de distancia para evitar que el goal esté cerca de un obstáculo
OBSTACLE_CLEARANCE_THRESHOLD = 2.0  # Puedes ajustar este valor según tus necesidades

def distance_between_poses(pose1: Pose, pose2: Pose) -> float:
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    return math.hypot(dx, dy)

class FrontierSelector(Node):
    def __init__(self):
        super().__init__('frontier_selector')
        # Suscripción a los puntos de frontera
        self.frontier_sub = self.create_subscription(
            PoseArray,
            '/frontier_points',
            self.frontier_callback,
            10
        )
        # Suscripción a los obstáculos
        self.occupied_nodes_sub = self.create_subscription(
            PoseArray,
            '/occupied_rejected_nodes',
            self.occupied_nodes_callback,
            10
        )
        # Publicador del goal (PoseArray)
        self.goal_pub = self.create_publisher(PoseArray, 'goal', 10)
        # Publicador del marcador para RViz
        self.goal_marker_pub = self.create_publisher(Marker, 'goal_marker', 10)
        # Suscripción a la señal de "goal_reached"
        self.goal_reached_sub = self.create_subscription(
            Bool,
            'goal_reached',
            self.goal_reached_callback,
            10
        )
        
        self.frontier_points = []  # Lista de puntos de frontera
        self.occupied_nodes = []   # Lista de obstáculos
        self.current_goal_set = False  # Indica si ya hay un goal activo
        self.last_goal_pose = None     # Último goal publicado

        # Timer para republicar el goal activo cada 0.5 s
        self.goal_timer = self.create_timer(0.5, self.goal_timer_callback)

    def frontier_callback(self, msg: PoseArray):
        """Callback para recibir los puntos de frontera."""
        self.get_logger().info('Recibidos {} puntos de frontera'.format(len(msg.poses)))
        self.frontier_points = msg.poses

        # Si hay puntos y no hay goal activo, intenta publicar uno nuevo.
        if self.frontier_points and not self.current_goal_set:
            self.publish_next_goal()

    def occupied_nodes_callback(self, msg: PoseArray):
        """Callback para recibir los puntos de obstáculos."""
        self.get_logger().info('Recibidos {} puntos de obstáculos'.format(len(msg.poses)))
        self.occupied_nodes = msg.poses

    def goal_reached_callback(self, msg: Bool):
        """Callback que se ejecuta al recibir la señal de que se alcanzó el goal."""
        if msg.data:
            self.get_logger().info('Señal de objetivo alcanzado recibida.')
            self.current_goal_set = False
            self.publish_next_goal()

    def goal_timer_callback(self):
        """Timer para republicar el goal actual cada 0.5 s si aún está activo."""
        if self.current_goal_set and self.last_goal_pose is not None:
            self.get_logger().debug('Republishing goal: ({:.2f}, {:.2f})'.format(
                self.last_goal_pose.position.x, self.last_goal_pose.position.y))
            self.publish_goal(self.last_goal_pose)

    def publish_goal(self, goal_pose: Pose):
        """Publica el goal y su marcador para RViz."""
        # Publicar como PoseArray
        goal_msg = PoseArray()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.poses.append(goal_pose)
        self.goal_pub.publish(goal_msg)

        # Publicar marcador para visualizar en RViz
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = goal_pose
        marker.scale.x = 2.0  # Ajusta el tamaño según necesites
        marker.scale.y = 2.0
        marker.scale.z = 2.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.goal_marker_pub.publish(marker)

    def publish_next_goal(self):
        """
        Selecciona aleatoriamente un goal de los puntos de frontera que esté a una distancia segura
        de los obstáculos y lo publica.
        """
        if not self.frontier_points:
            self.get_logger().warn('No hay puntos de frontera disponibles para publicar.')
            return

        # Filtrar puntos de frontera que no estén demasiado cerca de ningún obstáculo
        valid_frontier_points = []
        for point in self.frontier_points:
            too_close = False
            for obst in self.occupied_nodes:
                if distance_between_poses(point, obst) < OBSTACLE_CLEARANCE_THRESHOLD:
                    too_close = True
                    break
            if not too_close:
                valid_frontier_points.append(point)

        if not valid_frontier_points:
            self.get_logger().warn('No se encontraron puntos de frontera que cumplan con la distancia de seguridad.')
            return

        # Seleccionar aleatoriamente entre los puntos válidos
        random_goal_pose = random.choice(valid_frontier_points)
        self.get_logger().info(f'Publicando un nuevo goal: ({random_goal_pose.position.x:.2f}, {random_goal_pose.position.y:.2f})')
        self.last_goal_pose = random_goal_pose
        self.current_goal_set = True
        self.publish_goal(random_goal_pose)

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
