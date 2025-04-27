#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import heapq
import time

from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, Twist, PoseStamped
# Parámetros del mapa y del path
RESOLUTION = 1.0    # metros por celda
MAP_WIDTH = 100     # celdas en x
MAP_HEIGHT = 100    # celdas en y

GOAL_REACHED_DIST = 2.5  # Umbral para considerar alcanzado el goal
CONNECTION_THRESHOLD = 3.0  # Umbral para conectar nodos

def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def dijkstra(grafo, inicio, objetivo):
    dist = {nodo: float('inf') for nodo in grafo.keys()}
    prev = {nodo: None for nodo in grafo.keys()}
    dist[inicio] = 0
    cola = [(0, inicio)]
    while cola:
        costo_actual, actual = heapq.heappop(cola)
        if actual == objetivo:
            break
        if costo_actual > dist[actual]:
            continue
        for vecino, peso in grafo.get(actual, []):
            alt = dist[actual] + peso
            if alt < dist[vecino]:
                dist[vecino] = alt
                prev[vecino] = actual
                heapq.heappush(cola, (alt, vecino))
    camino = []
    nodo = objetivo
    while nodo is not None:
        camino.append(nodo)
        nodo = prev[nodo]
    camino.reverse()
    return camino

def quaternion_to_euler(q: Quaternion):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw



class GlobalPathPublisher(Node):
    def __init__(self):
        super(GlobalPathPublisher, self).__init__('global_path_publisher')

        
        # Suscriptores
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self.current_grid_callback, 10)
        self.create_subscription(OccupancyGrid, '/persistent_dynamic_occupancy_grid', self.memory_grid_callback, 10)
        
        # Publicadores
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.path_marker_pub = self.create_publisher(Marker, '/global_path_marker', 10)
        
        # Variables de estado
        self.odom = None
        self.goal = None
        self.current_grid = None
        self.memory_grid = None
        
        # Timer para actualizar y publicar el path (cada 0.5 s)
        self.create_timer(0.5, self.publish_global_path)
        self.get_logger().info("Nodo GlobalPathPublisher iniciado.")

    # Callback para odometría
    def odom_callback(self, msg: Odometry):
        self.odom = msg.pose.pose

    # Callback para goal (se toma el primer goal recibido)
    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]
            self.get_logger().info("Goal recibido.")

    # Callbacks para mapas
    def current_grid_callback(self, msg: OccupancyGrid):
        self.current_grid = msg

    def memory_grid_callback(self, msg: OccupancyGrid):
        self.memory_grid = msg

    # Función para extraer nodos candidatos de un OccupancyGrid
    def extract_candidate_nodes(self, grid_msg: OccupancyGrid):
        info = grid_msg.info
        grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
        free_indices = np.argwhere(grid == 0)
        nodes = []
        for idx in free_indices:
            j, i = idx
            # Convertir índices a coordenadas en el mundo
            x = info.origin.position.x + (i + 0.5) * info.resolution
            y = info.origin.position.y + (j + 0.5) * info.resolution
            nodes.append((x, y))
        return nodes

    # Construye un grafo combinando ambos mapas
    def build_combined_graph(self):
        nodes = []
        if self.current_grid is not None:
            nodes += self.extract_candidate_nodes(self.current_grid)
        if self.memory_grid is not None:
            nodes += self.extract_candidate_nodes(self.memory_grid)
        return nodes

    # Calcula el path global usando Dijkstra. Si no hay nodos candidatos, se usa una línea directa.
    def plan_global_path(self):
        nodes = self.build_combined_graph()
        if not nodes:
            self.get_logger().warn("No se encontraron nodos candidatos. Usando path directo.")
            if self.odom is None or self.goal is None:
                return None
            robot_pos = (self.odom.position.x, self.odom.position.y)
            goal_pos = (self.goal.position.x, self.goal.position.y)
            nodes = [robot_pos, goal_pos]
            graph = {0: [(1, distance(robot_pos, goal_pos))], 1: [(0, distance(robot_pos, goal_pos))]}
            path_indices = [0, 1]
        else:
            # Seleccionamos el nodo candidato más cercano al goal
            if self.goal is None:
                return None
            goal_coords = (self.goal.position.x, self.goal.position.y)
            best_candidate = min(nodes, key=lambda n: distance(n, goal_coords))
            # Si el candidato está lejos del goal, se crea un punto virtual a mitad de camino
            if distance(best_candidate, goal_coords) > 1.0:
                fraction = 0.5
                virtual_candidate = (best_candidate[0] + fraction*(goal_coords[0] - best_candidate[0]),
                                     best_candidate[1] + fraction*(goal_coords[1] - best_candidate[1]))
            else:
                virtual_candidate = best_candidate

            # Agregamos la posición actual y el candidato (virtual) al grafo
            if self.odom is None:
                return None
            robot_pos = (self.odom.position.x, self.odom.position.y)
            nodes.append(robot_pos)
            nodes.append(virtual_candidate)
            robot_index = len(nodes) - 2
            goal_index = len(nodes) - 1

            # Construimos el grafo conectando nodos que estén a una distancia razonable
            graph = {i: [] for i in range(len(nodes))}
            for i in range(len(nodes)):
                for j in range(i+1, len(nodes)):
                    d = distance(nodes[i], nodes[j])
                    if d <= CONNECTION_THRESHOLD:
                        graph[i].append((j, d))
                        graph[j].append((i, d))
            path_indices = dijkstra(graph, robot_index, goal_index)
            if not path_indices:
                self.get_logger().warn("No se pudo calcular un path global con Dijkstra.")
                return None

        # Reconstruir el path a partir de los índices
        path_points = [nodes[i] for i in path_indices]
        # Aquí se puede aplicar un suavizado si es necesario
        return path_points

    # Publica el path como un mensaje de tipo Path y también como Marker para RViz
    def publish_global_path(self):
        if self.odom is None or self.goal is None:
            return
        path_points = self.plan_global_path()
        if path_points is None or len(path_points) < 2:
            return

        # Publicar Path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for pt in path_points:
            pose_st = PoseStamped()
            pose_st.header = path_msg.header
            pose_st.pose.position.x = pt[0]
            pose_st.pose.position.y = pt[1]
            pose_st.pose.position.z = 0.0
            pose_st.pose.orientation.w = 1.0
            path_msg.poses.append(pose_st)
        self.path_pub.publish(path_msg)

        # Publicar Marker (línea)
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.ns = "global_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # Ancho de la línea
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = []
        for pt in path_points:
            p = Point()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            marker.points.append(p)
        self.path_marker_pub.publish(marker)
        self.get_logger().info(f"Path global publicado con {len(path_points)} puntos.")

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
