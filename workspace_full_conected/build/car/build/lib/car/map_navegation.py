#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
import cv2
import math
import time

class DynamicOccupancyGridFuser(Node):
    def __init__(self):
        super().__init__('dynamic_occupancy_grid_fuser')
        # Parámetros base para el mapa (valores iniciales)
        self.resolution = 1.0         # metros por celda
        self.margin = 10              # margen extra en celdas para expansión
        # Valores iniciales para el grid
        self.origin_x = -25.0         # origen en x
        self.origin_y = -25.0         # origen en y
        self.width = 100              # celdas en x
        self.height = 100             # celdas en y

        # Nuevo parámetro: rango máximo para considerar obstáculos "actuales"
        self.sensor_range = 10.0      # en metros

        # Variables para almacenar la información de los topics
        # (en este caso, no se usa timestamp ya que filtraremos por distancia)
        self.free_points = None       # PoseArray
        self.obstacle_points = None   # PoseArray

        # Suscriptores
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.free_points_callback, 10)
        self.create_subscription(PoseArray, '/obstacle_navigation_nodes', self.obstacle_points_callback, 10)

        # Publicador del OccupancyGrid
        self.occ_grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Timer para actualizar y publicar el mapa de ocupación (por ejemplo, 1 Hz)
        self.create_timer(1.0, self.publish_occupancy_grid)
        self.get_logger().info("Nodo DynamicOccupancyGridFuser iniciado.")

        # Para conocer la posición actual del robot (necesario para filtrar obstáculos)
        self.robot_pose = None
        self.create_subscription(PoseArray, '/robot_pose_topic', self.robot_pose_callback, 10)  # Cambia el topic si es necesario

    def free_points_callback(self, msg: PoseArray):
        self.free_points = msg

    def obstacle_points_callback(self, msg: PoseArray):
        self.obstacle_points = msg

    def robot_pose_callback(self, msg: PoseArray):
        # Asumimos que viene un PoseArray con al menos una pose, la primera es la posición actual
        if msg.poses:
            self.robot_pose = msg.poses[0]

    def world_to_map(self, x, y):
        # Convierte coordenadas del mundo a índices en la matriz del mapa
        i = int((x - self.origin_x) / self.resolution)
        j = int((y - self.origin_y) / self.resolution)
        return i, j

    def update_map_size(self):
        """
        Recalcula los límites del mapa basados en los puntos libres y de obstáculos,
        y ajusta el origen, el ancho y la altura para incluir todos los puntos.
        """
        all_points = []
        for msg in [self.free_points, self.obstacle_points]:
            if msg is None:
                continue
            for pose in msg.poses:
                all_points.append((pose.position.x, pose.position.y))
        if not all_points:
            return

        xs, ys = zip(*all_points)
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        # Añadir margen (en metros)
        margin_m = self.margin * self.resolution

        new_origin_x = min(self.origin_x, min_x - margin_m)
        new_origin_y = min(self.origin_y, min_y - margin_m)
        new_width = int(math.ceil((max_x - new_origin_x) / self.resolution)) + self.margin
        new_height = int(math.ceil((max_y - new_origin_y) / self.resolution)) + self.margin

        if new_width != self.width or new_height != self.height or new_origin_x != self.origin_x or new_origin_y != self.origin_y:
            self.get_logger().info(
                f"Expandiendo mapa: nuevo origen=({new_origin_x:.2f}, {new_origin_y:.2f}), tamaño=({new_width}x{new_height})"
            )
        self.origin_x = new_origin_x
        self.origin_y = new_origin_y
        self.width = new_width
        self.height = new_height

    def publish_occupancy_grid(self):
        # Actualiza el tamaño del mapa basado en los puntos actuales
        self.update_map_size()

        # Inicializa el grid con -1 (desconocido)
        grid = -1 * np.ones((self.height, self.width), dtype=np.int8)
        free_mask = np.zeros((self.height, self.width), dtype=np.uint8)
        obs_mask = np.zeros((self.height, self.width), dtype=np.uint8)

        # Dibujar puntos libres
        if self.free_points is not None:
            for pose in self.free_points.poses:
                x = pose.position.x
                y = pose.position.y
                i, j = self.world_to_map(x, y)
                if 0 <= i < self.width and 0 <= j < self.height:
                    free_mask[j, i] = 255

        # Rellenar áreas libres
        if np.count_nonzero(free_mask) > 0:
            contours, _ = cv2.findContours(free_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.fillPoly(free_mask, contours, 255)

        # Dibujar puntos de obstáculos, filtrando por distancia al robot
        if self.obstacle_points is not None:
            for pose in self.obstacle_points.poses:
                x = pose.position.x
                y = pose.position.y
                # Si se conoce la posición del robot, se filtran los obstáculos lejanos
                if self.robot_pose is not None:
                    rx = self.robot_pose.position.x
                    ry = self.robot_pose.position.y
                    if distance((x, y), (rx, ry)) > self.sensor_range:
                        continue  # Se omite el obstáculo si está fuera del rango actual
                i, j = self.world_to_map(x, y)
                if 0 <= i < self.width and 0 <= j < self.height:
                    obs_mask[j, i] = 255

        # Dilatación para obstáculo (ajustable según necesidad)
        kernel = np.ones((1, 1), np.uint8)
        obs_mask = cv2.dilate(obs_mask, kernel, iterations=1)

        # Fusionar las máscaras:
        grid[free_mask == 255] = 0
        grid[obs_mask == 255] = 100

        # Preparar y publicar el mensaje OccupancyGrid
        occ_grid_msg = OccupancyGrid()
        occ_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occ_grid_msg.header.frame_id = "map"
        occ_grid_msg.info = MapMetaData()
        occ_grid_msg.info.resolution = self.resolution
        occ_grid_msg.info.width = self.width
        occ_grid_msg.info.height = self.height
        occ_grid_msg.info.origin.position.x = self.origin_x
        occ_grid_msg.info.origin.position.y = self.origin_y
        occ_grid_msg.info.origin.position.z = 0.0
        occ_grid_msg.info.origin.orientation.w = 1.0
        occ_grid_msg.data = grid.flatten().tolist()

        self.occ_grid_pub.publish(occ_grid_msg)
        self.get_logger().info(f"Mapa de ocupación publicado a las {time.strftime('%H:%M:%S')}")

def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def main(args=None):
    rclpy.init(args=args)
    node = DynamicOccupancyGridFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# #!/usr/bin/env python3
# import math
# import time

# import cv2
# import numpy as np
# import rclpy
# from nav_msgs.msg import OccupancyGrid, MapMetaData
# from geometry_msgs.msg import PoseArray, PoseStamped
# from nav_msgs.msg import Odometry
# from rclpy.node import Node

# def distance(p1, p2):
#     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# class DynamicOccupancyGridFuser(Node):
#     def __init__(self):
#         super().__init__('dynamic_occupancy_grid_fuser')

#         # Parámetros del grid local estático
#         self.declare_parameter('resolution', 0.5)   # metros por celda
#         self.declare_parameter('width', 200)        # celdas en x
#         self.declare_parameter('height', 200)       # celdas en y
#         self.declare_parameter('sensor_range', 10.0) # rango de sensores en m
#         self.declare_parameter('morph_kernel', 2)    # tamaño de kernel de cierre morfológico
#         self.declare_parameter('morph_iterations', 2)

#         self.resolution = self.get_parameter('resolution').value
#         self.width      = self.get_parameter('width').value
#         self.height     = self.get_parameter('height').value
#         self.sensor_range      = self.get_parameter('sensor_range').value
#         self.morph_kernel      = self.get_parameter('morph_kernel').value
#         self.morph_iterations  = self.get_parameter('morph_iterations').value

#         # Estado
#         self.free_points     = None  # PoseArray
#         self.obstacle_points = None  # PoseArray
#         self.robot_pose      = None  # Odometry

#         # Suscripciones
#         self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.free_cb, 10)
#         self.create_subscription(PoseArray, '/obstacle_navigation_nodes', self.obs_cb, 10)
#         self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

#         # Publicador de OccupancyGrid
#         self.occ_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

#         # Timer de publicación
#         self.create_timer(1.0, self.publish_occupancy_grid)
#         self.get_logger().info('DynamicOccupancyGridFuser mejorado iniciado.')

#     def free_cb(self, msg: PoseArray):
#         self.free_points = msg

#     def obs_cb(self, msg: PoseArray):
#         self.obstacle_points = msg

#     def odom_cb(self, msg: Odometry):
#         self.robot_pose = msg

#     def world_to_map(self, x, y):
#         # Convierte mundo->índices de la grid actual
#         i = int((x - self.origin_x) / self.resolution)
#         j = int((y - self.origin_y) / self.resolution)
#         return i, j

#     def publish_occupancy_grid(self):
#         # Necesitamos posición del robot
#         if self.robot_pose is None:
#             return
#         rx = self.robot_pose.pose.pose.position.x
#         ry = self.robot_pose.pose.pose.position.y

#         # Centrar ventana local en robot
#         self.origin_x = rx - (self.width * self.resolution) / 2.0
#         self.origin_y = ry - (self.height * self.resolution) / 2.0

#         # Iniciar grid con desconocido
#         grid = -1 * np.ones((self.height, self.width), dtype=np.int8)
#         free_mask = np.zeros((self.height, self.width), dtype=np.uint8)
#         obs_mask  = np.zeros((self.height, self.width), dtype=np.uint8)

#         # Marcar puntos libres
#         if self.free_points:
#             for pose in self.free_points.poses:
#                 x, y = pose.position.x, pose.position.y
#                 i, j = self.world_to_map(x, y)
#                 if 0 <= i < self.width and 0 <= j < self.height:
#                     free_mask[j, i] = 255

#         # Cierre morfológico para rellenar huecos libres suavemente
#         kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
#         free_closed = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel,
#                                        iterations=self.morph_iterations)

#         # Marcar puntos de obstáculos dentro de rango
#         if self.obstacle_points:
#             for pose in self.obstacle_points.poses:
#                 x, y = pose.position.x, pose.position.y
#                 # Filtrar por distancia al robot
#                 if distance((x, y), (rx, ry)) > self.sensor_range:
#                     continue
#                 i, j = self.world_to_map(x, y)
#                 if 0 <= i < self.width and 0 <= j < self.height:
#                     obs_mask[j, i] = 255
#         # Dilatación sencilla para robustez
#         obs_mask = cv2.dilate(obs_mask, kernel, iterations=1)

#         # Combinar máscaras en grid
#         grid[free_closed == 255] = 0    # libre
#         grid[obs_mask   == 255] = 100  # obstáculo

#         # Publicar
#         msg = OccupancyGrid()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'map'
#         msg.info = MapMetaData()
#         msg.info.resolution = self.resolution
#         msg.info.width      = self.width
#         msg.info.height     = self.height
#         msg.info.origin.position.x = self.origin_x
#         msg.info.origin.position.y = self.origin_y
#         msg.info.origin.position.z = 0.0
#         msg.info.origin.orientation.w = 1.0
#         msg.data = grid.flatten().tolist()

#         self.occ_pub.publish(msg)
#         self.get_logger().info(f'OccupancyGrid publicado a {time.strftime("%H:%M:%S")}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = DynamicOccupancyGridFuser()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
