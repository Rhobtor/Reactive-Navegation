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
        # Valores iniciales para el grid (se pueden ajustar)
        self.origin_x = -25.0         # origen en x
        self.origin_y = -25.0         # origen en y
        self.width = 100              # celdas en x
        self.height = 100             # celdas en y

        # Variables para almacenar la información de los topics
        self.free_points = []       # puntos libres (PoseArray)
        self.obstacle_points = []   # puntos de obstáculos (PoseArray)

        # Suscriptores
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.free_points_callback, 10)
        self.create_subscription(PoseArray, '/obstacle_navigation_nodes', self.obstacle_points_callback, 10)

        # Publicador del OccupancyGrid
        self.occ_grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Timer para actualizar y publicar el mapa de ocupación (por ejemplo, 1 Hz)
        self.create_timer(1.0, self.publish_occupancy_grid)
        self.get_logger().info("Nodo DynamicOccupancyGridFuser iniciado.")

    def free_points_callback(self, msg: PoseArray):
        # Se asume que cada pose representa un punto libre
        self.free_points = msg.poses

    def obstacle_points_callback(self, msg: PoseArray):
        # Cada pose se considera la posición de un obstáculo
        self.obstacle_points = msg.poses

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
        for pose in self.free_points + self.obstacle_points:
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
        # Actualiza el tamaño del mapa según los puntos recibidos
        self.update_map_size()
        # Inicializa el grid con -1 (desconocido)
        grid = -1 * np.ones((self.height, self.width), dtype=np.int8)
        # Crear máscaras para áreas libres y obstáculos
        free_mask = np.zeros((self.height, self.width), dtype=np.uint8)
        obs_mask = np.zeros((self.height, self.width), dtype=np.uint8)
        
        # Dibujar puntos libres
        for pose in self.free_points:
            x = pose.position.x
            y = pose.position.y
            i, j = self.world_to_map(x, y)
            if 0 <= i < self.width and 0 <= j < self.height:
                free_mask[j, i] = 255

        if np.count_nonzero(free_mask) > 0:
            contours, _ = cv2.findContours(free_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.fillPoly(free_mask, contours, 255)
        
        # Dibujar puntos de obstáculos
        for pose in self.obstacle_points:
            x = pose.position.x
            y = pose.position.y
            i, j = self.world_to_map(x, y)
            if 0 <= i < self.width and 0 <= j < self.height:
                obs_mask[j, i] = 255

        # Dilatación para obstáculos
        kernel = np.ones((3, 3), np.uint8)
        obs_mask = cv2.dilate(obs_mask, kernel, iterations=1)

        # Fusionar las máscaras:
        # Se marca 0 para libres y 100 para obstaculos
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
