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

class OccupancyGridFuser(Node):
    def __init__(self):
        super().__init__('occupancy_grid_fuser')
        # Parámetros del mapa
        self.resolution = 1.0           # metros por celda
        self.width = 100                # número de celdas en x
        self.height = 100               # número de celdas en y
        self.origin_x = -25.0           # origen en x (posición mínima)
        self.origin_y = -25.0           # origen en y

        # Variables para almacenar la información de los topics
        self.free_points = []       # puntos del topic de áreas libres (por ejemplo, /filtered_navigation_nodes)
        self.obstacle_points = []   # puntos del topic de obstáculos (por ejemplo, /obstacle_points)

        # Suscriptores
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.free_points_callback, 10)
        self.create_subscription(PoseArray, '/obstacle_navigation_nodes', self.obstacle_points_callback, 10)

        # Publicador del OccupancyGrid
        self.occ_grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Timer para actualizar y publicar el mapa de ocupación (por ejemplo, a 1 Hz)
        self.create_timer(1.0, self.publish_occupancy_grid)
        self.get_logger().info("Nodo OccupancyGridFuser iniciado.")

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

    def publish_occupancy_grid(self):
        # Inicializa el grid con -1 (desconocido)
        grid = -1 * np.ones((self.height, self.width), dtype=np.int8)

        # Crear máscaras para áreas libres y obstáculos
        free_mask = np.zeros((self.height, self.width), dtype=np.uint8)
        obs_mask = np.zeros((self.height, self.width), dtype=np.uint8)

        # Dibujar puntos libres en la máscara (valor 255 para marcar el área)
        for pose in self.free_points:
            x = pose.position.x
            y = pose.position.y
            i, j = self.world_to_map(x, y)
            if 0 <= i < self.width and 0 <= j < self.height:
                free_mask[j, i] = 255

        # Si hay puntos libres, se pueden rellenar sus contornos para crear un área continua
        if np.count_nonzero(free_mask) > 0:
            # Encuentra contornos en la máscara de libres
            contours, _ = cv2.findContours(free_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Rellena los contornos para formar un polígono continuo
            cv2.fillPoly(free_mask, contours, 255)

        # Dibujar puntos de obstáculos en la máscara
        for pose in self.obstacle_points:
            x = pose.position.x
            y = pose.position.y
            i, j = self.world_to_map(x, y)
            if 0 <= i < self.width and 0 <= j < self.height:
                obs_mask[j, i] = 255

        # Aplicar una dilatación a los obstáculos para ampliar su área
        kernel = np.ones((1,1), np.uint8)
        obs_mask = cv2.dilate(obs_mask, kernel, iterations=1)

        # Fusionar las máscaras:
        # Primero se asigna 0 a las celdas que son libres y luego se sobreescribe con 100 los obstáculos
        grid[free_mask == 255] = 0
        grid[obs_mask == 255] = 100

        # Preparar y publicar el mensaje OccupancyGrid
        occ_grid_msg = OccupancyGrid()
        occ_grid_msg.header = Header()
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
    node = OccupancyGridFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
