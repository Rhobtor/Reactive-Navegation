#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')
        # Subscripción al mapa de ocupación
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/occupancy_grid',
            self.occupancy_callback,
            10
        )
        # Publicador del costmap
        self.publisher = self.create_publisher(OccupancyGrid, '/costmap', 10)
        # Parámetros ajustables
        self.inflation_radius = 0.5  # en metros
        self.cost_obstacle = 100     # valor asignado a obstáculos (puedes ajustar según tu criterio)
        self.cost_free = 0
        self.cost_unknown = -1
        self.latest_grid = None
        # Timer para publicar el costmap a intervalos regulares
        self.timer = self.create_timer(0.5, self.timer_callback)

    def occupancy_callback(self, msg: OccupancyGrid):
        self.latest_grid = msg

    def timer_callback(self):
        if self.latest_grid is None:
            return
        costmap = self.create_costmap(self.latest_grid)
        self.publisher.publish(costmap)
        self.get_logger().info('Costmap publicado.')

    def create_costmap(self, grid_msg: OccupancyGrid) -> OccupancyGrid:
        # Extraer información del mapa y convertir los datos a una matriz 2D
        info = grid_msg.info
        grid_data = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
        
        # Inicializar el costmap copiando el occupancy grid
        costmap_array = np.copy(grid_data)
        
        # Calcular el número de celdas correspondiente al radio de inflado
        inflation_cells = int(math.ceil(self.inflation_radius / info.resolution))
        
        # Encontrar las celdas que son obstáculos
        obstacles = np.argwhere(grid_data == self.cost_obstacle)
        
        # Inflar cada obstáculo: asignar un costo a las celdas vecinas
        for (i, j) in obstacles:
            for di in range(-inflation_cells, inflation_cells + 1):
                for dj in range(-inflation_cells, inflation_cells + 1):
                    ni = i + di
                    nj = j + dj
                    if 0 <= ni < info.height and 0 <= nj < info.width:
                        # Se puede calcular una función de costo en función de la distancia, aquí usamos un simple umbral
                        if math.sqrt(di**2 + dj**2) <= inflation_cells:
                            costmap_array[ni, nj] = self.cost_obstacle
        
        # Crear y configurar el mensaje OccupancyGrid para el costmap
        costmap_msg = OccupancyGrid()
        costmap_msg.header = grid_msg.header
        costmap_msg.info = grid_msg.info
        costmap_msg.data = costmap_array.flatten().tolist()
        return costmap_msg

def main(args=None):
    rclpy.init(args=args)
    node = CostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
