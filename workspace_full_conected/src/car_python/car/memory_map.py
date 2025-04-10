# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid, MapMetaData
# from std_msgs.msg import Header
# import numpy as np

# class PersistentDynamicOccupancyMemory(Node):
#     def __init__(self):
#         super().__init__('persistent_dynamic_occupancy_memory')
#         # Suscripción al mapa de fusión actual
#         self.subscription = self.create_subscription(
#             OccupancyGrid,
#             '/occupancy_grid',
#             self.occ_grid_callback,
#             10
#         )
#         # Publicador para el mapa de memoria dinámico
#         self.memory_pub = self.create_publisher(OccupancyGrid, '/persistent_dynamic_occupancy_grid', 10)
#         # Timer para publicar el mapa de memoria a 1 Hz
#         self.create_timer(1.0, self.publish_memory_grid)
        
#         # Variables para almacenar el mapa de memoria y su "timestamp"
#         self.memory_grid = None        # Array 2D con los valores de ocupación
#         self.memory_timestamp = None   # Array 2D del mismo tamaño, con el "número de iteración" en que se actualizó la celda
#         self.map_info = None           # Metadata del mapa
#         self.iteration = 0             # Contador de iteraciones (se incrementa con cada nuevo OccupancyGrid recibido)
#         self.retention_steps = 10      # Número de iteraciones que se mantiene la información

#         self.get_logger().info("Nodo PersistentDynamicOccupancyMemory iniciado.")

#     # def occ_grid_callback(self, msg: OccupancyGrid):
#     #     # Convertir el mensaje a un array 2D (se asume orden row-major)
#     #     grid_np = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
#     #     self.iteration += 1  # Actualizamos el contador de iteraciones
#     #     current_iter = self.iteration

#     #     # Inicializar la memoria si es la primera recepción
#     #     if self.memory_grid is None:
#     #         self.memory_grid = grid_np.copy()
#     #         # En las celdas con lectura válida (no desconocida, es decir, distinto de -1) guardamos el iterador
#     #         self.memory_timestamp = np.where(grid_np != -1, current_iter, 0)
#     #         self.map_info = msg.info
#     #         self.get_logger().info("Memoria inicializada en iteración {}".format(current_iter))
#     #     else:
#     #         # Para cada celda, si el nuevo grid tiene información válida (0 o 100), se actualiza el valor y su timestamp
#     #         valid_mask = (grid_np != -1)
#     #         self.memory_grid[valid_mask] = grid_np[valid_mask]
#     #         self.memory_timestamp[valid_mask] = current_iter

#     def occ_grid_callback(self, msg: OccupancyGrid):
#         # Convertir el mensaje a un array 2D (se asume orden row-major)
#         grid_np = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
#         self.iteration += 1  # Actualizamos el contador de iteraciones
#         current_iter = self.iteration

#         # Si la memoria aún no ha sido inicializada o el tamaño del grid cambió, reinicializamos
#         if (self.memory_grid is None or 
#         grid_np.shape != self.memory_grid.shape):
#             self.memory_grid = grid_np.copy()
#             # En las celdas con lectura válida (diferente de -1), guardamos el iterador
#             self.memory_timestamp = np.where(grid_np != -1, current_iter, 0)
#             self.map_info = msg.info
#             self.get_logger().info("Memoria reinicializada en iteración {} con forma {}.".format(current_iter, grid_np.shape))
#         else:
#             # Para cada celda, si el nuevo grid tiene información válida (0 o 100), se actualiza el valor y su timestamp
#             valid_mask = (grid_np != -1)
#             self.memory_grid[valid_mask] = grid_np[valid_mask]
#             self.memory_timestamp[valid_mask] = current_iter


#     def publish_memory_grid(self):
#         if self.memory_grid is None or self.map_info is None:
#             return  # Aún no se ha recibido ningún mapa
#         current_iter = self.iteration
#         # Creamos una copia del mapa de memoria para aplicar el "decaimiento"
#         published_grid = self.memory_grid.copy()
#         # Si una celda no ha sido actualizada en más de 'retention_steps' iteraciones, se marca como desconocida (-1)
#         decay_mask = (current_iter - self.memory_timestamp) > self.retention_steps
#         published_grid[decay_mask] = -1

#         # Preparar el mensaje OccupancyGrid con el mapa persistente actualizado
#         occ_msg = OccupancyGrid()
#         occ_msg.header = Header()
#         occ_msg.header.stamp = self.get_clock().now().to_msg()
#         occ_msg.header.frame_id = "map"
#         occ_msg.info = self.map_info
#         occ_msg.data = published_grid.flatten().tolist()
#         self.memory_pub.publish(occ_msg)
#         self.get_logger().info("Publicando memoria dinámica, iteración {} - celdas decaídas: {}".format(
#             current_iter, np.count_nonzero(decay_mask)
#         ))

# def main(args=None):
#     rclpy.init(args=args)
#     node = PersistentDynamicOccupancyMemory()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from std_srvs.srv import Empty
import numpy as np

class PersistentDynamicOccupancyMemory(Node):
    def __init__(self):
        super().__init__('persistent_dynamic_occupancy_memory')
        # Suscripción al mapa actual
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/occupancy_grid',
            self.occ_grid_callback,
            10
        )
        # Publicador para el mapa de memoria dinámico
        self.memory_pub = self.create_publisher(OccupancyGrid, '/persistent_dynamic_occupancy_grid', 10)
        
        # Variables para almacenar el mapa y su timestamp
        self.memory_grid = None
        self.memory_timestamp = None
        self.map_info = None
        self.iteration = 0
        self.retention_steps = 60

        # Crear cliente para resetear el octomap
        self.reset_client = self.create_client(Empty, 'octomap_server/reset')
        # Esperar un momento a que el servicio esté disponible
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("Servicio octomap_server/reset no disponible al iniciar.")

        # Timer para publicar el mapa de memoria a 1 Hz
        self.create_timer(1.0, self.publish_memory_grid)
        self.get_logger().info("Nodo PersistentDynamicOccupancyMemory iniciado.")

    def occ_grid_callback(self, msg: OccupancyGrid):
        grid_np = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.iteration += 1
        current_iter = self.iteration

        # Cada 10 pasos se llama al servicio para reiniciar el octomap
        if current_iter % 10 == 0:
            if self.reset_client.wait_for_service(timeout_sec=1.0):
                req = Empty.Request()
                self.reset_client.call_async(req)
                self.get_logger().info("Octomap reseteado en iteración {}.".format(current_iter))
            else:
                self.get_logger().warning("Servicio octomap_server/reset no disponible en iteración {}.".format(current_iter))

        # Inicializar o actualizar la memoria
        if self.memory_grid is None or grid_np.shape != self.memory_grid.shape:
            self.memory_grid = grid_np.copy()
            self.memory_timestamp = np.where(grid_np != -1, current_iter, 0)
            self.map_info = msg.info
            self.get_logger().info("Memoria inicializada/reinicializada en iteración {}.".format(current_iter))
        else:
            valid_mask = (grid_np != -1)
            self.memory_grid[valid_mask] = grid_np[valid_mask]
            self.memory_timestamp[valid_mask] = current_iter

    def publish_memory_grid(self):
        if self.memory_grid is None or self.map_info is None:
            return
        current_iter = self.iteration
        published_grid = self.memory_grid.copy()
        decay_mask = (current_iter - self.memory_timestamp) > self.retention_steps
        published_grid[decay_mask] = -1

        occ_msg = OccupancyGrid()
        occ_msg.header = Header()
        occ_msg.header.stamp = self.get_clock().now().to_msg()
        occ_msg.header.frame_id = "map"
        occ_msg.info = self.map_info
        occ_msg.data = published_grid.flatten().tolist()
        self.memory_pub.publish(occ_msg)
        self.get_logger().info("Publicando memoria dinámica, iteración {} - celdas decaídas: {}".format(
            current_iter, int(np.count_nonzero(decay_mask))
        ))

def main(args=None):
    rclpy.init(args=args)
    node = PersistentDynamicOccupancyMemory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
