# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import math
# import numpy as np
# from geometry_msgs.msg import PoseArray, Pose, Point
# from nav_msgs.msg import OccupancyGrid
# from visualization_msgs.msg import Marker
# from std_msgs.msg import Float64, Float64MultiArray
# from rclpy.qos import QoSProfile, DurabilityPolicy
# import time

# # Parámetros ajustables
# CLUSTER_DISTANCE_THRESHOLD = 5.0  # Distancia máxima para que dos puntos frontera pertenezcan al mismo cluster
# SAFE_DISTANCE_THRESHOLD = 3       # Distancia mínima entre un punto (o centroide) y un obstáculo para considerarlo seguro

# class FrontierClusterNode(Node):
#     def __init__(self):
#         super().__init__('frontier_cluster_node')
#         qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

#         # Suscripciones
#         self.occupancy_sub = self.create_subscription(
#             OccupancyGrid,
#             '/occupancy_grid',
#             self.occupancy_callback,
#             10
#         )
#         self.frontier_sub = self.create_subscription(
#             PoseArray,
#             'frontier_points',  # Tópico con los puntos frontera generados en C++
#             self.frontier_callback,
#             10
#         )
#         self.obstacle_sub = self.create_subscription(
#             PoseArray,
#             'obstacle_navigation_nodes',  # Tópico con la posición de obstáculos
#             self.obstacle_callback,
#             10
#         )

#         # Publicadores
#         self.marker_pub = self.create_publisher(Marker, 'frontier_marker_centroid', 10)
#         self.frontier_points_pub = self.create_publisher(PoseArray, 'frontier_points_centroid', 10)
#         self.frontier_entropy_pub = self.create_publisher(Float64MultiArray, 'frontier_entropies_centroid', 10)
#         self.total_entropy_pub = self.create_publisher(Float64, 'total_entropy_centroid', 10)
#         self.safe_frontier_pub = self.create_publisher(PoseArray, 'safe_frontier_points_centroid', 10)

#         # Variables para almacenar los datos recibidos
#         self.occupancy_grid = None
#         self.frontier_points = None  # PoseArray
#         self.obstacle_points = None  # PoseArray

#         self.get_logger().info("Nodo de cluster de frontera iniciado.")

#     # ----------------------- Callbacks -----------------------
#     def occupancy_callback(self, msg: OccupancyGrid):
#         self.occupancy_grid = msg
#         # Una vez recibido el mapa, podemos calcular la entropía total y local
#         self.compute_and_publish_entropy(msg)

#     def frontier_callback(self, msg: PoseArray):
#         self.frontier_points = msg
#         self.process_frontiers()

#     def obstacle_callback(self, msg: PoseArray):
#         self.obstacle_points = msg
#         self.process_frontiers()

#     # ----------------------- Cálculo de Entropía -----------------------
#     def computeCellEntropy(self, cell_value: int) -> float:
#         if cell_value == -1:
#             p = 0.5
#         else:
#             p = float(cell_value) / 100.0
#         if p <= 0.0 or p >= 1.0:
#             return 0.0
#         return -(p * math.log(p) + (1 - p) * math.log(1 - p))

#     def compute_and_publish_entropy(self, grid: OccupancyGrid):
#         total_entropy_sum = 0.0
#         for cell in grid.data:
#             total_entropy_sum += self.computeCellEntropy(cell)
#         total_entropy = total_entropy_sum / len(grid.data) if grid.data else 0.0

#         # Publicar la entropía total
#         total_entropy_msg = Float64()
#         total_entropy_msg.data = total_entropy
#         self.total_entropy_pub.publish(total_entropy_msg)

#         # Calcular entropía local para cada celda frontera (si se desea)
#         # Aquí se puede hacer similar a lo que se hacía en C++ para cada celda.
#         # Por simplicidad, se omite el cálculo detallado de entropía local aquí.
#         # Se publicará un arreglo vacío.
#         entropy_array_msg = Float64MultiArray()
#         entropy_array_msg.data = []
#         self.frontier_entropy_pub.publish(entropy_array_msg)
#         self.get_logger().info(f"Entropía total del mapa: {total_entropy:.3f}")


#     # ----------------------- Procesamiento y Clustering de Puntos Frontera -----------------------
#     def process_frontiers(self):
#         # Procesar solo si tenemos datos de frontera y obstáculos (aunque obstáculos pueden no estar disponibles)
#         if self.frontier_points is None:
#             return

#         # Extraer las posiciones de los puntos frontera
#         frontier_poses = self.frontier_points.poses
#         frontier_positions = [(pose.position.x, pose.position.y) for pose in frontier_poses]

#         if not frontier_positions:
#             return

#         # Clustering simple: asignar cada punto a un cluster si está cerca de un cluster existente.
#         clusters = []  # Cada cluster será una lista de (x, y)
#         for pos in frontier_positions:
#             assigned = False
#             for cluster in clusters:
#                 # Usamos la distancia entre el punto y el centroide actual del cluster
#                 centroid = np.mean(cluster, axis=0)
#                 if np.linalg.norm(np.array(pos) - centroid) < CLUSTER_DISTANCE_THRESHOLD:
#                     cluster.append(pos)
#                     assigned = True
#                     break
#             if not assigned:
#                 clusters.append([pos])
        
#         # Calcular el centroide de cada cluster
#         cluster_centroids = []
#         for cluster in clusters:
#             cluster = np.array(cluster)
#             centroid = np.mean(cluster, axis=0)
#             cluster_centroids.append(centroid)

#         # Filtrar los centroides que estén muy cerca de algún obstáculo
#         safe_centroids = []
#         if self.obstacle_points is not None:
#             for centroid in cluster_centroids:
#                 safe = True
#                 for obs_pose in self.obstacle_points.poses:
#                     obs = (obs_pose.position.x, obs_pose.position.y)
#                     if np.linalg.norm(np.array(centroid) - np.array(obs)) < SAFE_DISTANCE_THRESHOLD:
#                         safe = False
#                         break
#                 if safe:
#                     safe_centroids.append(centroid)
#         else:
#             safe_centroids = cluster_centroids

#         # Publicar los puntos frontera agrupados (safe centroids)
#         safe_pose_array = PoseArray()
#         safe_pose_array.header = self.frontier_points.header
#         safe_pose_array.poses = []
#         for centroid in safe_centroids:
#             pose = Pose()
#             pose.position.x = centroid[0]
#             pose.position.y = centroid[1]
#             pose.position.z = 0.0
#             # Asumimos una orientación por defecto (sin rotación)
#             pose.orientation.w = 1.0
#             safe_pose_array.poses.append(pose)
#         self.safe_frontier_pub.publish(safe_pose_array)
#         self.publish_safe_marker(safe_pose_array.poses)
#         self.get_logger().info(f"Se publicaron {len(safe_pose_array.poses)} centroides seguros de {len(frontier_positions)} puntos frontera.")

#     def publish_safe_marker(self, poses):
#         marker = Marker()
#         marker.header = self.frontier_points.header
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "safe_frontier"
#         marker.id = 0
#         marker.type = Marker.SPHERE_LIST
#         marker.action = Marker.ADD
#         marker.scale.x = 3.15
#         marker.scale.y = 3.15
#         marker.scale.z = 3.15
#         marker.color.a = 1.0
#         marker.color.r = 0.0
#         marker.color.g = 1.0
#         marker.color.b = 0.0
#         marker.points = []
#         for pose in poses:
#             marker.points.append(pose.position)
#         self.marker_pub.publish(marker)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FrontierClusterNode()
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
import math
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64, Float64MultiArray, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
import time

# Se usa DBSCAN de scikit-learn para clustering
from sklearn.cluster import DBSCAN

# Parámetros ajustables (las distancias se expresan en metros)
CLUSTER_DISTANCE_THRESHOLD = 5.0  # Distancia máxima para agrupar puntos frontera
SAFE_DISTANCE_THRESHOLD = 3.0       # Distancia mínima entre un centroide y un obstáculo para considerarlo seguro
PERSISTENCE_TOLERANCE = 0.5         # Tolerancia para considerar que un nuevo centroide ya existe en la lista persistente

class FrontierClusterNode(Node):
    def __init__(self):
        super().__init__('frontier_cluster_node')
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        # Suscripciones
        self.occupancy_sub = self.create_subscription(
            OccupancyGrid,
            '/occupancy_grid',
            self.occupancy_callback,
            10
        )
        self.frontier_sub = self.create_subscription(
            PoseArray,
            'frontier_points',  # Tópico con los puntos frontera generados en C++
            self.frontier_callback,
            10
        )
        self.obstacle_sub = self.create_subscription(
            PoseArray,
            'obstacle_navigation_nodes',  # Tópico con la posición de obstáculos
            self.obstacle_callback,
            10
        )
        # Nueva suscripción para reiniciar la lista de fronteras al alcanzar el goal
        self.goal_reached_sub = self.create_subscription(
            Bool,
            '/goal_reached',
            self.goal_reached_callback,
            10
        )

        # Publicadores
        self.marker_pub = self.create_publisher(Marker, 'frontier_marker_centroid', 10)
        self.frontier_points_pub = self.create_publisher(PoseArray, 'frontier_points_centroid', 10)
        self.frontier_entropy_pub = self.create_publisher(Float64MultiArray, 'frontier_entropies_centroid', 10)
        self.total_entropy_pub = self.create_publisher(Float64, 'total_entropy_centroid', 10)
        self.safe_frontier_pub = self.create_publisher(PoseArray, 'safe_frontier_points_centroid', 10)

        # Variables para almacenar los datos recibidos
        self.occupancy_grid = None
        self.frontier_points = None  # PoseArray
        self.obstacle_points = None  # PoseArray

        # Lista persistente para almacenar los centroides de fronteras seguros
        self.persistent_safe_frontiers = []  # Cada elemento es un (x, y)

        self.get_logger().info("Nodo de cluster de frontera iniciado.")

    # ----------------------- CALLBACKS -----------------------
    def occupancy_callback(self, msg: OccupancyGrid):
        self.occupancy_grid = msg
        self.compute_and_publish_entropy(msg)

    def frontier_callback(self, msg: PoseArray):
        self.frontier_points = msg
        self.process_frontiers()

    def obstacle_callback(self, msg: PoseArray):
        self.obstacle_points = msg
        self.process_frontiers()

    def goal_reached_callback(self, msg: Bool):
        if msg.data:
            # Reiniciar la lista persistente de safe frontier points
            self.get_logger().info("Goal alcanzado. Reiniciando la lista de centroides persistentes...")
            self.persistent_safe_frontiers.clear()

    # ----------------------- CÁLCULO DE ENTROPÍA -----------------------
    def computeCellEntropy(self, cell_value: int) -> float:
        if cell_value == -1:
            p = 0.5
        else:
            p = float(cell_value) / 100.0
        if p <= 0.0 or p >= 1.0:
            return 0.0
        return -(p * math.log(p) + (1 - p) * math.log(1 - p))

    def compute_and_publish_entropy(self, grid: OccupancyGrid):
        total_entropy_sum = 0.0
        for cell in grid.data:
            total_entropy_sum += self.computeCellEntropy(cell)
        total_entropy = total_entropy_sum / len(grid.data) if grid.data else 0.0

        total_entropy_msg = Float64()
        total_entropy_msg.data = total_entropy
        self.total_entropy_pub.publish(total_entropy_msg)

        entropy_array_msg = Float64MultiArray()
        entropy_array_msg.data = []
        self.frontier_entropy_pub.publish(entropy_array_msg)
        self.get_logger().info(f"Entropía total del mapa: {total_entropy:.3f}")

    # ----------------------- PROCESAMIENTO Y CLUSTERING -----------------------
    def process_frontiers(self):
        if self.frontier_points is None:
            return

        # Extraer posiciones (x,y) de cada punto frontera
        frontier_positions = [(pose.position.x, pose.position.y) for pose in self.frontier_points.poses]
        if not frontier_positions:
            return

        frontier_positions = np.array(frontier_positions)

        # Agrupamiento usando DBSCAN
        clustering = DBSCAN(eps=CLUSTER_DISTANCE_THRESHOLD, min_samples=2).fit(frontier_positions)
        labels = clustering.labels_

        # Calcular el centroide de cada cluster (ignorando ruido, etiqueta -1)
        cluster_centroids = []
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:
                continue
            cluster_points = frontier_positions[labels == label]
            centroid = np.mean(cluster_points, axis=0)
            cluster_centroids.append(centroid)

        # Filtrar centroides que estén demasiado cerca de algún obstáculo
        safe_centroids = []
        if self.obstacle_points is not None:
            for centroid in cluster_centroids:
                seguro = True
                for obs_pose in self.obstacle_points.poses:
                    obs = (obs_pose.position.x, obs_pose.position.y)
                    if np.linalg.norm(np.array(centroid) - np.array(obs)) < SAFE_DISTANCE_THRESHOLD:
                        seguro = False
                        break
                if seguro:
                    safe_centroids.append(centroid)
        else:
            safe_centroids = cluster_centroids

        # Actualización de la lista persistente:
        # Sólo se añaden los centroides nuevos que no existan ya (dentro de PERSISTENCE_TOLERANCE)
        for centroid in safe_centroids:
            exists = False
            for stored in self.persistent_safe_frontiers:
                if np.linalg.norm(np.array(centroid) - np.array(stored)) < PERSISTENCE_TOLERANCE:
                    exists = True
                    break
            if not exists:
                self.get_logger().info(f"Agregando nuevo centroide persistente: {centroid}")
                self.persistent_safe_frontiers.append(centroid)

        # Publicar la lista persistente como PoseArray
        persistent_pose_array = PoseArray()
        persistent_pose_array.header = self.frontier_points.header
        persistent_pose_array.poses = []
        for centroid in self.persistent_safe_frontiers:
            pose = Pose()
            pose.position.x = float(centroid[0])
            pose.position.y = float(centroid[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            persistent_pose_array.poses.append(pose)
        self.safe_frontier_pub.publish(persistent_pose_array)

        # Publicar Marker para visualización
        self.publish_safe_marker(persistent_pose_array.poses)
        self.get_logger().info(f"Se publicaron {len(self.persistent_safe_frontiers)} centroides persistentes.")

    def publish_safe_marker(self, poses):
        marker = Marker()
        if self.frontier_points is not None:
            marker.header = self.frontier_points.header
        else:
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "safe_frontier"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 3.15
        marker.scale.y = 3.15
        marker.scale.z = 3.15
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = []
        for pose in poses:
            marker.points.append(pose.position)
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
