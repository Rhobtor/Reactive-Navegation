#################################################
#################################################

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import math
# import numpy as np
# from geometry_msgs.msg import PoseArray, Pose, Point
# from nav_msgs.msg import OccupancyGrid
# from visualization_msgs.msg import Marker
# from std_msgs.msg import Float64, Float64MultiArray, Bool
# from rclpy.qos import QoSProfile, DurabilityPolicy
# import time

# # Se usa DBSCAN de scikit-learn para clustering
# from sklearn.cluster import DBSCAN

# # Parámetros ajustables (las distancias se expresan en metros)
# CLUSTER_DISTANCE_THRESHOLD = 5.0  # Distancia máxima para agrupar puntos frontera
# SAFE_DISTANCE_THRESHOLD = 3.0       # Distancia mínima entre un centroide y un obstáculo para considerarlo seguro
# PERSISTENCE_TOLERANCE = 0.5         # Tolerancia para considerar que un nuevo centroide ya existe en la lista persistente

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
#         # Nueva suscripción para reiniciar la lista de fronteras al alcanzar el goal
#         self.goal_reached_sub = self.create_subscription(
#             Bool,
#             '/goal_reached',
#             self.goal_reached_callback,
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

#         # Lista persistente para almacenar los centroides de fronteras seguros
#         self.persistent_safe_frontiers = []  # Cada elemento es un (x, y)

#         self.get_logger().info("Nodo de cluster de frontera iniciado.")

#     # ----------------------- CALLBACKS -----------------------
#     def occupancy_callback(self, msg: OccupancyGrid):
#         self.occupancy_grid = msg
#         self.compute_and_publish_entropy(msg)

#     def frontier_callback(self, msg: PoseArray):
#         self.frontier_points = msg
#         self.process_frontiers()

#     def obstacle_callback(self, msg: PoseArray):
#         self.obstacle_points = msg
#         self.process_frontiers()

#     def goal_reached_callback(self, msg: Bool):
#         if msg.data:
#             # Reiniciar la lista persistente de safe frontier points
#             self.get_logger().info("Goal alcanzado. Reiniciando la lista de centroides persistentes...")
#             self.persistent_safe_frontiers.clear()

#     # ----------------------- CÁLCULO DE ENTROPÍA -----------------------
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

#         total_entropy_msg = Float64()
#         total_entropy_msg.data = total_entropy
#         self.total_entropy_pub.publish(total_entropy_msg)

#         entropy_array_msg = Float64MultiArray()
#         entropy_array_msg.data = []
#         self.frontier_entropy_pub.publish(entropy_array_msg)
#         self.get_logger().info(f"Entropía total del mapa: {total_entropy:.3f}")

#     # ----------------------- PROCESAMIENTO Y CLUSTERING -----------------------
#     def process_frontiers(self):
#         if self.frontier_points is None:
#             return

#         # Extraer posiciones (x,y) de cada punto frontera
#         frontier_positions = [(pose.position.x, pose.position.y) for pose in self.frontier_points.poses]
#         if not frontier_positions:
#             return

#         frontier_positions = np.array(frontier_positions)

#         # Agrupamiento usando DBSCAN
#         clustering = DBSCAN(eps=CLUSTER_DISTANCE_THRESHOLD, min_samples=2).fit(frontier_positions)
#         labels = clustering.labels_

#         # Calcular el centroide de cada cluster (ignorando ruido, etiqueta -1)
#         cluster_centroids = []
#         unique_labels = set(labels)
#         for label in unique_labels:
#             if label == -1:
#                 continue
#             cluster_points = frontier_positions[labels == label]
#             centroid = np.mean(cluster_points, axis=0)
#             cluster_centroids.append(centroid)

#         # Filtrar centroides que estén demasiado cerca de algún obstáculo
#         safe_centroids = []
#         if self.obstacle_points is not None:
#             for centroid in cluster_centroids:
#                 seguro = True
#                 for obs_pose in self.obstacle_points.poses:
#                     obs = (obs_pose.position.x, obs_pose.position.y)
#                     if np.linalg.norm(np.array(centroid) - np.array(obs)) < SAFE_DISTANCE_THRESHOLD:
#                         seguro = False
#                         break
#                 if seguro:
#                     safe_centroids.append(centroid)
#         else:
#             safe_centroids = cluster_centroids

#         # Actualización de la lista persistente:
#         # Sólo se añaden los centroides nuevos que no existan ya (dentro de PERSISTENCE_TOLERANCE)
#         for centroid in safe_centroids:
#             exists = False
#             for stored in self.persistent_safe_frontiers:
#                 if np.linalg.norm(np.array(centroid) - np.array(stored)) < PERSISTENCE_TOLERANCE:
#                     exists = True
#                     break
#             if not exists:
#                 self.get_logger().info(f"Agregando nuevo centroide persistente: {centroid}")
#                 self.persistent_safe_frontiers.append(centroid)

#         # Publicar la lista persistente como PoseArray
#         persistent_pose_array = PoseArray()
#         persistent_pose_array.header = self.frontier_points.header
#         persistent_pose_array.poses = []
#         for centroid in self.persistent_safe_frontiers:
#             pose = Pose()
#             pose.position.x = float(centroid[0])
#             pose.position.y = float(centroid[1])
#             pose.position.z = 0.0
#             pose.orientation.w = 1.0
#             persistent_pose_array.poses.append(pose)
#         self.safe_frontier_pub.publish(persistent_pose_array)

#         # Publicar Marker para visualización
#         self.publish_safe_marker(persistent_pose_array.poses)
#         self.get_logger().info(f"Se publicaron {len(self.persistent_safe_frontiers)} centroides persistentes.")

#     def publish_safe_marker(self, poses):
#         marker = Marker()
#         if self.frontier_points is not None:
#             marker.header = self.frontier_points.header
#         else:
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
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
########################################################
#########################################################
# #!/usr/bin/env python3
# """
# Nodo ROS 2 para clustering y publicación de fronteras seguras con:
# 1. **Reset perezoso**: al alcanzar la meta se marca un flag y la lista persistente se limpia justo
#    después de recibir las primeras fronteras nuevas.
# 2. **Filtro por rango**: sólo se publican centroides dentro de `MAX_FRONTIER_DISTANCE` respecto a la
#    posición actual del robot obtenida desde el tópico **/odom** (`nav_msgs/Odometry`).
# """

# import math
# from typing import List

# import numpy as np
# import rclpy
# from geometry_msgs.msg import Pose, PoseArray
# from nav_msgs.msg import Odometry, OccupancyGrid
# from rclpy.node import Node
# from rclpy.qos import DurabilityPolicy, QoSProfile
# from sklearn.cluster import DBSCAN
# from std_msgs.msg import Bool, Float64, Float64MultiArray
# from visualization_msgs.msg import Marker

# # -------------------- Parámetros ajustables --------------------
# CLUSTER_DISTANCE_THRESHOLD = 3.0   # [m] eps para DBSCAN
# SAFE_DISTANCE_THRESHOLD = 3.0      # [m] distancia mínima a un obstáculo
# PERSISTENCE_TOLERANCE = 0.5        # [m] para evitar duplicados
# MAX_FRONTIER_DISTANCE = 40.0       # [m] rango máximo del sensor / robot


# # -------------------- Nodo --------------------
# class FrontierClusterNode(Node):
#     def __init__(self):
#         super().__init__("frontier_cluster_node")
#         qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

#         # --- Suscripciones ---
#         self.create_subscription(OccupancyGrid, "/occupancy_grid", self.occupancy_callback, 10)
#         self.create_subscription(PoseArray, "frontier_points", self.frontier_callback, 10)
#         self.create_subscription(PoseArray, "obstacle_navigation_nodes", self.obstacle_callback, 10)
#         self.create_subscription(Bool, "/goal_reached", self.goal_reached_callback, 10)
#         self.create_subscription(Odometry, "/odom", self.odom_callback, 10)  # ← ahora /odom

#         # --- Publicadores ---
#         self.marker_pub = self.create_publisher(Marker, "frontier_marker_centroid", 10)
#         self.safe_frontier_pub = self.create_publisher(PoseArray, "safe_frontier_points_centroid", 10)
#         self.total_entropy_pub = self.create_publisher(Float64, "total_entropy_centroid", 10)
#         self.frontier_entropy_pub = self.create_publisher(Float64MultiArray, "frontier_entropies_centroid", 10)

#         # --- Estado interno ---
#         self.occupancy_grid: OccupancyGrid | None = None
#         self.frontier_points: PoseArray | None = None
#         self.obstacle_points: PoseArray | None = None
#         self.robot_position: np.ndarray | None = None  # [x, y] tomada de /odom

#         self.persistent_safe_frontiers: List[np.ndarray] = []
#         self.reset_needed: bool = False  # flag "lazy reset"

#         self.get_logger().info("Nodo de cluster de frontera iniciado (reset perezoso + filtro de rango con /odom).")

#     # ------------------------------------------------------------
#     # Callbacks básicos
#     # ------------------------------------------------------------
#     def occupancy_callback(self, msg: OccupancyGrid):
#         self.occupancy_grid = msg
#         self.compute_and_publish_entropy(msg)

#     def frontier_callback(self, msg: PoseArray):
#         self.frontier_points = msg

#         # Reset perezoso: borrar justo tras recibir fronteras nuevas
#         if self.reset_needed:
#             self.persistent_safe_frontiers.clear()
#             self.reset_needed = False
#             self.get_logger().info("Lista de centroides reiniciada tras goal alcanzado.")

#         self.process_frontiers()

#     def obstacle_callback(self, msg: PoseArray):
#         self.obstacle_points = msg

#     def goal_reached_callback(self, msg: Bool):
#         if msg.data:
#             self.reset_needed = True  # sólo marcamos
#             self.get_logger().info("Goal alcanzado → marcado de reset (lazy).")

#     def odom_callback(self, msg: Odometry):
#         self.robot_position = np.array([
#             msg.pose.pose.position.x,
#             msg.pose.pose.position.y,
#         ])

#     # ------------------------------------------------------------
#     # Entropía global (opcional)
#     # ------------------------------------------------------------
#     @staticmethod
#     def _cell_entropy(cell_val: int) -> float:
#         if cell_val == -1:
#             p = 0.5
#         else:
#             p = float(cell_val) / 100.0
#         return 0.0 if p <= 0.0 or p >= 1.0 else -(p * math.log(p) + (1 - p) * math.log(1 - p))

#     def compute_and_publish_entropy(self, grid: OccupancyGrid):
#         entropies = [self._cell_entropy(c) for c in grid.data]
#         total_entropy = sum(entropies) / len(entropies) if entropies else 0.0

#         self.total_entropy_pub.publish(Float64(data=total_entropy))
#         self.frontier_entropy_pub.publish(Float64MultiArray(data=[]))

#     # ------------------------------------------------------------
#     # Procesamiento principal
#     # ------------------------------------------------------------
#     def process_frontiers(self):
#         if not self.frontier_points:
#             return

#         positions = np.array([(p.position.x, p.position.y) for p in self.frontier_points.poses])
#         if positions.size == 0:
#             return

#         labels = DBSCAN(eps=CLUSTER_DISTANCE_THRESHOLD, min_samples=1).fit_predict(positions)

#         centroids: List[np.ndarray] = [np.mean(positions[labels == lbl], axis=0)
#                                        for lbl in set(labels) if lbl != -1]

#         safe_centroids: List[np.ndarray] = []
#         for c in centroids:
#             # filtro obstáculo
#             if self.obstacle_points and any(
#                 np.linalg.norm(c - np.array([o.position.x, o.position.y])) < SAFE_DISTANCE_THRESHOLD
#                 for o in self.obstacle_points.poses):
#                 continue
#             # filtro rango
#             if self.robot_position is not None and np.linalg.norm(c - self.robot_position) > MAX_FRONTIER_DISTANCE:
#                 continue
#             safe_centroids.append(c)

#         # fusión con lista persistente
#         for c in safe_centroids:
#             if all(np.linalg.norm(c - p) > PERSISTENCE_TOLERANCE for p in self.persistent_safe_frontiers):
#                 self.persistent_safe_frontiers.append(c)


#         if not self.persistent_safe_frontiers:
#             # busca celda libre aleatoria a < MAX_FRONTIER_DISTANCE
#             free = positions[
#                 np.linalg.norm(positions - self.robot_position, axis=1)
#                 < MAX_FRONTIER_DISTANCE]
#             if free.size:
#                 rnd = free[np.random.randint(len(free))]
#                 self.persistent_safe_frontiers.append(rnd)
#                 self.get_logger().info("Publicando punto libre provisional.")


#         self.publish_persistent_frontiers()

#     # ------------------------------------------------------------
#     # Utils de publicación
#     # ------------------------------------------------------------
#     def publish_persistent_frontiers(self):
#         pose_array = PoseArray()
#         if self.frontier_points:
#             pose_array.header = self.frontier_points.header
#         else:
#             pose_array.header.frame_id = "map"
#             pose_array.header.stamp = self.get_clock().now().to_msg()

#         pose_array.poses = [self._centroid_to_pose(c) for c in self.persistent_safe_frontiers]
#         self.safe_frontier_pub.publish(pose_array)
#         self._publish_marker(pose_array.poses)

#     @staticmethod
#     def _centroid_to_pose(c: np.ndarray) -> Pose:
#         pose = Pose()
#         pose.position.x, pose.position.y, pose.position.z = float(c[0]), float(c[1]), 0.0
#         pose.orientation.w = 1.0
#         return pose

#     def _publish_marker(self, poses: List[Pose]):
#         marker = Marker()
#         if self.frontier_points:
#             marker.header = self.frontier_points.header
#         else:
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "safe_frontier"
#         marker.id = 0
#         marker.type = Marker.SPHERE_LIST
#         marker.action = Marker.ADD
#         marker.scale.x = marker.scale.y = marker.scale.z = 2.3
#         marker.color.a, marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 1.0, 0.0
#         marker.points = [p.position for p in poses]
#         self.marker_pub.publish(marker)


# def main(args=None):
#     rclpy.init(args=args)
#     node = FrontierClusterNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
frontier_cluster_node.py
────────────────────────
• Agrupa puntos de frontera con DBSCAN y filtra por obstáculo y distancia
• Mantiene una lista “persistente” para no publicar duplicados
• Añade **puntos interiores** (celdas libres pegadas a -1) cuando no hay
  fronteras válidas, de modo que el robot siempre tenga algo que explorar
"""

# ───────────────────────── Imports ──────────────────────────────────────
import math
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg      import Odometry, OccupancyGrid
from rclpy.node        import Node
from rclpy.qos         import DurabilityPolicy, QoSProfile
from sklearn.cluster   import DBSCAN
from std_msgs.msg      import Bool, Float64, Float64MultiArray
from visualization_msgs.msg import Marker
from scipy.ndimage     import binary_dilation      # ← NEW

# ───────────────────── Parámetros ajustables ────────────────────────────
CLUSTER_DISTANCE_THRESHOLD = 3.0     # [m]  eps DBSCAN
SAFE_DISTANCE_THRESHOLD    = 3.0     # [m]  obstáculo-centroide mínimo
PERSISTENCE_TOLERANCE      = 0.5     # [m]  evita duplicados
MAX_FRONTIER_DISTANCE      = 40.0    # [m]  radio máximo desde el robot
INTERIOR_SAMPLES           = 100     # nº máx de puntos interiores

# ──────────────────────────── Nodo ───────────────────────────────────────
class FrontierClusterNode(Node):
    # .....................................................................
    def __init__(self):
        super().__init__("frontier_cluster_node")
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        # Subscripciones ---------------------------------------------------
        self.create_subscription(OccupancyGrid, "/occupancy_grid",
                                 self.occupancy_callback, 10)
        self.create_subscription(PoseArray, "frontier_points",
                                 self.frontier_callback, 10)
        self.create_subscription(PoseArray, "obstacle_navigation_nodes",
                                 self.obstacle_callback, 10)
        self.create_subscription(Bool, "/goal_reached",
                                 self.goal_reached_callback, 10)
        self.create_subscription(Odometry, "/odom",
                                 self.odom_callback, 10)

        # Publicadores -----------------------------------------------------
        self.marker_pub          = self.create_publisher(
            Marker, "frontier_marker_centroid", 10)
        self.safe_frontier_pub   = self.create_publisher(
            PoseArray, "safe_frontier_points_centroid", 10)
        self.total_entropy_pub   = self.create_publisher(Float64,  "total_entropy_centroid", 10)
        self.frontier_entropy_pub= self.create_publisher(Float64MultiArray,
                                                         "frontier_entropies_centroid", 10)

        # Estado interno ---------------------------------------------------
        self.occupancy_grid : OccupancyGrid | None = None
        self.frontier_points: PoseArray     | None = None
        self.obstacle_points: PoseArray     | None = None
        self.robot_position : np.ndarray    | None = None

        self.persistent_safe_frontiers: List[np.ndarray] = []
        self.reset_needed = False

        self.get_logger().info("Nodo de cluster de frontera listo.")

    # ─────────────────────── Callbacks básicos ──────────────────────────
    def occupancy_callback(self, msg: OccupancyGrid):
        self.occupancy_grid = msg
        self._compute_and_publish_entropy(msg)

    def frontier_callback(self, msg: PoseArray):
        self.frontier_points = msg

        if self.reset_needed:                       # lazy-reset al llegar a goal
            self.persistent_safe_frontiers.clear()
            self.reset_needed = False
            self.get_logger().info("Reset de centroides tras goal.")

        self._process_frontiers()

    def obstacle_callback(self, msg: PoseArray):
        self.obstacle_points = msg

    def goal_reached_callback(self, msg: Bool):
        if msg.data:
            self.reset_needed = True

    def odom_callback(self, msg: Odometry):
        self.robot_position = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y])

    # ───────────────────── Entropía global (opcional) ───────────────────
    def _cell_entropy(self, v: int) -> float:
        p = 0.5 if v == -1 else float(v) / 100.0
        return 0.0 if p <= 0.0 or p >= 1.0 else -(p*math.log(p)+(1-p)*math.log(1-p))

    def _compute_and_publish_entropy(self, grid: OccupancyGrid):
        entropies = [self._cell_entropy(c) for c in grid.data]
        tot = sum(entropies)/len(entropies) if entropies else 0.0
        self.total_entropy_pub.publish(Float64(data=tot))
        self.frontier_entropy_pub.publish(Float64MultiArray(data=[]))

    # ───────────────────── Helpers de coordenadas ───────────────────────
    def _cell_to_world(self, i: int, j: int) -> np.ndarray:
        info = self.occupancy_grid.info
        return np.array([
            info.origin.position.x + (i + 0.5) * info.resolution,
            info.origin.position.y + (j + 0.5) * info.resolution
        ])

    def _interior_candidates(self) -> list[np.ndarray]:
        """Celdas libres contiguas a -1 (frontier interior)."""
        if self.occupancy_grid is None:
            return []

        info = self.occupancy_grid.info
        grid = np.array(self.occupancy_grid.data, np.int8)\
                 .reshape(info.height, info.width)

        free  = grid == 0
        unkn  = grid == -1
        band  = binary_dilation(unkn, iterations=1) & free

        js, is_ = np.where(band)
        if not len(js):
            return []

        idx = np.random.permutation(len(js))[:INTERIOR_SAMPLES]
        return [self._cell_to_world(i, j) for i, j in zip(is_[idx], js[idx])]

    # ───────────────────── Procesamiento principal ──────────────────────
    def _process_frontiers(self):
        if not self.frontier_points:
            return

        positions = np.array([(p.position.x, p.position.y)
                              for p in self.frontier_points.poses])
        if positions.size == 0:
            return

        labels = DBSCAN(eps=CLUSTER_DISTANCE_THRESHOLD,
                        min_samples=1).fit_predict(positions)

        centroids = [positions[labels == lbl].mean(axis=0)
                     for lbl in set(labels) if lbl != -1]

        # -------- 1 · Filtros básicos sobre centroides ------------------
        safe_centroids: list[np.ndarray] = []
        for c in centroids:
            if self.robot_position is not None and \
               np.linalg.norm(c - self.robot_position) > MAX_FRONTIER_DISTANCE:
                continue
            if self.obstacle_points and any(
                np.linalg.norm(c - np.array([o.position.x, o.position.y]))
                < SAFE_DISTANCE_THRESHOLD for o in self.obstacle_points.poses):
                continue
            safe_centroids.append(c)

        # -------- 2 · Puntos interiores si no hay fronteras válidas -----
        if not safe_centroids:
            for c in self._interior_candidates():
                if self.robot_position is not None and \
                   np.linalg.norm(c - self.robot_position) > MAX_FRONTIER_DISTANCE:
                    continue
                if self.obstacle_points and any(
                    np.linalg.norm(c - np.array([o.position.x, o.position.y]))
                    < SAFE_DISTANCE_THRESHOLD for o in self.obstacle_points.poses):
                    continue
                safe_centroids.append(c)
            if safe_centroids:
                self.get_logger().info("Publicando puntos interiores provisionales.")

        # -------- 3 · Fusionar con lista persistente --------------------
        for c in safe_centroids:
            if all(np.linalg.norm(c - p) > PERSISTENCE_TOLERANCE
                   for p in self.persistent_safe_frontiers):
                self.persistent_safe_frontiers.append(c)

        self._publish_persistent_frontiers()

    # ─────────────────────── Publicación Utils ─────────────────────────
    def _centroid_to_pose(self, c: np.ndarray) -> Pose:
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(c[0]), float(c[1]), 0.0
        p.orientation.w = 1.0
        return p

    def _publish_persistent_frontiers(self):
        pa = PoseArray()
        pa.header.frame_id = (self.frontier_points.header.frame_id
                              if self.frontier_points else "map")
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.poses = [self._centroid_to_pose(c) for c in self.persistent_safe_frontiers]
        self.safe_frontier_pub.publish(pa)

        mk = Marker(header=pa.header, ns="safe_frontier", id=0,
                    type=Marker.SPHERE_LIST, action=Marker.ADD)
        mk.scale.x = mk.scale.y = mk.scale.z = 2.3
        mk.color.a, mk.color.r, mk.color.g, mk.color.b = 1.0, 0.0, 1.0, 0.0
        mk.points = [p.position for p in pa.poses]
        self.marker_pub.publish(mk)

# ──────────────────────────── main ───────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = FrontierClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
