# #!/usr/bin/env python3
# import rclpy
# import random
# import math
# from rclpy.node import Node
# from geometry_msgs.msg import PoseArray, Pose
# from std_msgs.msg import Bool
# from visualization_msgs.msg import Marker

# # Umbral de distancia para evitar que el goal esté cerca de un obstáculo
# OBSTACLE_CLEARANCE_THRESHOLD = 6.0  # Puedes ajustar este valor según tus necesidades

# def distance_between_poses(pose1: Pose, pose2: Pose) -> float:
#     dx = pose1.position.x - pose2.position.x
#     dy = pose1.position.y - pose2.position.y
#     return math.hypot(dx, dy)

# class FrontierSelector(Node):
#     def __init__(self):
#         super().__init__('frontier_selector')
#         # Suscripción a los puntos de frontera
#         self.frontier_sub = self.create_subscription(
#             PoseArray,
#             '/safe_frontier_points',
#             self.frontier_callback,
#             10
#         )
#         # Suscripción a los obstáculos
#         self.occupied_nodes_sub = self.create_subscription(
#             PoseArray,
#             '/occupied_rejected_nodes',
#             self.occupied_nodes_callback,
#             10
#         )
#         # Publicador del goal (PoseArray)
#         self.goal_pub = self.create_publisher(PoseArray, 'goal', 10)
#         # Publicador del marcador para RViz
#         self.goal_marker_pub = self.create_publisher(Marker, 'goal_marker', 10)
#         # Suscripción a la señal de "goal_reached"
#         self.goal_reached_sub = self.create_subscription(
#             Bool,
#             'goal_reached',
#             self.goal_reached_callback,
#             10
#         )
        
#         self.frontier_points = []  # Lista de puntos de frontera
#         self.occupied_nodes = []   # Lista de obstáculos
#         self.current_goal_set = False  # Indica si ya hay un goal activo
#         self.last_goal_pose = None     # Último goal publicado

#         # Timer para republicar el goal activo cada 0.5 s
#         self.goal_timer = self.create_timer(0.5, self.goal_timer_callback)

#     def frontier_callback(self, msg: PoseArray):
#         """Callback para recibir los puntos de frontera."""
#         self.get_logger().info('Recibidos {} puntos de frontera'.format(len(msg.poses)))
#         self.frontier_points = msg.poses

#         # Si hay puntos y no hay goal activo, intenta publicar uno nuevo.
#         if self.frontier_points and not self.current_goal_set:
#             self.publish_next_goal()

#     def occupied_nodes_callback(self, msg: PoseArray):
#         """Callback para recibir los puntos de obstáculos."""
#         self.get_logger().info('Recibidos {} puntos de obstáculos'.format(len(msg.poses)))
#         self.occupied_nodes = msg.poses

#     def goal_reached_callback(self, msg: Bool):
#         """Callback que se ejecuta al recibir la señal de que se alcanzó el goal."""
#         if msg.data:
#             self.get_logger().info('Señal de objetivo alcanzado recibida.')
#             self.current_goal_set = False
#             self.publish_next_goal()

#     def goal_timer_callback(self):
#         """Timer para republicar el goal actual cada 0.5 s si aún está activo."""
#         if self.current_goal_set and self.last_goal_pose is not None:
#             self.get_logger().debug('Republishing goal: ({:.2f}, {:.2f})'.format(
#                 self.last_goal_pose.position.x, self.last_goal_pose.position.y))
#             self.publish_goal(self.last_goal_pose)

#     def publish_goal(self, goal_pose: Pose):
#         """Publica el goal y su marcador para RViz."""
#         # Publicar como PoseArray
#         goal_msg = PoseArray()
#         goal_msg.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.header.frame_id = 'map'
#         goal_msg.poses.append(goal_pose)
#         self.goal_pub.publish(goal_msg)

#         # Publicar marcador para visualizar en RViz
#         marker = Marker()
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.header.frame_id = 'map'
#         marker.ns = "goal_marker"
#         marker.id = 0
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD
#         marker.pose = goal_pose
#         marker.scale.x = 2.0  # Ajusta el tamaño según necesites
#         marker.scale.y = 2.0
#         marker.scale.z = 2.0
#         marker.color.a = 1.0
#         marker.color.r = 1.0
#         marker.color.g = 0.0
#         marker.color.b = 0.0
#         self.goal_marker_pub.publish(marker)

#     def publish_next_goal(self):
#         """
#         Selecciona aleatoriamente un goal de los puntos de frontera que esté a una distancia segura
#         de los obstáculos y lo publica.
#         """
#         if not self.frontier_points:
#             self.get_logger().warn('No hay puntos de frontera disponibles para publicar.')
#             return

#         # Filtrar puntos de frontera que no estén demasiado cerca de ningún obstáculo
#         valid_frontier_points = []
#         for point in self.frontier_points:
#             too_close = False
#             for obst in self.occupied_nodes:
#                 if distance_between_poses(point, obst) < OBSTACLE_CLEARANCE_THRESHOLD:
#                     too_close = True
#                     break
#             if not too_close:
#                 valid_frontier_points.append(point)

#         if not valid_frontier_points:
#             self.get_logger().warn('No se encontraron puntos de frontera que cumplan con la distancia de seguridad.')
#             return

#         # Seleccionar aleatoriamente entre los puntos válidos
#         random_goal_pose = random.choice(valid_frontier_points)
#         self.get_logger().info(f'Publicando un nuevo goal: ({random_goal_pose.position.x:.2f}, {random_goal_pose.position.y:.2f})')
#         self.last_goal_pose = random_goal_pose
#         self.current_goal_set = True
#         self.publish_goal(random_goal_pose)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FrontierSelector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy, math, random
from collections import deque
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

# ---------- Parámetros ajustables ----------
OBSTACLE_COST_TH = 50          # ≥50 ⇒ ocupado
UNKNOWN_COST_TH  = -1          # -1 ⇒ desconocido (trátalo como obst.)
CLEARANCE_M      = 0.30        # [m] distancia mínima a obstáculo
MAX_SEARCH_M     = 2.0         # [m] radio máx. para “rescatar” el goal

class FrontierSelector(Node):
    def __init__(self):
        super().__init__('frontier_selector')     # ← ¡IMPORTANTE!

        # --- Suscripciones ---
        self.frontier_sub = self.create_subscription(
            PoseArray, '/safe_frontier_points', self.frontier_cb, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/occupancy_grid', self.map_cb, 10)
        self.goal_reached_sub = self.create_subscription(
            Bool, 'goal_reached', self.goal_reached_cb, 10)

        # --- Publicaciones ---
        self.goal_pub   = self.create_publisher(PoseArray, 'goal', 10)
        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)

        # --- Timer para re-publicar el goal activo ---
        self.timer = self.create_timer(0.5, self.timer_cb)

        # --- Estado interno ---
        self.frontiers: list[Pose] = []
        self.current_goal: Pose | None = None
        self.goal_active  = False
        self.grid_data    = None        # OccupancyGrid.data (lista de int8)
        self.grid_info    = None        # OccupancyGrid.info

    # ============ Callbacks ============

    def frontier_cb(self, msg: PoseArray):
        self.frontiers = msg.poses
        if self.frontiers and not self.goal_active:
            self.select_and_publish_goal()

    def map_cb(self, msg: OccupancyGrid):
        self.grid_data = msg.data
        self.grid_info = msg.info
        # Valida el goal actual cada vez que llega un mapa
        if self.goal_active and self.current_goal:
            if self.is_goal_in_collision(self.current_goal):
                self.get_logger().info(
                    'El goal actual ocupa o roza un obstáculo; buscándolo libre…')
                new_pose = self.closest_free_pose(self.current_goal)
                if new_pose:
                    self.current_goal = new_pose
                    self.publish_goal(new_pose)

    def goal_reached_cb(self, msg: Bool):
        if msg.data:
            self.goal_active = False
            self.select_and_publish_goal()

    def timer_cb(self):
        if self.goal_active and self.current_goal:
            self.publish_goal(self.current_goal)

    # ============ Conversión mundo <-> grid ============

    def world_to_grid(self, pose: Pose):
        if not self.grid_info:
            return None
        res = self.grid_info.resolution
        gx = int((pose.position.x - self.grid_info.origin.position.x) / res)
        gy = int((pose.position.y - self.grid_info.origin.position.y) / res)
        if 0 <= gx < self.grid_info.width and 0 <= gy < self.grid_info.height:
            return gx, gy
        return None

    def grid_to_world_pose(self, gx: int, gy: int, ref_orientation):
        res = self.grid_info.resolution
        p = Pose()
        p.position.x = self.grid_info.origin.position.x + (gx + 0.5) * res
        p.position.y = self.grid_info.origin.position.y + (gy + 0.5) * res
        p.orientation = ref_orientation
        return p

    # ============ Occupancy & Clearance ============

    def is_cell_occupied(self, gx: int, gy: int) -> bool:
        idx = gy * self.grid_info.width + gx
        if idx >= len(self.grid_data):
            return True             # fuera de rango ⇒ trátalo ocupado
        cost = self.grid_data[idx]
        return cost >= OBSTACLE_COST_TH or cost == UNKNOWN_COST_TH

    def has_clearance(self, cx: int, cy: int, rad_cells: int) -> bool:
        res = self.grid_info.resolution
        for dx in range(-rad_cells, rad_cells + 1):
            for dy in range(-rad_cells, rad_cells + 1):
                if math.hypot(dx, dy) * res >= CLEARANCE_M:
                    continue
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < self.grid_info.width and
                    0 <= ny < self.grid_info.height and
                    self.is_cell_occupied(nx, ny)):
                    return False
        return True

    def is_goal_in_collision(self, pose: Pose) -> bool:
        cell = self.world_to_grid(pose)
        return cell is not None and self.is_cell_occupied(*cell)

    # ============ Búsqueda BFS de la celda libre más cercana ============

    def closest_free_pose(self, origin_pose: Pose):
        start = self.world_to_grid(origin_pose)
        if start is None:
            return None
        res = self.grid_info.resolution
        max_cells = int(MAX_SEARCH_M / res)
        clearance_cells = int(CLEARANCE_M / res)

        q = deque([(start[0], start[1], 0)])
        visited = {start}
        neigh = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

        while q:
            x, y, d = q.popleft()
            if d > max_cells:
                break
            if (not self.is_cell_occupied(x, y) and
                self.has_clearance(x, y, clearance_cells)):
                return self.grid_to_world_pose(x, y, origin_pose.orientation)
            for dx, dy in neigh:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.grid_info.width and
                    0 <= ny < self.grid_info.height and
                    (nx, ny) not in visited):
                    visited.add((nx, ny))
                    q.append((nx, ny, d + 1))
        return None

    # ============ Gestión del objetivo ============

    def select_and_publish_goal(self):
        # Filtra fronteras que, con el mapa actual, estén libres
        valid = [p for p in self.frontiers if not self.is_goal_in_collision(p)]
        if not valid:
            self.get_logger().warn('No hay fronteras libres ahora mismo.')
            return
        self.current_goal = random.choice(valid)
        # Por si la celda acaba de marcarse ocupada entre ciclos…
        if self.is_goal_in_collision(self.current_goal):
            corr = self.closest_free_pose(self.current_goal)
            if corr:
                self.current_goal = corr
        self.goal_active = True
        self.publish_goal(self.current_goal)

    def publish_goal(self, pose: Pose):
        # PoseArray para planners
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'
        arr.poses.append(pose)
        self.goal_pub.publish(arr)

        # Marker RViz
        m          = Marker()
        m.header   = arr.header
        m.ns, m.id = 'goal_marker', 0
        m.type     = Marker.SPHERE
        m.action   = Marker.ADD
        m.pose     = pose
        m.scale.x = m.scale.y = m.scale.z = 2.3
        m.color.r, m.color.a = 1.0, 1.0
        self.marker_pub.publish(m)

# ----------------------- main -----------------------
def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(FrontierSelector())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
