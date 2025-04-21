import rclpy
from rclpy.node import Node
import math
import numpy as np
import heapq
import threading
import time
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Pose, PoseStamped, Twist, Point, PoseArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

### FUNCIONES AUXILIARES COMUNES ###

def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def index_to_world(i, j, info):
    x = info.origin.position.x + (i + 0.5) * info.resolution
    y = info.origin.position.y + (j + 0.5) * info.resolution
    return (x, y)

def planificador_modificable(costmap, info, start, goal, max_iters=1000):
    """
    Genera un path desde 'start' hasta 'goal' usando una búsqueda voraz incremental.
    Permite modificar la heurística para mayor flexibilidad.
    """
    # Convertir posiciones en el mundo a índices en la grilla.
    start_idx = (int((start[0]-info.origin.position.x) / info.resolution),
                 int((start[1]-info.origin.position.y) / info.resolution))
    goal_idx = (int((goal[0]-info.origin.position.x) / info.resolution),
                int((goal[1]-info.origin.position.y) / info.resolution))
    
    path = [start]  # iniciar el path en la posición inicial
    
    # Parámetros de la función heurística:
    alpha = 1.0   # peso para la distancia al goal
    beta = 10.0   # peso para penalizar el costo de la celda
    
    threshold = 1.0  # distancia de parada en metros
    current_idx = start_idx
    iters = 0

    # Vecinos 8-direccionales.
    vecinos = [(-1, 0), (1, 0), (0, -1), (0, 1),
               (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    while distance(index_to_world(current_idx[0], current_idx[1], info), goal) > threshold and iters < max_iters:
        iters += 1
        candidatos = []
        current_world = index_to_world(current_idx[0], current_idx[1], info)
        
        # Evaluar vecinos
        for dx, dy in vecinos:
            ni = current_idx[0] + dx
            nj = current_idx[1] + dy
            if ni < 0 or ni >= info.width or nj < 0 or nj >= info.height:
                continue
            # Evitar celdas con obstáculo
            if costmap[nj, ni] >= 100:
                continue
            vecino_world = index_to_world(ni, nj, info)
            d_goal = distance(vecino_world, goal)
            costo_celda = costmap[nj, ni]
            h = alpha * d_goal + beta * costo_celda
            candidatos.append(((ni, nj), h, vecino_world))
        
        if not candidatos:
            print("No hay vecinos viables, rompiendo la búsqueda")
            break

        # Seleccionar el vecino con la menor heurística
        candidatos.sort(key=lambda x: x[1])
        next_idx, _, next_world = candidatos[0]
        
        if next_idx == current_idx:
            break
        
        path.append(next_world)
        current_idx = next_idx

    return path

def a_star_planning(costmap, info, start_idx, goal_idx):
    height, width = costmap.shape
    open_set = []
    heapq.heappush(open_set, (0, start_idx))
    came_from = {}
    g_score = {start_idx: 0}
    f_score = {start_idx: distance(index_to_world(start_idx[0], start_idx[1], info),
                                    index_to_world(goal_idx[0], goal_idx[1], info))}
    neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == goal_idx:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_idx)
            path.reverse()
            return path
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor[0] < 0 or neighbor[0] >= width or neighbor[1] < 0 or neighbor[1] >= height:
                continue
            if costmap[neighbor[1], neighbor[0]] == 100:
                continue  # obstáculo
            tentative_g = g_score[current] + math.hypot(dx, dy)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                neighbor_world = index_to_world(neighbor[0], neighbor[1], info)
                h = distance(neighbor_world, index_to_world(goal_idx[0], goal_idx[1], info))
                f = tentative_g + h
                f_score[neighbor] = f
                heapq.heappush(open_set, (f, neighbor))
    return None

def extract_frontiers(costmap, info):
    height, width = costmap.shape
    frontier_cells = []
    for j in range(height):
        for i in range(width):
            if costmap[j, i] == 0:
                for di in [-1, 0, 1]:
                    for dj in [-1, 0, 1]:
                        if di == 0 and dj == 0:
                            continue
                        ni, nj = i + di, j + dj
                        if 0 <= ni < width and 0 <= nj < height:
                            if costmap[nj, ni] == -1:
                                frontier_cells.append((i, j))
                                break
    clusters = []
    visited = set()
    for cell in frontier_cells:
        if cell in visited:
            continue
        cluster = []
        stack = [cell]
        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)
            cluster.append(current)
            ci, cj = current
            for di in [-1, 0, 1]:
                for dj in [-1, 0, 1]:
                    neighbor = (ci + di, cj + dj)
                    if neighbor in frontier_cells and neighbor not in visited:
                        stack.append(neighbor)
        clusters.append(cluster)
    centroids = []
    for cluster in clusters:
        xs = [i for i, j in cluster]
        ys = [j for i, j in cluster]
        centroid_i = sum(xs) / len(xs)
        centroid_j = sum(ys) / len(ys)
        centroids.append(index_to_world(centroid_i, centroid_j, info))
    return centroids

### NODO DE NAVEGACIÓN CON ACTUALIZACIÓN DINÁMICA DEL PATH ###

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self.current_grid_callback, 10)
        self.create_subscription(OccupancyGrid, '/persistent_dynamic_occupancy_grid', self.memory_grid_callback, 10)
        self.create_subscription(PoseArray, 'safe_frontier_points_centroid', self.safe_frontier_callback, 10)
        self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)
        
        # Publicadores
        self.path_pub = self.create_publisher(Path, '/global_path_predicted', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.frontier_marker_pub = self.create_publisher(Marker, '/frontier_markers', 10)
        
        # Parámetros de planificación y control
        self.lookahead_distance = 10.0  
        self.linear_speed = 2.5
        self.k_pursuit = 3.0
        self.goal_threshold = 2.0
        self.inflation_radius = 0.5
        self.cost_weight = 0.1

        # Variables de estado
        self.current_global_path = []  
        self.current_path_index = 0
        self.waypoint_threshold = 0.8
        self.last_replan_position = None  
        self.odom = None
        self.goal = None
        self.goal_reached = False
        self.current_grid = None
        self.memory_grid = None
        self.safe_frontier_points = []  
        self.current_subgoal = None  

        # Parámetro para eliminación de frontier alcanzados
        self.frontier_reached_threshold = 1.5

        self.last_twist = Twist()
        self.twist_pub_rate = 20
        self.start_twist_publisher()
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nodo de navegación (dinámico) iniciado.")

    # ---------- CALLBACKS ------------
    def odom_callback(self, msg):
        self.odom = msg.pose.pose
        

    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]
            self.goal_reached = False  # Se resetea la bandera para el nuevo goal
            self.get_logger().info("Goal recibido.")

    def current_grid_callback(self, msg: OccupancyGrid):
        self.current_grid = msg

    def memory_grid_callback(self, msg: OccupancyGrid):
        self.memory_grid = msg

    def goal_reached_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Goal alcanzado. Reiniciando path y safe frontier points.")
            self.current_global_path = []
            self.current_path_index = 0
            self.goal_reached = True

    def safe_frontier_callback(self, msg: PoseArray):
        nuevos = [(p.position.x, p.position.y) for p in msg.poses]
        # Agregar solo puntos que no estén ya cercanos a los existentes.
        for pt in nuevos:
            if all(distance(pt, cand) > 0.5 for cand in self.safe_frontier_points):
                self.safe_frontier_points.append(pt)
        self.get_logger().debug(f"Safe frontier points: {len(self.safe_frontier_points)}")

    # ---------- FUNCIONES AUXILIARES -----------
    def world_to_index(self, point, info):
        i = int((point[0] - info.origin.position.x) / info.resolution)
        j = int((point[1] - info.origin.position.y) / info.resolution)
        return (i, j)

    def publish_path(self, path, publisher, label):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for pt in path:
            pose_st = PoseStamped()
            pose_st.header = path_msg.header
            pose_st.pose.position.x = pt[0]
            pose_st.pose.position.y = pt[1]
            pose_st.pose.position.z = 0.0
            pose_st.pose.orientation.w = 1.0
            path_msg.poses.append(pose_st)
        publisher.publish(path_msg)
        self.get_logger().info(f"[{label}] Path publicado con {len(path)} puntos.")

    def publish_segment_marker(self, segment):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path_segment"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        for pt in segment:
            p = Point()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            marker.points.append(p)
        self.frontier_marker_pub.publish(marker)

    def start_twist_publisher(self):
        thread = threading.Thread(target=self.publish_twist_continuously, daemon=True)
        thread.start()

    def publish_twist_continuously(self):
        rate = 1.0 / self.twist_pub_rate
        while rclpy.ok():
            self.cmd_vel_pub.publish(self.last_twist)
            time.sleep(rate)

    def create_costmap_from_fused_grid(self, fused_grid, info):
        factor_robot = 1.5
        inflation_cells = int(math.ceil((self.inflation_radius * factor_robot) / info.resolution))
        costmap = np.copy(fused_grid)
        height, width = fused_grid.shape
        for j in range(height):
            for i in range(width):
                if fused_grid[j, i] == 100:
                    for dj in range(-inflation_cells, inflation_cells+1):
                        for di in range(-inflation_cells, inflation_cells+1):
                            ni = i + di
                            nj = j + dj
                            if 0 <= ni < width and 0 <= nj < height:
                                if math.hypot(di, dj) <= inflation_cells:
                                    costmap[nj, ni] = 100
        return costmap

    def is_line_free(self, grid, info, start, end):
        def world_to_index(point, info):
            i = int((point[0] - info.origin.position.x) / info.resolution)
            j = int((point[1] - info.origin.position.y) / info.resolution)
            return i, j
        x0, y0 = world_to_index(start, info)
        x1, y1 = world_to_index(end, info)
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        def is_in_bounds(x, y, grid):
            return 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if not is_in_bounds(x, y, grid) or grid[y, x] == 100:
                    return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if not is_in_bounds(x, y, grid) or grid[y, x] == 100:
                    return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        if not is_in_bounds(x, y, grid) or grid[y, x] == 100:
            return False
        return True

    def remove_reached_frontiers(self, current_pos):
        nuevos = []
        for pt in self.safe_frontier_points:
            if distance(current_pos, pt) > self.frontier_reached_threshold:
                nuevos.append(pt)
            else:
                self.get_logger().info(f"Frontier alcanzada y removida: {pt}")
        self.safe_frontier_points = nuevos

    def select_intermediate_goal(self, current_pos, goal_pos):
        """
        Selecciona un subgoal de los safe frontier candidates.
        Se verifica que exista línea de vista entre la posición actual y el candidato.
        """
        if not self.safe_frontier_points:
            return None
        candidatos = []
        # Se convierte la grilla actual en 2D para usar en la verificación de línea libre.
        if self.current_grid is not None:
            grid_data = np.array(self.current_grid.data, dtype=np.int8).reshape((self.current_grid.info.height, self.current_grid.info.width))
        else:
            grid_data = None

        for pt in self.safe_frontier_points:
            if grid_data is not None and self.is_line_free(grid_data, self.current_grid.info, current_pos, pt):
                score = distance(current_pos, pt) + distance(pt, goal_pos)
                candidatos.append((pt, score))
            else:
                self.get_logger().debug(f"Punto {pt} descartado por falta de línea de vista.")
        if not candidatos:
            return None
        best_point, _ = min(candidatos, key=lambda x: x[1])
        self.get_logger().info(f"Subgoal intermedio seleccionado: {best_point}")
        return best_point

    def is_goal_known(self, goal_pos, costmap, info):
        i, j = self.world_to_index(goal_pos, info)
        if 0 <= i < costmap.shape[1] and 0 <= j < costmap.shape[0]:
            return costmap[j, i] != -1
        return False

    def global_plan_a_star(self, costmap, info, start, goal):
        start_idx = self.world_to_index(start, info)
        goal_idx = self.world_to_index(goal, info)
        grid_path = a_star_planning(costmap, info, start_idx, goal_idx)
        if grid_path is None:
            self.get_logger().error("A* no encontró path.")
            return None
        path = [index_to_world(i, j, info) for (i, j) in grid_path]
        return path

    def update_path_with_current_position(self, current_pos):
        # Encuentra el waypoint más cercano a la posición actual.
        min_dist = float('inf')
        index_closest = 0
        for i, pt in enumerate(self.current_global_path):
            d = distance(current_pos, pt)
            if d < min_dist:
                min_dist = d
                index_closest = i
        self.current_path_index = index_closest

    def stuck_in_waypoint(self, current_pos):
        """
        Retorna True si el robot parece estar atascado en el mismo waypoint.
        Por ejemplo, si el avance desde la última replanificación es menor que un umbral.
        """
        if self.last_replan_position:
            progreso = distance(current_pos, self.last_replan_position)
            if progreso < 0.5:  # Umbral ajustable
                return True
        return False

    def publicar_frontiers(self, centroids, info):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for pt in centroids:
            p = Point()
            p.x, p.y = pt[0], pt[1]
            p.z = 0.0
            marker.points.append(p)
        self.frontier_marker_pub.publish(marker)

    # ---------- CICLO DE CONTROL -----------
    def control_loop(self):
        if self.odom is None or self.goal is None:
            return

        # Se utiliza la grilla actual o la memoria si la actual no está disponible.
        grid_msg = self.current_grid if self.current_grid is not None else self.memory_grid
        if grid_msg is None:
            self.get_logger().warn("No se recibió OccupancyGrid aún.")
            return

        info = grid_msg.info
        fused_grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
        current_pos = (self.odom.position.x, self.odom.position.y)
        goal_pos = (self.goal.position.x, self.goal.position.y)
        costmap = self.create_costmap_from_fused_grid(fused_grid, info)

        # Actualizar safe frontier points eliminando los alcanzados.
        self.remove_reached_frontiers(current_pos)

        if self.goal_reached:
            self.get_logger().info("Goal ya alcanzado. Deteniendo navegación.")
            return

        # Determinar el objetivo (goal o subgoal) según la visibilidad en el mapa.
        if not self.is_goal_known(goal_pos, costmap, info):
            self.get_logger().warn("Goal fuera del área mapeada. Buscando subgoal.")
            subgoal = self.select_intermediate_goal(current_pos, goal_pos)
            if subgoal is None:
                self.get_logger().error("No se ha podido seleccionar un subgoal intermedio.")
                return
            self.current_subgoal = subgoal
        else:
            self.current_subgoal = goal_pos

        # Forzar replanificación en condiciones:
        #   1. No hay path actual.
        #   2. El robot se ha desviado desde la última replanificación.
        #   3. Se detecta que el robot está atascado.
        if (not self.current_global_path or 
            (self.last_replan_position and distance(current_pos, self.last_replan_position) > self.lookahead_distance * 0.5) or 
            self.stuck_in_waypoint(current_pos)):
            new_path = planificador_modificable(costmap, info, current_pos, self.current_subgoal)
            if not new_path or len(new_path) < 2:
                self.get_logger().error("El planificador modificable no generó un path válido.")
                return
            new_path = self.smooth_path(new_path)
            self.current_global_path = new_path
            self.current_path_index = 0
            self.publish_path(self.current_global_path, self.path_pub, "Global")
            self.last_replan_position = current_pos

        # Actualizar el path para alinearlo con la posición actual.
        self.update_path_with_current_position(current_pos)

        # Si se alcanza el subgoal actual, eliminarlo y seleccionar uno nuevo.
        if distance(current_pos, self.current_subgoal) < self.frontier_reached_threshold:
            self.get_logger().info(f"Subgoal alcanzado: {self.current_subgoal}")
            try:
                self.safe_frontier_points.remove(self.current_subgoal)
            except ValueError:
                pass
            nuevo_subgoal = self.select_intermediate_goal(current_pos, goal_pos)
            if nuevo_subgoal is not None:
                self.current_subgoal = nuevo_subgoal
                self.get_logger().info(f"Nuevo subgoal seleccionado: {nuevo_subgoal}")
                new_path = planificador_modificable(costmap, info, current_pos, self.current_subgoal)
                if new_path and len(new_path) >= 2:
                    new_path = self.smooth_path(new_path)
                    self.current_global_path = new_path
                    self.current_path_index = 0
                    self.publish_path(self.current_global_path, self.path_pub, "Global")
                    self.last_replan_position = current_pos

        # Publicar segmento actual para visualización.
        segment = self.current_global_path[self.current_path_index:] if self.current_global_path else []
        self.publish_segment_marker(segment)

        # Control de seguimiento: pure pursuit.
        if not segment or len(segment) == 0:
            self.get_logger().warn("No se encontró un segmento válido de path.")
            return

        q = self.odom.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        linear_speed, angular_speed = self.pure_pursuit_control(segment, current_pos, current_yaw)
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.last_twist = twist
        self.get_logger().info(f"Twist: {linear_speed:.2f} m/s, {angular_speed:.2f} rad/s")

    def pure_pursuit_control(self, path, current_pos, current_yaw):
        # Seleccionar el siguiente punto según la distancia lookahead.
        target_point = None
        for pt in path:
            if distance(current_pos, pt) >= self.lookahead_distance:
                target_point = pt
                break
        if target_point is None:
            target_point = path[-1]
        desired_heading = math.atan2(target_point[1] - current_pos[1],
                                     target_point[0] - current_pos[0])
        error_angle = desired_heading - current_yaw
        error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
        return self.linear_speed, self.k_pursuit * error_angle

    def smooth_path(self, path):
        if len(path) < 3:
            return path
        smoothed = [path[0]]
        for i in range(1, len(path)-1):
            avg_x = (path[i-1][0] + path[i][0] + path[i+1][0]) / 3.0
            avg_y = (path[i-1][1] + path[i][1] + path[i+1][1]) / 3.0
            smoothed.append((avg_x, avg_y))
        smoothed.append(path[-1])
        return smoothed

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
