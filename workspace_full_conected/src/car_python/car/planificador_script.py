#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math
import numpy as np
import heapq
from nav_msgs.msg import Odometry, OccupancyGrid, Path, MapMetaData
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Twist, Point
import threading
import time

# Función auxiliar para calcular la distancia Euclidiana
def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# Algoritmo de Dijkstra para planificar el camino en un grafo
def dijkstra(graph, start, goal):
    dist = {node: float('inf') for node in graph.keys()}
    prev = {node: None for node in graph.keys()}
    dist[start] = 0
    queue = [(0, start)]
    while queue:
        current_dist, current_node = heapq.heappop(queue)
        if current_node == goal:
            break
        if current_dist > dist[current_node]:
            continue
        for neighbor, weight in graph[current_node]:
            alt = current_dist + weight
            if alt < dist[neighbor]:
                dist[neighbor] = alt
                prev[neighbor] = current_node
                heapq.heappush(queue, (alt, neighbor))
    # Reconstruir el camino
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = prev[node]
    path.reverse()
    return path

# Función para convertir índices del mapa a coordenadas en el mundo
def index_to_world(i, j, info):
    x = info.origin.position.x + (i + 0.5) * info.resolution
    y = info.origin.position.y + (j + 0.5) * info.resolution
    return (x, y)

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        # Suscriptores
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self.current_grid_callback, 10)
        self.create_subscription(OccupancyGrid, '/persistent_dynamic_occupancy_grid', self.memory_grid_callback, 10)
        
        # Publicadores
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parámetros para la fusión y planificación
        self.connection_threshold = 3.0        # Umbral para conectar nodos
        self.lookahead_distance = 1.0            # Para el controlador Pure Pursuit
        self.linear_speed = 5.0                # Velocidad lineal base
        self.k_pursuit = 3.0                   # Ganancia angular
        
        # Variables para mantener el path global y sus actualizaciones
        self.last_valid_path = None            # Último path aceptado
        self.path_update_threshold = 0.3         # Umbral para actualizar el camino (por ejemplo, 30% de diferencia)
        self.last_replan_time = 0.0
        self.replan_interval = 3.0               # Intervalo mínimo en segundos para replanificar
        self.path_change_threshold = 0.3         # Umbral mínimo para actualizar el camino
        self.current_global_path = None          # Path global persistente
        self.current_path_index = 0              # Índice del siguiente waypoint a seguir
        self.waypoint_threshold = 2.0            # Umbral para considerar que se alcanzó un waypoint

        # Buffer para registrar datos para entrenamiento
        self.training_buffer = []

        # Variables de estado
        self.odom = None
        self.goal = None
        self.current_grid = None
        self.memory_grid = None

        # Publicación continua del Twist
        self.last_twist = Twist()
        self.twist_pub_rate = 20  # Hz
        self.start_twist_publisher()

        # Timer para el control loop (10 Hz)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nodo de navegación iniciado.")

    def start_twist_publisher(self):
        thread = threading.Thread(target=self.publish_twist_continuously, daemon=True)
        thread.start()

    def publish_twist_continuously(self):
        rate = 1.0 / self.twist_pub_rate
        while rclpy.ok():
            self.cmd_vel_pub.publish(self.last_twist)
            time.sleep(rate)

    def odom_callback(self, msg: Odometry):
        self.odom = msg.pose.pose

    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]
            self.get_logger().info("Goal recibido.")

    def current_grid_callback(self, msg: OccupancyGrid):
        self.current_grid = msg

    def memory_grid_callback(self, msg: OccupancyGrid):
        self.memory_grid = msg

    def compute_dynamic_twist(self, current_pos, current_yaw, path, fused_grid, info):
        # Velocidades base usando Pure Pursuit
        linear_speed_base, angular_speed = self.pure_pursuit_control(path, current_pos, current_yaw)
        
        # Calcular error angular respecto al siguiente waypoint (si existe)
        target_point = path[min(1, len(path)-1)]
        desired_heading = math.atan2(target_point[1] - current_pos[1],
                                     target_point[0] - current_pos[0])
        error_angle = math.atan2(math.sin(desired_heading - current_yaw),
                                 math.cos(desired_heading - current_yaw))
        
        # Reducir velocidad si el error angular es alto
        reduction_factor = max(0.1, 1 - abs(error_angle))
        linear_speed = linear_speed_base * reduction_factor

        # Comprobar obstáculos locales para activar retroceso si es necesario
        safe_distance = 0.5
        obstacle_detected, obs_distance = self.check_local_obstacles(fused_grid, info, current_pos, current_yaw)
        if obstacle_detected and obs_distance < safe_distance:
            self.get_logger().warn(f"Obstáculo muy cerca ({obs_distance:.2f} m), activando retroceso.")
            linear_speed = -abs(linear_speed_base)
        
        return linear_speed, angular_speed

    def pure_pursuit_control(self, path, current_pos, current_yaw):
        # Buscar el primer punto en el path a una distancia >= lookahead_distance
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
        linear_speed = self.linear_speed
        angular_speed = self.k_pursuit * error_angle
        return linear_speed, angular_speed

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

    def path_quality_improved(self, new_path, old_path):
        if old_path is None:
            return True
        len_new = len(new_path)
        len_old = len(old_path)
        return (len_new - len_old) / (len_old + 1e-6) > self.path_update_threshold

    def check_local_obstacles(self, fused_grid, info, current_pos, current_yaw, lookahead_distance=1.0, cone_angle=0.5):
        ox = info.origin.position.x
        oy = info.origin.position.y
        resolution = info.resolution
        grid_height, grid_width = fused_grid.shape

        obstacle_detected = False
        min_distance = float('inf')
        steps = int(lookahead_distance / resolution)
        for step in range(1, steps + 1):
            dist = step * resolution
            num_angles = 5
            for a in np.linspace(-cone_angle, cone_angle, num_angles):
                angle = current_yaw + a
                x = current_pos[0] + dist * math.cos(angle)
                y = current_pos[1] + dist * math.sin(angle)
                i = int((x - ox) / resolution)
                j = int((y - oy) / resolution)
                if 0 <= i < grid_width and 0 <= j < grid_height:
                    cell_value = fused_grid[j, i]
                    if cell_value == 100 or cell_value == -1:
                        obstacle_detected = True
                        if dist < min_distance:
                            min_distance = dist
                else:
                    obstacle_detected = True
                    if dist < min_distance:
                        min_distance = dist
        return obstacle_detected, min_distance

    def fuse_maps_dynamic(self, grid_msg1, grid_msg2):
        info1 = grid_msg1.info if grid_msg1 is not None else None
        info2 = grid_msg2.info if grid_msg2 is not None else None
        if info1 is None and info2 is None:
            return None, None

        def get_bounds(info):
            ox = info.origin.position.x
            oy = info.origin.position.y
            max_x = ox + info.width * info.resolution
            max_y = oy + info.height * info.resolution
            return ox, oy, max_x, max_y

        bounds = []
        if info1 is not None:
            bounds.append(get_bounds(info1))
        if info2 is not None:
            bounds.append(get_bounds(info2))
        
        min_x = min(b[0] for b in bounds)
        min_y = min(b[1] for b in bounds)
        max_x = max(b[2] for b in bounds)
        max_y = max(b[3] for b in bounds)

        margin = 5.0
        min_x -= margin
        min_y -= margin
        max_x += margin
        max_y += margin

        resolution = info1.resolution if info1 is not None else info2.resolution
        new_width = int(math.ceil((max_x - min_x) / resolution))
        new_height = int(math.ceil((max_y - min_y) / resolution))

        fused_grid = -1 * np.ones((new_height, new_width), dtype=np.int8)

        def reproject_map(grid_msg, fused_grid, new_origin_x, new_origin_y, resolution):
            info = grid_msg.info
            grid_array = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
            for j in range(info.height):
                for i in range(info.width):
                    x = info.origin.position.x + (i + 0.5) * resolution
                    y = info.origin.position.y + (j + 0.5) * resolution
                    new_i = int((x - new_origin_x) / resolution)
                    new_j = int((y - new_origin_y) / resolution)
                    if 0 <= new_i < fused_grid.shape[1] and 0 <= new_j < fused_grid.shape[0]:
                        value = grid_array[j, i]
                        if value == 100:
                            fused_grid[new_j, new_i] = 100
                        elif value == 0 and fused_grid[new_j, new_i] != 100:
                            fused_grid[new_j, new_i] = 0
            return fused_grid

        if grid_msg1 is not None:
            fused_grid = reproject_map(grid_msg1, fused_grid, min_x, min_y, resolution)
        if grid_msg2 is not None:
            fused_grid = reproject_map(grid_msg2, fused_grid, min_x, min_y, resolution)

        new_info = MapMetaData()
        new_info.resolution = resolution
        new_info.width = new_width
        new_info.height = new_height
        new_info.origin.position.x = min_x
        new_info.origin.position.y = min_y
        new_info.origin.position.z = 0.0
        new_info.origin.orientation.w = 1.0

        return fused_grid, new_info

    def extract_nodes(self, fused_grid, info):
        nodes = []
        for j in range(fused_grid.shape[0]):
            for i in range(fused_grid.shape[1]):
                if fused_grid[j, i] == 0:
                    nodes.append(index_to_world(i, j, info))
        return nodes

    def project_goal(self, goal_pos, info):
        ox = info.origin.position.x
        oy = info.origin.position.y
        max_x = ox + info.width * info.resolution
        max_y = oy + info.height * info.resolution
        x, y = goal_pos
        x = min(max(x, ox), max_x)
        y = min(max(y, oy), max_y)
        return (x, y)

    def rrt_plan_path(self, start, goal, fused_grid, info, max_iterations=1000, step_size=1.0, goal_threshold=1.0, goal_bias=0.2):
        tree = [{'point': start, 'parent': None}]
        for i in range(max_iterations):
            ox = info.origin.position.x
            oy = info.origin.position.y
            max_x = ox + info.width * info.resolution
            max_y = oy + info.height * info.resolution

            if np.random.rand() < goal_bias:
                random_point = goal
            else:
                random_point = (np.random.uniform(ox, max_x), np.random.uniform(oy, max_y))
            
            nearest = min(tree, key=lambda node: distance(node['point'], random_point))
            theta = math.atan2(random_point[1] - nearest['point'][1],
                               random_point[0] - nearest['point'][0])
            new_point = (nearest['point'][0] + step_size * math.cos(theta),
                         nearest['point'][1] + step_size * math.sin(theta))
            if not self.is_line_free(fused_grid, info, nearest['point'], new_point):
                continue
            new_node = {'point': new_point, 'parent': nearest}
            tree.append(new_node)
            if distance(new_point, goal) < goal_threshold and self.is_line_free(fused_grid, info, new_point, goal):
                goal_node = {'point': goal, 'parent': new_node}
                tree.append(goal_node)
                path = []
                current = goal_node
                while current is not None:
                    path.append(current['point'])
                    current = current['parent']
                path.reverse()
                return path
        return None
#buena
    # def is_line_free(self, fused_grid, info, start, end):
    #     def world_to_index(point, info):
    #         i = int((point[0] - info.origin.position.x) / info.resolution)
    #         j = int((point[1] - info.origin.position.y) / info.resolution)
    #         return i, j

    #     x0, y0 = world_to_index(start, info)
    #     x1, y1 = world_to_index(end, info)

    #     dx = abs(x1 - x0)
    #     dy = abs(y1 - y0)
    #     x, y = x0, y0
    #     sx = -1 if x0 > x1 else 1
    #     sy = -1 if y0 > y1 else 1
    #     if dx > dy:
    #         err = dx / 2.0
    #         while x != x1:
    #             if fused_grid[y, x] == 100:
    #                 return False
    #             err -= dy
    #             if err < 0:
    #                 y += sy
    #                 err += dx
    #             x += sx
    #     else:
    #         err = dy / 2.0
    #         while y != y1:
    #             if fused_grid[y, x] == 100:
    #                 return False
    #             err -= dx
    #             if err < 0:
    #                 x += sx
    #                 err += dy
    #             y += sy
    #     if fused_grid[y, x] == 100:
    #         return False
    #     return True
    def is_line_free(self, fused_grid, info, start, end):
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

        # Función auxiliar para verificar límites
        def is_in_bounds(x, y, grid):
            return 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if not is_in_bounds(x, y, fused_grid) or fused_grid[y, x] == 100:
                    return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if not is_in_bounds(x, y, fused_grid) or fused_grid[y, x] == 100:
                    return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        if not is_in_bounds(x, y, fused_grid) or fused_grid[y, x] == 100:
            return False
        return True

    # def plan_path(self, nodes, fused_grid, info):
    #     if self.odom is None or self.goal is None:
    #         return None
    #     robot_pos = (self.odom.position.x, self.odom.position.y)
    #     goal_pos = (self.goal.position.x, self.goal.position.y)
    #     if self.current_grid is not None:
    #         goal_pos = self.project_goal(goal_pos, self.current_grid.info)
    #     elif self.memory_grid is not None:
    #         goal_pos = self.project_goal(goal_pos, self.memory_grid.info)
        
    #     all_nodes = nodes.copy()
    #     all_nodes.append(robot_pos)
    #     all_nodes.append(goal_pos)
    #     robot_index = len(all_nodes) - 2
    #     goal_index = len(all_nodes) - 1
        
    #     graph = {i: [] for i in range(len(all_nodes))}
    #     for i in range(len(all_nodes)):
    #         for j in range(i + 1, len(all_nodes)):
    #             d = distance(all_nodes[i], all_nodes[j])
    #             if d <= self.connection_threshold:
    #                 if self.is_line_free(fused_grid, info, all_nodes[i], all_nodes[j]):
    #                     graph[i].append((j, d))
    #                     graph[j].append((i, d))
    #     path_indices = dijkstra(graph, robot_index, goal_index)
    #     if not path_indices or len(path_indices) < 3:
    #         self.get_logger().warn("Path en línea recta detectado. Forzando RRT.")
    #         return None
    #     path = [all_nodes[i] for i in path_indices]
    #     return path

    def plan_path(self, nodes, fused_grid, info):
        if self.odom is None or self.goal is None:
            return None
        robot_pos = (self.odom.position.x, self.odom.position.y)
        goal_pos = (self.goal.position.x, self.goal.position.y)
        if self.current_grid is not None:
            goal_pos = self.project_goal(goal_pos, self.current_grid.info)
        elif self.memory_grid is not None:
            goal_pos = self.project_goal(goal_pos, self.memory_grid.info)
        
        all_nodes = nodes.copy()
        all_nodes.append(robot_pos)
        all_nodes.append(goal_pos)
        robot_index = len(all_nodes) - 2
        goal_index = len(all_nodes) - 1
        
        graph = {i: [] for i in range(len(all_nodes))}
        for i in range(len(all_nodes)):
            for j in range(i + 1, len(all_nodes)):
                d = distance(all_nodes[i], all_nodes[j])
                if d <= self.connection_threshold:
                    if self.is_line_free(fused_grid, info, all_nodes[i], all_nodes[j]):
                        graph[i].append((j, d))
                        graph[j].append((i, d))
        path_indices = dijkstra(graph, robot_index, goal_index)
        if not path_indices or len(path_indices) < 3:
            self.get_logger().warn("Path global insuficiente (solo 2 puntos). Forzando RRT.")
            return None  # Se fuerza replan usando otro método
        path = [all_nodes[i] for i in path_indices]
        
        # Verificar que cada segmento del path sea seguro
        for i in range(len(path) - 1):
            if not self.is_line_free(fused_grid, info, path[i], path[i+1]):
                self.get_logger().warn(f"Segmento inseguro entre {path[i]} y {path[i+1]}. Forzando replan.")
                return None
        return path


    def update_path_index(self, current_pos):
        if self.current_global_path is None or len(self.current_global_path) == 0:
            return
        old_index = self.current_path_index
        # Mientras la distancia al waypoint actual sea menor que el umbral, avanzar el índice.
        while (self.current_path_index < len(self.current_global_path) and 
            distance(current_pos, self.current_global_path[self.current_path_index]) < self.waypoint_threshold):
            self.get_logger().info(f"Waypoint alcanzado: índice {self.current_path_index}, distancia {distance(current_pos, self.current_global_path[self.current_path_index]):.2f} m")
            self.current_path_index += 1
        if self.current_path_index >= len(self.current_global_path):
            self.current_path_index = len(self.current_global_path) - 1
        if old_index != self.current_path_index:
            self.get_logger().info(f"Índice actualizado: {old_index} -> {self.current_path_index}")

    def publish_path(self, path):
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
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path global publicado con {len(path)} puntos.")

    def fallback_path(self, current_pos, goal_pos, fused_grid, info, step_size=0.2, offset_step=0.3, max_offset=1.0):
        """
        Genera un path incremental entre current_pos y goal_pos en pasos de step_size.
        Si algún segmento no es seguro (is_line_free falla), se intenta desplazar el punto
        intermedio en dirección perpendicular hasta encontrar una posición segura.
        """
        # Generar la línea base (recta) con varios puntos
        path = []
        total_dist = distance(current_pos, goal_pos)
        num_steps = int(total_dist / step_size)
        if num_steps < 1:
            num_steps = 1
        for i in range(num_steps + 1):
            ratio = i / float(num_steps)
            x = current_pos[0] + ratio * (goal_pos[0] - current_pos[0])
            y = current_pos[1] + ratio * (goal_pos[1] - current_pos[1])
            path.append((x, y))
        
        # Ahora ajustar los puntos intermedios si el segmento no es seguro
        safe_path = [path[0]]
        for i in range(1, len(path)):
            prev_pt = safe_path[-1]
            pt = path[i]
            # Verificar si el segmento es seguro
            if self.is_line_free(fused_grid, info, prev_pt, pt):
                safe_path.append(pt)
            else:
                # Calcular la dirección del segmento y su perpendicular
                dx = pt[0] - prev_pt[0]
                dy = pt[1] - prev_pt[1]
                theta = math.atan2(dy, dx)
                perp_angle = theta + math.pi / 2  # dirección perpendicular
                
                found_safe = False
                # Probar a desplazar en ambas direcciones
                for sign in [1, -1]:
                    offset = offset_step
                    while offset <= max_offset:
                        new_pt = (pt[0] + sign * offset * math.cos(perp_angle),
                                pt[1] + sign * offset * math.sin(perp_angle))
                        # Verificar si el segmento modificado es seguro
                        if self.is_line_free(fused_grid, info, prev_pt, new_pt):
                            safe_path.append(new_pt)
                            found_safe = True
                            break
                        offset += offset_step
                    if found_safe:
                        break
                # Si no se encontró ninguna alternativa segura, se añade el punto original
                if not found_safe:
                    safe_path.append(pt)
        return safe_path


    def control_loop(self):
        if self.odom is None or self.goal is None:
            return

        fused_result = self.fuse_maps_dynamic(self.current_grid, self.memory_grid)
        if fused_result is None or fused_result[0] is None:
            self.get_logger().warn("No se pudo fusionar mapas. Esperando datos válidos...")
            return
        fused_grid, info = fused_result
        current_pos = (self.odom.position.x, self.odom.position.y)
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Replanificación global: si no existe path o se fuerza replanteo
        if self.current_global_path is None or self.should_replan(current_pos):
            nodes = self.extract_nodes(fused_grid, info)
            new_path = self.plan_path(nodes, fused_grid, info)
            if new_path is None or len(new_path) < 3:
                self.get_logger().warn("Planificación convencional devolvió un path insuficiente (línea recta). Forzando RRT.")
                new_path = self.rrt_plan_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
                if new_path is None or len(new_path) < 3:
                    self.get_logger().warn("RRT falló, usando fallback path incremental.")
                    new_path = self.fallback_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
                    if new_path is None or len(new_path) < 3:
                        self.get_logger().error("Fallback path también falló, abortando.")
                        return
            new_path = self.smooth_path(new_path)
            self.current_global_path = new_path
            self.current_path_index = 0
            self.training_buffer.append({
                'time': current_time,
                'global_path': new_path,
                'position': current_pos,
            })
        
        # Actualizar índice del path conforme se alcanzan waypoints
        self.update_path_index(current_pos)
        effective_path = self.current_global_path[self.current_path_index:] if self.current_global_path else None

        # Verificar que el effective_path sea seguro. Si no lo es, forzar replanificación.
        if effective_path is None or len(effective_path) < 2 or not self.check_effective_path_safety(effective_path, fused_grid, info):
            self.get_logger().warn("Effective path inseguro, forzando replanificación.")
            nodes = self.extract_nodes(fused_grid, info)
            new_path = self.plan_path(nodes, fused_grid, info)
            if new_path is None or len(new_path) < 3:
                self.get_logger().warn("Planificación convencional insuficiente. Forzando RRT.")
                new_path = self.rrt_plan_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
                if new_path is None or len(new_path) < 3:
                    self.get_logger().warn("RRT falló, usando fallback path incremental.")
                    new_path = self.fallback_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
                    if new_path is None or len(new_path) < 3:
                        self.get_logger().error("Fallback path también falló, abortando.")
                        return
            new_path = self.smooth_path(new_path)
            self.current_global_path = new_path
            self.current_path_index = 0
            effective_path = new_path  # Nuevo effective path

        self.publish_path(effective_path)

        # Calcular velocidades dinámicas
        q = self.odom.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        linear_speed, angular_speed = self.compute_dynamic_twist(current_pos, current_yaw, effective_path, fused_grid, info)
        
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)
        self.training_buffer.append({
            'time': current_time,
            'position': current_pos,
            'twist': {'linear': linear_speed, 'angular': angular_speed}
        })
        self.get_logger().info(f"Twist publicado: linear = {linear_speed:.2f} m/s, angular = {angular_speed:.2f} rad/s")

    def check_effective_path_safety(self, path, fused_grid, info):
        """
        Revisa cada segmento del path para asegurarse de que la línea de vista sea segura.
        Retorna True si todos los segmentos son seguros; en caso contrario, False.
        """
        for i in range(len(path) - 1):
            if not self.is_line_free(fused_grid, info, path[i], path[i+1]):
                self.get_logger().warn(f"Segmento inseguro entre {path[i]} y {path[i+1]}.")
                return False
        return True




    def should_replan(self, current_pos):
        # Aquí se puede definir una condición para replanificar globalmente.
        # En este ejemplo se retorna False para mantener el path actual.
        return False

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
