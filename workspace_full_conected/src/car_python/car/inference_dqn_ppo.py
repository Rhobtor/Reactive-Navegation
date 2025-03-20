#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import tensorflow as tf
import datetime
import time

from std_srvs.srv import Empty
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from octomap_msgs.msg import Octomap
from gazebo_msgs.msg import ModelState

# --------------------- UTILIDADES ---------------------
def quaternion_to_yaw(q: Quaternion):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# --------------------- CONSTANTES ---------------------
GOAL_REACHED_DIST = 3.0       # Umbral para considerar que se ha alcanzado el goal
MAX_CANDIDATES = 5            
FEATURE_DIM = 6               
GLOBAL_STATE_DIM = 7          
MAX_FRONTIER_POINTS = 30

# --------------------- MODELOS PPO ---------------------
class RecurrentActorNetwork(tf.keras.Model):
    def __init__(self, max_candidates, feature_dim, lstm_units=64, **kwargs):
        super(RecurrentActorNetwork, self).__init__(**kwargs)
        self.max_candidates = max_candidates
        self.feature_dim = feature_dim
        self.td1 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
        self.lstm = tf.keras.layers.LSTM(lstm_units, return_sequences=True, return_state=True)
        self.logits_layer = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
        
    def call(self, x, mask=None, initial_state=None):
        x = self.td1(x)
        lstm_out, h, c = self.lstm(x, initial_state=initial_state)
        logits = self.logits_layer(lstm_out)
        logits = tf.squeeze(logits, axis=-1)
        if mask is not None:
            logits = tf.where(mask, logits, -1e9 * tf.ones_like(logits))
        return logits, (h, c)

class CriticNetwork(tf.keras.Model):
    def __init__(self, input_dim):
        super(CriticNetwork, self).__init__()
        self.dense1 = tf.keras.layers.Dense(64, activation='relu')
        self.dense2 = tf.keras.layers.Dense(64, activation='relu')
        self.dense3 = tf.keras.layers.Dense(64, activation='relu')
        self.value = tf.keras.layers.Dense(1)
    def call(self, x):
        x = self.dense1(x)
        x = self.dense2(x)
        x = self.dense3(x)
        v = self.value(x)
        return tf.squeeze(v, axis=-1)

# --------------------- MODELO DQN ---------------------
class DQN(tf.keras.Model):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = tf.keras.layers.Dense(64, activation='relu')
        self.fc2 = tf.keras.layers.Dense(64, activation='relu')
        self.fc3 = tf.keras.layers.Dense(action_size)
    def call(self, x):
        x = self.fc1(x)
        x = self.fc2(x)
        return self.fc3(x)

# --------------------- NODO DE INFERENCIA COMBINADA ---------------------
# Estado para DQN: [odom_x, odom_y, total_entropy] + (2 * MAX_CANDIDATES) coordenadas
STATE_SIZE_DQN = 3 + MAX_CANDIDATES * 2
ACTION_SIZE = MAX_CANDIDATES

class NavigationInferenceTrainer(Node):
    def __init__(self):
        super().__init__('navigation_inference_trainer')
        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        self.create_subscription(Bool, '/virtual_collision', self.collision_callback, 10)
        self.create_subscription(Float64, '/total_entropy', self.entropy_callback, 10)
        # Publicadores
        self.goal_pub = self.create_publisher(PoseArray, '/goal', 10)
        self.marker_pub = self.create_publisher(Marker, '/selected_candidate_marker', 10)
        self.nav_point_pub = self.create_publisher(PoseArray, '/nav_point', 10)

        # Variables del entorno
        self.odom = None
        self.goal = None
        self.filtered_nodes = None
        self.occupied_nodes = None
        self.virtual_collision = False
        self.total_entropy = 0.0

        # Variables para la selección del candidato (PPO)
        self.current_candidate = None
        self.last_candidate = None
        self.last_candidate_features = None
        self.last_action_index = None

        # Instanciar modelos para inferencia
        self.actor = RecurrentActorNetwork(MAX_CANDIDATES, FEATURE_DIM)
        self.critic = CriticNetwork(GLOBAL_STATE_DIM)
        self.dqn_model = DQN(STATE_SIZE_DQN, ACTION_SIZE)
        
        # Construir los modelos con entradas dummy para inicializarlos
        dummy_actor_input = np.zeros((1, MAX_CANDIDATES, FEATURE_DIM), dtype=np.float32)
        dummy_mask = np.ones((1, MAX_CANDIDATES), dtype=bool)
        _ = self.actor(dummy_actor_input, mask=tf.convert_to_tensor(dummy_mask))
        dummy_critic_input = np.zeros((1, GLOBAL_STATE_DIM), dtype=np.float32)
        _ = self.critic(dummy_critic_input)
        dummy_dqn_input = np.zeros((1, STATE_SIZE_DQN), dtype=np.float32)
        _ = self.dqn_model(dummy_dqn_input)
        
        # Cargar modelos guardados (ajusta las rutas según corresponda)
        self.actor.load_weights("actor_model_20250318_210021.h5")
        self.critic.load_weights("critic_model_20250318_210021.h5")
        self.dqn_model.load_weights("dqn_model_20250318_210021.h5")
        self.get_logger().info("Modelos cargados para inferencia.")

        # Esperar datos iniciales
        while self.odom is None or self.goal is None:
            self.get_logger().warn("Esperando /odom y /goal...")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Datos iniciales recibidos. Iniciando inferencia.")
        self.timer = self.create_timer(0.1, self.step)

    # ----- Callbacks -----
    def odom_callback(self, msg: Odometry):
        original_pose = msg.pose.pose
        offset_x = 0.5
        adjusted_pose = Pose()
        adjusted_pose.position.x = original_pose.position.x - offset_x
        adjusted_pose.position.y = original_pose.position.y
        adjusted_pose.position.z = original_pose.position.z
        adjusted_pose.orientation = original_pose.orientation
        self.odom = adjusted_pose

    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]

    def filtered_nodes_callback(self, msg: PoseArray):
        self.filtered_nodes = msg

    def occupied_nodes_callback(self, msg: PoseArray):
        self.occupied_nodes = msg

    def collision_callback(self, msg: Bool):
        self.virtual_collision = msg.data

    def entropy_callback(self, msg: Float64):
        self.total_entropy = msg.data

    # ----- Funciones comunes -----
    def compute_obstacle_stats(self):
        if self.occupied_nodes and self.occupied_nodes.poses:
            distances = [math.hypot(self.odom.position.x - occ.position.x,
                                    self.odom.position.y - occ.position.y)
                         for occ in self.occupied_nodes.poses]
            avg_dist = np.mean(distances)
            num_obs = len(distances)
        else:
            avg_dist = 10.0
            num_obs = 0
        num_obs_norm = num_obs / 10.0
        return avg_dist, num_obs_norm

    def compute_candidate_features(self):
        features = []
        valid_nodes = []
        if self.filtered_nodes is None or not self.filtered_nodes.poses:
            return None, None, None
        robot_x = self.odom.position.x
        robot_y = self.odom.position.y
        goal_x = self.goal.position.x
        goal_y = self.goal.position.y
        avg_obs_dist, num_obs_norm = self.compute_obstacle_stats()
        for node in self.filtered_nodes.poses:
            if self.occupied_nodes and self.occupied_nodes.poses:
                clearance = min([math.hypot(node.position.x - occ.position.x,
                                            node.position.y - occ.position.y)
                                 for occ in self.occupied_nodes.poses])
            else:
                clearance = 10.0
            if clearance < 0.5:
                continue
            dx = node.position.x - robot_x
            dy = node.position.y - robot_y
            dist_robot = math.hypot(dx, dy)
            angle_to_node = math.atan2(dy, dx)
            angle_diff = abs(angle_to_node)
            dist_to_goal = math.hypot(node.position.x - goal_x, node.position.y - goal_y)
            feature_vector = [dist_robot, angle_diff, clearance, dist_to_goal, avg_obs_dist, num_obs_norm]
            features.append(feature_vector)
            valid_nodes.append(node)
        if len(features) == 0:
            return None, None, None
        features, valid_nodes = zip(*sorted(zip(features, valid_nodes), key=lambda x: x[0][3]))
        features = list(features)
        valid_nodes = list(valid_nodes)
        if len(features) > MAX_CANDIDATES:
            features = features[:MAX_CANDIDATES]
            valid_nodes = valid_nodes[:MAX_CANDIDATES]
        num_valid = len(features)
        while len(features) < MAX_CANDIDATES:
            features.append([0.0]*FEATURE_DIM)
        features = np.array(features, dtype=np.float32)
        mask = np.array([True]*num_valid + [False]*(MAX_CANDIDATES - num_valid))
        return features, valid_nodes, mask

    def get_global_state(self):
        avg_obs_dist, num_obs_norm = self.compute_obstacle_stats()
        return np.array([
            self.odom.position.x,
            self.odom.position.y,
            self.goal.position.x,
            self.goal.position.y,
            avg_obs_dist,
            num_obs_norm,
            0.0
        ], dtype=np.float32)

    def get_dqn_state(self):
        # Estado para DQN: [odom_x, odom_y, total_entropy] + 2 coords por candidato
        state = [self.odom.position.x, self.odom.position.y, self.total_entropy]
        features, valid_nodes, _ = self.compute_candidate_features()
        if valid_nodes is None:
            state += [0.0] * (MAX_CANDIDATES * 2)
        else:
            for node in valid_nodes[:MAX_CANDIDATES]:
                state.extend([node.position.x, node.position.y])
            state += [0.0] * ((MAX_CANDIDATES - len(valid_nodes)) * 2)
        return np.array(state, dtype=np.float32)

    def publish_candidate_marker(self, candidate: Pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "candidate_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = candidate
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.b = 1.0
        self.marker_pub.publish(marker)

    def publish_nav_point(self, candidate: Pose):
        nav_points = PoseArray()
        nav_points.header.stamp = self.get_clock().now().to_msg()
        nav_points.header.frame_id = "map"
        nav_points.poses.append(candidate)
        self.nav_point_pub.publish(nav_points)

    # ----- Ciclo de inferencia (step) -----
    def step(self):
        if self.odom is None or self.goal is None:
            return

        # Si ya se ha seleccionado un candidato, verificar si se ha alcanzado
        if self.current_candidate is not None:
            dist_to_candidate = math.hypot(self.odom.position.x - self.current_candidate.position.x,
                                           self.odom.position.y - self.current_candidate.position.y)
            if dist_to_candidate > GOAL_REACHED_DIST:
                # Si aún no se alcanza, mantener la meta actual
                self.get_logger().info(f"Manteniendo meta actual; distancia = {dist_to_candidate:.2f}")
                candidate = self.current_candidate
            else:
                self.get_logger().info("Meta alcanzada. Seleccionando nuevo candidato.")
                self.current_candidate = None
        else:
            candidate_features, valid_nodes, mask = self.compute_candidate_features()
            if candidate_features is None:
                self.get_logger().warn("No hay candidatos válidos para inferir.")
                return
            # Inferencia con PPO
            actor_input = np.expand_dims(candidate_features, axis=0)
            mask_input = np.expand_dims(mask, axis=0)
            logits, _ = self.actor(actor_input, mask=tf.convert_to_tensor(mask_input), initial_state=None)
            probs = tf.nn.softmax(logits, axis=-1).numpy()[0]
            action_index_ppo = int(np.argmax(probs))
            if not mask[action_index_ppo]:
                self.get_logger().warn("Candidato PPO inválido.")
                return
            candidate_ppo = valid_nodes[action_index_ppo]

            # Inferencia con DQN
            dqn_state = self.get_dqn_state()
            dqn_out = self.dqn_model(np.array([dqn_state], dtype=np.float32))
            action_index_dqn = int(np.argmax(dqn_out.numpy()[0]))

            # Estrategia de fusión: en este ejemplo se opta por usar PPO
            chosen_index = action_index_ppo
            self.last_action_index = chosen_index
            self.get_logger().info(f"Acción PPO: {action_index_ppo} | Acción DQN: {action_index_dqn} => Usando: {chosen_index}")
            self.current_candidate = candidate_ppo
            self.last_candidate = candidate_ppo
            self.last_candidate_features = (candidate_features, mask, chosen_index, probs)
            candidate = self.current_candidate

        # Publicar marcador y punto de navegación
        self.publish_candidate_marker(candidate)
        self.publish_nav_point(candidate)

        # Publicar la meta (goal)
        goal_msg = PoseArray()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.poses.append(candidate)
        self.goal_pub.publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationInferenceTrainer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
