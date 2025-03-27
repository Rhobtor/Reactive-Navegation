#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import tensorflow as tf
import datetime

from geometry_msgs.msg import PoseArray, Pose, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from octomap_msgs.msg import Octomap

# Parámetros y constantes (ajusta según tu caso)
GOAL_REACHED_DIST = 3.0         # Distancia para considerar que se alcanzó la meta
OBSTACLE_PENALTY_DIST = 2.0       # Umbral de clearance para descartar candidatos

MAX_CANDIDATES = 30             # Número máximo de candidatos a considerar
FEATURE_DIM = 6                 # Dimensión del vector de características para cada candidato
GLOBAL_STATE_DIM = 6            # [robot_x, robot_y, goal_x, goal_y, avg_obs_dist, num_obs_norm]

# --- Actor Recurrente con LSTM para Inferencia ---
class RecurrentActorNetwork(tf.keras.Model):
    def __init__(self, max_candidates=MAX_CANDIDATES, feature_dim=FEATURE_DIM, lstm_units=64, **kwargs):
        super(RecurrentActorNetwork, self).__init__(**kwargs)
        self.max_candidates = max_candidates
        self.feature_dim = feature_dim
        # Procesa cada candidato de forma independiente:
        self.td1 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
        # Capa LSTM que integra la secuencia (memoria recurrente)
        self.lstm = tf.keras.layers.LSTM(lstm_units, return_sequences=True, return_state=True)
        # Capa para generar logits para cada candidato
        self.logits_layer = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
        
    def call(self, x, mask=None, initial_state=None):
        # x: (batch, MAX_CANDIDATES, FEATURE_DIM)
        x = self.td1(x)
        lstm_out, h, c = self.lstm(x, initial_state=initial_state)
        logits = self.logits_layer(lstm_out)
        logits = tf.squeeze(logits, axis=-1)  # (batch, MAX_CANDIDATES)
        if mask is not None:
            logits = tf.where(mask, logits, -1e9 * tf.ones_like(logits))
        return logits, (h, c)
    
    def get_config(self):
        config = super(RecurrentActorNetwork, self).get_config()
        config.update({
            "max_candidates": self.max_candidates,
            "feature_dim": self.feature_dim
        })
        return config

    @classmethod
    def from_config(cls, config):
        config.pop('trainable', None)
        config.pop('dtype', None)
        return cls(**config)

# Nodo de inferencia usando el actor recurrente (LSTM)
class NavigationPPOCandidateInference(Node):
    def __init__(self):
        super().__init__('navigation_ppo_candidate_inference')
        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        # Publicadores para RViz
        self.marker_pub = self.create_publisher(Marker, '/selected_candidate_marker', 10)
        self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
        
        # Variables de estado
        self.odom = None
        self.goal = None
        self.filtered_nodes = None
        self.occupied_nodes = None
        self.current_candidate = None

        # Cargar el modelo actor guardado en modo inferencia (modelo entrenado con LSTM)
        self.actor = tf.keras.models.load_model('actor_model_20250320_174242.keras', 
                                                  custom_objects={'RecurrentActorNetwork': RecurrentActorNetwork})
        self.get_logger().info("Modelo actor (LSTM) cargado para inferencia.")
        # Estado recurrente inicial (se mantiene entre pasos)
        self.actor_state = None
        
        # Timer para ejecutar step periódicamente (cada 0.1 s)
        self.timer = self.create_timer(0.1, self.step)

    # Callbacks
    def odom_callback(self, msg: Odometry):
        self.odom = msg.pose.pose

    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]

    def filtered_nodes_callback(self, msg: PoseArray):
        self.filtered_nodes = msg

    def occupied_nodes_callback(self, msg: PoseArray):
        self.occupied_nodes = msg

    # Función para calcular estadísticas de obstáculos
    def compute_obstacle_stats(self):
        if self.occupied_nodes and self.occupied_nodes.poses:
            distances = [math.hypot(self.odom.position.x - obs.position.x,
                                    self.odom.position.y - obs.position.y)
                         for obs in self.occupied_nodes.poses]
            avg_dist = np.mean(distances)
            num_obs = len(distances)
        else:
            avg_dist = 10.0  # Valor por defecto
            num_obs = 0
        num_obs_norm = num_obs / 10.0
        return avg_dist, num_obs_norm

    # Extrae características para cada candidato (igual que en entrenamiento)
    def compute_candidate_features(self):
        features = []
        valid_nodes = []
        if self.filtered_nodes is None or not self.filtered_nodes.poses:
            return None, None, None
        robot_x = self.odom.position.x
        robot_y = self.odom.position.y
        goal_x = self.goal.position.x
        goal_y = self.goal.position.y
        current_yaw = 0.0  # Ajusta si se dispone de orientación real

        # Calcular estadísticas globales de obstáculos:
        avg_obs_dist, num_obs_norm = self.compute_obstacle_stats()

        for node in self.filtered_nodes.poses:
            if self.occupied_nodes and self.occupied_nodes.poses:
                clearance = min([math.hypot(node.position.x - occ.position.x,
                                            node.position.y - occ.position.y)
                                 for occ in self.occupied_nodes.poses])
            else:
                clearance = 10.0
            if clearance < OBSTACLE_PENALTY_DIST:
                continue
            dx = node.position.x - robot_x
            dy = node.position.y - robot_y
            dist_robot = math.hypot(dx, dy)
            angle_to_node = math.atan2(dy, dx)
            angle_diff = abs(angle_to_node - current_yaw)
            dist_to_goal = math.hypot(node.position.x - goal_x, node.position.y - goal_y)
            # Vector de características con 6 elementos:
            feature_vector = [dist_robot, angle_diff, clearance, dist_to_goal, avg_obs_dist, num_obs_norm]
            features.append(feature_vector)
            valid_nodes.append(node)
        if len(features) == 0:
            return None, None, None
        # Ordenar candidatos por distancia a la meta (usando la característica dist_to_goal)
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

    # Función step: utiliza el modelo recurrente para seleccionar el candidato y lo publica
    def step(self):
        if self.odom is None or self.goal is None:
            return

        # Si ya se ha seleccionado un candidato y se ha alcanzado, se reinicia la selección
        if self.current_candidate is not None:
            dist_to_candidate = math.hypot(
                self.odom.position.x - self.current_candidate.position.x,
                self.odom.position.y - self.current_candidate.position.y)
            if dist_to_candidate < 2.5:
                self.get_logger().info("Candidato alcanzado. Seleccionando nuevo candidato.")
                self.current_candidate = None
                # Opcional: reiniciar el estado recurrente si se quiere empezar fresco
                # self.actor_state = None

        # Seleccionar candidato si no hay uno vigente
        if self.current_candidate is None:
            candidate_features, valid_nodes, mask = self.compute_candidate_features()
            if candidate_features is None:
                self.get_logger().warn("No hay candidatos válidos.")
                return
            actor_input = np.expand_dims(candidate_features, axis=0)  # (1, MAX_CANDIDATES, FEATURE_DIM)
            mask_input = np.expand_dims(mask, axis=0)                  # (1, MAX_CANDIDATES)
            # Llamar al actor recurrente, pasando el estado actual
            logits, self.actor_state = self.actor(actor_input, mask=tf.convert_to_tensor(mask_input), initial_state=self.actor_state)
            probs = tf.nn.softmax(logits, axis=-1).numpy()[0]
            action_index = int(np.argmax(probs))
            if not mask[action_index]:
                self.get_logger().warn("Candidato seleccionado inválido.")
                return
            self.current_candidate = valid_nodes[action_index]
            self.get_logger().info(
                f"Nodo candidato seleccionado: ({self.current_candidate.position.x:.2f}, {self.current_candidate.position.y:.2f})"
            )
        # Publicar el candidato en RViz
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nodo_candidato"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = self.current_candidate
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.marker_pub.publish(marker)

        nav_points = PoseArray()
        nav_points.header.stamp = self.get_clock().now().to_msg()
        nav_points.header.frame_id = "map"
        nav_points.poses.append(self.current_candidate)
        self.nav_point.publish(nav_points)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationPPOCandidateInference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
