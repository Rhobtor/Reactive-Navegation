#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import tensorflow as tf
import tensorflow_probability as tfp

from geometry_msgs.msg import PoseArray, Pose, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from octomap_msgs.msg import Octomap

# PPO hiperparámetros y parámetros de navegación
GOAL_REACHED_DIST = 3.0       # Distancia para considerar que se alcanzó la meta
STEP_PENALTY = -0.05          # Penalización por paso
GOAL_REWARD = 50.0            # Recompensa al alcanzar la meta
OBSTACLE_PENALTY_DIST = 4.0     # Umbral de clearance para descartar candidatos

MAX_CANDIDATES = 5            # Número máximo de candidatos a considerar
FEATURE_DIM = 4               # Dimensión del vector de características para cada candidato
GLOBAL_STATE_DIM = 4          # [robot_x, robot_y, goal_x, goal_y]

GAMMA = 0.99
LAMBDA = 0.95
CLIP_EPS = 0.2
TRAIN_EPOCHS = 10
BATCH_SIZE = 32



# Red Actor para evaluar candidatos (output: logits para cada candidato)
class ActorNetwork(tf.keras.Model):
    def __init__(self, max_candidates, feature_dim):
        super(ActorNetwork, self).__init__()
        self.max_candidates = max_candidates
        self.feature_dim = feature_dim
        self.dense1 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
        self.dense2 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(32, activation='relu'))
        self.logits_layer = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
    def call(self, x, mask=None):
        # x: (batch, max_candidates, feature_dim)
        x = self.dense1(x)
        x = self.dense2(x)
        logits = self.logits_layer(x)  # (batch, max_candidates, 1)
        logits = tf.squeeze(logits, axis=-1)  # (batch, max_candidates)
        if mask is not None:
            logits = tf.where(mask, logits, -1e9 * tf.ones_like(logits))
        return logits

# Red Crítica para estimar el valor del estado (global)
class CriticNetwork(tf.keras.Model):
    def __init__(self, input_dim):
        super(CriticNetwork, self).__init__()
        self.dense1 = tf.keras.layers.Dense(64, activation='relu')
        self.dense2 = tf.keras.layers.Dense(32, activation='relu')
        self.value = tf.keras.layers.Dense(1)
    def call(self, x):
        x = self.dense1(x)
        x = self.dense2(x)
        v = self.value(x)
        return tf.squeeze(v, axis=-1)

# Nodo que entrena PPO para seleccionar el mejor nodo candidato y lo publica en RViz
class NavigationPPOCandidateTrainer(Node):
    def __init__(self):
        super().__init__('navigation_ppo_candidate_trainer')
        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        # Publicador para visualizar el nodo seleccionado
        self.marker_pub = self.create_publisher(Marker, '/selected_candidate_marker', 10)
        # Variables de estado
        self.odom = None
        self.goal = None
        self.filtered_nodes = None
        self.occupied_nodes = None
        self.current_candidate = None
        # Buffers PPO (se usan para el entrenamiento, si lo deseas)
        self.states = []
        self.actor_inputs = []
        self.actions = []
        self.log_probs = []
        self.rewards = []
        self.values = []
        self.dones = []
        self.steps = 0
        self.max_steps = 1000
        # Redes y optimizadores (si se entrena en línea)
        self.actor = ActorNetwork(MAX_CANDIDATES, FEATURE_DIM)
        self.critic = CriticNetwork(GLOBAL_STATE_DIM)
        self.actor_optimizer = tf.keras.optimizers.Adam(3e-4)
        self.critic_optimizer = tf.keras.optimizers.Adam(3e-4)
        self.get_logger().info("Navigation PPO Candidate Trainer iniciado.")
        while self.odom is None or self.goal is None:
            self.get_logger().warn("Esperando /odom y /goal...")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Datos iniciales recibidos, iniciando selección de nodo.")
        self.timer = self.create_timer(0.1, self.step)

    # Callbacks de suscripción
    def odom_callback(self, msg: Odometry):
        self.odom = msg.pose.pose
    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]
    def filtered_nodes_callback(self, msg: PoseArray):
        self.filtered_nodes = msg
    def occupied_nodes_callback(self, msg: PoseArray):
        self.occupied_nodes = msg
## manual
    # # Extrae características para cada nodo candidato:
    # # f1: distancia del robot al candidato  
    # # f2: diferencia angular (absoluta) entre la dirección del candidato y la orientación del robot  
    # # f3: clearance (distancia al obstáculo más cercano)  
    # # f4: distancia del candidato a la meta
    # def compute_candidate_features(self):
    #     features = []
    #     valid_nodes = []
    #     if self.filtered_nodes is None or not self.filtered_nodes.poses:
    #         return None, None, None
    #     robot_x = self.odom.position.x
    #     robot_y = self.odom.position.y
    #     goal_x = self.goal.position.x
    #     goal_y = self.goal.position.y
    #     # Se puede usar la orientación real si se dispone; aquí se asume 0 para simplificar.
    #     current_yaw = 0.0  
    #     for node in self.filtered_nodes.poses:
    #         if self.occupied_nodes and self.occupied_nodes.poses:
    #             clearance = min([math.hypot(node.position.x - occ.position.x,
    #                                         node.position.y - occ.position.y)
    #                              for occ in self.occupied_nodes.poses])
    #         else:
    #             clearance = 10.0
    #         if clearance < OBSTACLE_PENALTY_DIST:
    #             continue
    #         dx = node.position.x - robot_x
    #         dy = node.position.y - robot_y
    #         dist_robot = math.hypot(dx, dy)
    #         angle_to_node = math.atan2(dy, dx)
    #         angle_diff = abs(angle_to_node - current_yaw)
    #         dist_to_goal = math.hypot(node.position.x - goal_x, node.position.y - goal_y)
    #         features.append([dist_robot, angle_diff, clearance, dist_to_goal])
    #         valid_nodes.append(node)
    #     if len(features) == 0:
    #         return None, None, None
    #     # Opcionalmente se ordena por distancia a la meta
    #     features, valid_nodes = zip(*sorted(zip(features, valid_nodes), key=lambda x: x[0][3]))
    #     features = list(features)
    #     valid_nodes = list(valid_nodes)
    #     if len(features) > MAX_CANDIDATES:
    #         features = features[:MAX_CANDIDATES]
    #         valid_nodes = valid_nodes[:MAX_CANDIDATES]
    #     num_valid = len(features)
    #     while len(features) < MAX_CANDIDATES:
    #         features.append([0.0]*FEATURE_DIM)
    #     features = np.array(features, dtype=np.float32)  # (MAX_CANDIDATES, FEATURE_DIM)
    #     mask = np.array([True]*num_valid + [False]*(MAX_CANDIDATES - num_valid))
    #     return features, valid_nodes, mask

# intento de implementar la función compute_candidate_features
    def compute_candidate_features(self):
        features = []
        valid_nodes = []
        scores = []
        if self.filtered_nodes is None or not self.filtered_nodes.poses:
            return None, None, None
        robot_x = self.odom.position.x
        robot_y = self.odom.position.y
        goal_x = self.goal.position.x
        goal_y = self.goal.position.y
        current_yaw = 0.0  # Puedes usar la orientación real si la tienes
        alpha = 1.0      # Factor de penalización ajustable

        for node in self.filtered_nodes.poses:
            # Calcular el clearance: distancia mínima a los obstáculos
            if self.occupied_nodes and self.occupied_nodes.poses:
                clearance = min([math.hypot(node.position.x - occ.position.x,
                                            node.position.y - occ.position.y)
                                for occ in self.occupied_nodes.poses])
            else:
                clearance = 10.0
            # Calcular características
            dx = node.position.x - robot_x
            dy = node.position.y - robot_y
            dist_robot = math.hypot(dx, dy)
            angle_to_node = math.atan2(dy, dx)
            angle_diff = abs(angle_to_node - current_yaw)
            dist_to_goal = math.hypot(node.position.x - goal_x, node.position.y - goal_y)
            features.append([dist_robot, angle_diff, clearance, dist_to_goal])
            valid_nodes.append(node)
            # Calcular el score: se penaliza si clearance es menor que OBSTACLE_PENALTY_DIST
            score = dist_to_goal + alpha * max(0, OBSTACLE_PENALTY_DIST - clearance)
            scores.append(score)
            
        if len(features) == 0:
            return None, None, None
        # Ordenar candidatos por score (menor es mejor)
        combined = list(zip(features, valid_nodes, scores))
        combined.sort(key=lambda x: x[2])
        features, valid_nodes, scores = zip(*combined)
        features = list(features)
        valid_nodes = list(valid_nodes)
        num_valid = len(features)
        # Rellenar hasta MAX_CANDIDATES para tener un tamaño fijo
        while len(features) < MAX_CANDIDATES:
            features.append([0.0]*FEATURE_DIM)
        features = np.array(features, dtype=np.float32)  # (MAX_CANDIDATES, FEATURE_DIM)
        mask = np.array([True]*num_valid + [False]*(MAX_CANDIDATES - num_valid))
        return features, valid_nodes, mask



    # Estado global para el crítico: [robot_x, robot_y, goal_x, goal_y]
    def get_global_state(self):
        return np.array([
            self.odom.position.x,
            self.odom.position.y,
            self.goal.position.x,
            self.goal.position.y
        ], dtype=np.float32)

    # Función de recompensa (opcional si se entrena)
    def compute_reward(self):
        dist = math.hypot(self.odom.position.x - self.goal.position.x,
                          self.odom.position.y - self.goal.position.y)
        reward = -dist * 0.1 + STEP_PENALTY
        if dist < GOAL_REACHED_DIST:
            reward += GOAL_REWARD
        return reward

    # Función step: selecciona el mejor nodo candidato y publica su posición en RViz
    def step(self):
        if self.odom is None or self.goal is None:
            return

        # Si ya hay un candidato seleccionado, verificar si ya se alcanzó
        if self.current_candidate is not None:
            dist_to_candidate = math.hypot(
                self.odom.position.x - self.current_candidate.position.x,
                self.odom.position.y - self.current_candidate.position.y)
            # Si estamos cerca del candidato (por ejemplo, menos de 0.5 metros), lo consideramos alcanzado
            if dist_to_candidate < 2.5:
                self.get_logger().info("Candidato alcanzado. Se seleccionará uno nuevo.")
                self.current_candidate = None

        # Si no hay candidato vigente, calcularlo y almacenar el índice
        if self.current_candidate is None:
            candidate_features, valid_nodes, mask = self.compute_candidate_features()
            if candidate_features is None:
                self.get_logger().warn("No hay candidatos válidos.")
                return
            # Preparar la entrada para la red actor
            actor_input = np.expand_dims(candidate_features, axis=0)  # (1, MAX_CANDIDATES, FEATURE_DIM)
            mask_input = np.expand_dims(mask, axis=0)  # (1, MAX_CANDIDATES)
            logits = self.actor(actor_input, mask=tf.convert_to_tensor(mask_input))
            probs = tf.nn.softmax(logits, axis=-1).numpy()[0]  # (MAX_CANDIDATES,)
            # Seleccionar el índice con mayor probabilidad
            action_index = int(np.argmax(probs))
            if not mask[action_index]:
                self.get_logger().warn("Candidato seleccionado inválido, saltando paso.")
                return
            self.current_candidate = valid_nodes[action_index]
            self.last_action_index = action_index  # Guardamos el índice calculado
            self.last_probs = probs # Guardamos las probabilidades para el cálculo de log_probs
            self.get_logger().info(
                f"Nodo candidato seleccionado: ({self.current_candidate.position.x:.2f}, {self.current_candidate.position.y:.2f})"
            )
        else:
            # Si ya hay candidato, usar el último índice calculado
            action_index = self.last_action_index
            probs = self.last_probs

        # Publicar marker en RViz para visualizar el candidato actual
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

        # Guardar experiencia (para entrenamiento PPO, si lo deseas)
        global_state = self.get_global_state()
        value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
        reward = self.compute_reward()
        done = 1 if math.hypot(self.odom.position.x - self.goal.position.x,
                                self.odom.position.y - self.goal.position.y) < GOAL_REACHED_DIST else 0

        self.states.append(global_state)
        # Guardamos las features y la máscara (usando la versión actual de compute_candidate_features)
        self.actor_inputs.append(self.compute_candidate_features()[:2])
        self.actions.append(action_index)
        self.log_probs.append(np.log(probs[action_index] + 1e-8))
        self.values.append(value)
        self.rewards.append(reward)
        self.dones.append(done)
        self.steps += 1

        if done or self.steps >= self.max_steps:
            self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
            self.update_model()  # Entrenamiento PPO
            # Reiniciar buffers y candidato para el próximo episodio
            self.states = []
            self.actor_inputs = []
            self.actions = []
            self.log_probs = []
            self.values = []
            self.rewards = []
            self.dones = []
            self.steps = 0
            self.current_candidate = None


    # Computar ventajas y retornos para PPO (si se entrena)
    def compute_advantages(self, rewards, values, dones):
        advantages = np.zeros_like(rewards, dtype=np.float32)
        last_adv = 0.0
        for t in reversed(range(len(rewards))):
            next_value = values[t+1] if t+1 < len(values) else 0.0
            delta = rewards[t] + GAMMA * next_value * (1 - dones[t]) - values[t]
            advantages[t] = last_adv = delta + GAMMA * LAMBDA * (1 - dones[t]) * last_adv
        returns = advantages + np.array(values, dtype=np.float32)
        return advantages, returns

    # # Actualizar redes usando PPO (opcional, si se entrena en línea)
    # def update_model(self):
    #     states = np.array(self.states, dtype=np.float32)
    #     actions = np.array(self.actions, dtype=np.int32)
    #     log_probs_old = np.array(self.log_probs, dtype=np.float32)
    #     rewards = np.array(self.rewards, dtype=np.float32)
    #     dones = np.array(self.dones, dtype=np.float32)
    #     advantages, returns = self.compute_advantages(rewards, self.values, dones)
    #     advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
    #     N = len(states)
    #     actor_inputs = []
    #     masks = []
    #     for cand_feat, m in self.actor_inputs:
    #         actor_inputs.append(cand_feat)
    #         masks.append(m)
    #     actor_inputs = np.array(actor_inputs, dtype=np.float32)  # (N, MAX_CANDIDATES, FEATURE_DIM)
    #     masks = np.array(masks, dtype=bool)  # (N, MAX_CANDIDATES)
    #     dataset = tf.data.Dataset.from_tensor_slices((states, actor_inputs, masks, actions, log_probs_old, returns, advantages))
    #     dataset = dataset.shuffle(N).batch(BATCH_SIZE)
    #     for epoch in range(TRAIN_EPOCHS):
    #         for batch in dataset:
    #             batch_states, batch_actor_inputs, batch_masks, batch_actions, batch_old_log_probs, batch_returns, batch_advantages = batch
    #             with tf.GradientTape(persistent=True) as tape:
    #                 logits = self.actor(batch_actor_inputs, mask=batch_masks)
    #                 batch_actions = tf.cast(batch_actions, tf.int32)
    #                 indices = tf.stack([tf.range(tf.shape(logits)[0]), batch_actions], axis=1)
    #                 chosen_logits = tf.gather_nd(logits, indices)
    #                 new_log_probs = tf.math.log(tf.nn.softmax(logits, axis=1) + 1e-8)
    #                 new_log_probs = tf.gather_nd(new_log_probs, indices)
    #                 ratio = tf.exp(new_log_probs - batch_old_log_probs)
    #                 clipped_ratio = tf.clip_by_value(ratio, 1 - CLIP_EPS, 1 + CLIP_EPS)
    #                 actor_loss = -tf.reduce_mean(tf.minimum(ratio * batch_advantages, clipped_ratio * batch_advantages))
    #                 values_pred = self.critic(batch_states)
    #                 critic_loss = tf.reduce_mean(tf.square(batch_returns - values_pred))
    #                 total_loss = actor_loss + 0.5 * critic_loss
    #             actor_grads = tape.gradient(total_loss, self.actor.trainable_variables)
    #             critic_grads = tape.gradient(total_loss, self.critic.trainable_variables)
    #             self.actor_optimizer.apply_gradients(zip(actor_grads, self.actor.trainable_variables))
    #             self.critic_optimizer.apply_gradients(zip(critic_grads, self.critic.trainable_variables))
    #     self.get_logger().info("Modelo PPO actualizado.")

    def update_model(self):
        states = np.array(self.states, dtype=np.float32)
        actions = np.array(self.actions, dtype=np.int32)
        log_probs_old = np.array(self.log_probs, dtype=np.float32)
        rewards = np.array(self.rewards, dtype=np.float32)
        dones = np.array(self.dones, dtype=np.float32)
        advantages, returns = self.compute_advantages(rewards, self.values, dones)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        N = len(states)

        # Separar las features y las máscaras de los actor_inputs
        actor_inputs_feats = []
        actor_inputs_masks = []
        for cand_feat, m in self.actor_inputs:
            actor_inputs_feats.append(cand_feat)
            actor_inputs_masks.append(m)
        actor_inputs_feats = np.array(actor_inputs_feats, dtype=np.float32)  # (N, MAX_CANDIDATES, FEATURE_DIM)
        actor_inputs_masks = np.array(actor_inputs_masks, dtype=bool)         # (N, MAX_CANDIDATES)

        dataset = tf.data.Dataset.from_tensor_slices(
            (states, actor_inputs_feats, actor_inputs_masks, actions, log_probs_old, returns, advantages)
        )
        dataset = dataset.shuffle(N).batch(BATCH_SIZE)

        for epoch in range(TRAIN_EPOCHS):
            for batch in dataset:
                batch_states, batch_actor_inputs, batch_masks, batch_actions, batch_old_log_probs, batch_returns, batch_advantages = batch
                with tf.GradientTape(persistent=True) as tape:
                    logits = self.actor(batch_actor_inputs, mask=batch_masks)
                    batch_actions = tf.cast(batch_actions, tf.int32)
                    indices = tf.stack([tf.range(tf.shape(logits)[0]), batch_actions], axis=1)
                    chosen_logits = tf.gather_nd(logits, indices)
                    new_log_probs = tf.math.log(tf.nn.softmax(logits, axis=1) + 1e-8)
                    new_log_probs = tf.gather_nd(new_log_probs, indices)
                    ratio = tf.exp(new_log_probs - batch_old_log_probs)
                    clipped_ratio = tf.clip_by_value(ratio, 1 - CLIP_EPS, 1 + CLIP_EPS)
                    actor_loss = -tf.reduce_mean(tf.minimum(ratio * batch_advantages,
                                                            clipped_ratio * batch_advantages))
                    values_pred = self.critic(batch_states)
                    critic_loss = tf.reduce_mean(tf.square(batch_returns - values_pred))
                    total_loss = actor_loss + 0.5 * critic_loss
                actor_grads = tape.gradient(total_loss, self.actor.trainable_variables)
                critic_grads = tape.gradient(total_loss, self.critic.trainable_variables)
                self.actor_optimizer.apply_gradients(zip(actor_grads, self.actor.trainable_variables))
                self.critic_optimizer.apply_gradients(zip(critic_grads, self.critic.trainable_variables))
        self.get_logger().info("Modelo PPO actualizado.")



def main(args=None):
    rclpy.init(args=args)
    node = NavigationPPOCandidateTrainer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
