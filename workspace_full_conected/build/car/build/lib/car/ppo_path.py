#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import tensorflow as tf
import datetime
import time
import heapq
import os
import signal
import random

from std_srvs.srv import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from octomap_msgs.msg import Octomap

# ----------------------- Constantes y Hiperparámetros -----------------------
GOAL_REACHED_DIST = 3.0       
STEP_PENALTY = -0.05          
GOAL_REWARD = 50.0            

# Parámetros de entrenamiento y del entorno
FEATURE_DIM = 6               
GLOBAL_STATE_DIM = 7  
MAX_SEQ_LENGTH = 5  # Número fijo de waypoints que genera el path planner

GAMMA = 0.99
LAMBDA = 0.95
CLIP_EPS = 0.2
TRAIN_EPOCHS = 10
BATCH_SIZE = 32

MEMORY_WINDOW_SIZE = 200  # Máximo número de experiencias a retener

# Parámetros para la máquina de estados
TIMEOUT_THRESHOLD = 10.0   # Timeout en estado MOVING
WAYPOINT_THRESHOLD = 2.5   # Distancia para considerar alcanzado un waypoint

# Parámetros para evitar trampas
SAME_CANDIDATE_THRESHOLD = 3

# ----------------------- Funciones Auxiliares -----------------------
def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def quaternion_to_yaw(q: Quaternion):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# ----------------------- Red de Path Planner End-to-End -----------------------
class RecurrentPathPlanner(tf.keras.Model):
    def __init__(self, seq_length=MAX_SEQ_LENGTH, lstm_units=64, **kwargs):
        super(RecurrentPathPlanner, self).__init__(**kwargs)
        self.seq_length = seq_length
        # Una capa para procesar el estado global
        self.dense_context = tf.keras.layers.Dense(64, activation='relu')
        # LSTM para generar la secuencia
        self.lstm = tf.keras.layers.LSTM(lstm_units, return_sequences=True)
        # Capa de salida que genera dos valores (x, y) por cada paso
        self.out_layer = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(2))
        
    def call(self, x):
        # x: estado global con forma (batch, GLOBAL_STATE_DIM)
        context = self.dense_context(x)  # (batch, 64)
        # Usamos el mismo vector para estado oculto y celda
        initial_state = [context, context]
        # Creamos una secuencia inicial de entrada para el decodificador (por ejemplo, ceros)
        batch_size = tf.shape(x)[0]
        decoder_input = tf.zeros((batch_size, self.seq_length, 64))
        output_seq = self.lstm(decoder_input, initial_state=initial_state)
        waypoints = self.out_layer(output_seq)  # (batch, seq_length, 2)
        return waypoints

# ----------------------- Nodo de Entrenamiento End-to-End -----------------------
class NavigationEndToEndTrainer(Node):
    def __init__(self):
        super().__init__('navigation_end2end_trainer')
        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        # Las suscripciones a nodos filtrados y obstaculos se mantienen si las usas en otras partes.
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        self.create_subscription(PoseArray, '/obstacle_points', self.obstacle_points_callback, 10)
        self.create_subscription(Bool, '/virtual_collision', self.collision_callback, 10)
        self.create_subscription(Bool, '/map_reset', self.map_reset_callback, 10)
        # Publicadores
        self.marker_pub = self.create_publisher(Marker, '/planned_path_marker', 10)
        self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
        self.reset_request_pub = self.create_publisher(Bool, '/reset_request', 10)

        # Estado interno
        self.odom = None
        self.goal = None
        self.filtered_nodes = None
        self.occupied_nodes = None
        self.obstacle_points = None
        self.virtual_collision = False

        self.last_movement_time = None
        self.movement_threshold = 0.01
        self.inactivity_time_threshold = 300.0

        self.last_reset_time = 0.0
        self.reset_cooldown = 20.0

        # Buffers de experiencia
        self.actor_inputs = []
        self.states = []
        self.actions = []  
        self.log_probs = []
        self.rewards = []
        self.values = []
        self.dones = []
        self.steps = 0
        self.max_steps = 1000

        self.episode_count = 0
        self.total_episodes = 1000
        self.start_time = time.time()
        self.log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.summary_writer = tf.summary.create_file_writer(self.log_dir)
        self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")

        self.models_saved = False

        # Enfoque end-to-end: en vez de seleccionar un candidato, se genera un path completo.
        self.path_planner = RecurrentPathPlanner(seq_length=MAX_SEQ_LENGTH)
        self.actor_optimizer = tf.keras.optimizers.Adam(3e-4)
        # Para el crítico usamos la misma arquitectura que antes.
        self.critic = tf.keras.Sequential([
            tf.keras.layers.InputLayer(input_shape=(GLOBAL_STATE_DIM,)),
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dense(1)
        ])
        self.critic_optimizer = tf.keras.optimizers.Adam(3e-4)

        # Máquina de estados: "IDLE" para planificar path; "MOVING" para seguir el path.
        self.state = "IDLE"
        self.state_start_time = None

        self.TIMEOUT_THRESHOLD = TIMEOUT_THRESHOLD
        self.WAYPOINT_THRESHOLD = WAYPOINT_THRESHOLD

        self.current_path = None  # Secuencia de waypoints generada
        self.current_waypoint_index = 0

        # Variables para detectar repetición (si se repite el mismo path o se atasa)
        self.same_path_count = 0

        # Reset aleatorio cada X episodios (número aleatorio entre 2 y 5).
        self.reset_interval = random.randint(2, 5)

        self.get_logger().info("Navigation End-to-End Trainer iniciado.")
        while self.odom is None or self.goal is None:
            self.get_logger().warn("Esperando /odom y /goal...")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Datos iniciales recibidos, iniciando entrenamiento.")

        # Timers: uno para la planificación reactiva y otro para el registro de experiencia.
        self.reactive_timer = self.create_timer(0.1, self.reactive_step)
        self.experience_timer = self.create_timer(0.5, self.experience_step)

    # ----------------------- Callbacks -----------------------
    def odom_callback(self, msg: Odometry):
        current_time = self.get_clock().now().nanoseconds / 1e9
        original_pose = msg.pose.pose
        offset_x = 0.5
        adjusted_pose = Pose()
        adjusted_pose.position.x = original_pose.position.x - offset_x
        adjusted_pose.position.y = original_pose.position.y
        adjusted_pose.position.z = original_pose.position.z
        adjusted_pose.orientation = original_pose.orientation
        self.odom = adjusted_pose
        linear = msg.twist.twist.linear
        speed = math.hypot(linear.x, linear.y)
        if self.last_movement_time is None or speed > self.movement_threshold:
            self.last_movement_time = current_time

    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]

    def filtered_nodes_callback(self, msg: PoseArray):
        self.filtered_nodes = msg

    def occupied_nodes_callback(self, msg: PoseArray):
        self.occupied_nodes = msg

    def obstacle_points_callback(self, msg: PoseArray):
        self.obstacle_points = msg

    def collision_callback(self, msg: Bool):
        self.virtual_collision = msg.data

    def map_reset_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Se recibió señal para reiniciar el mapa.")

    # ----------------------- Solicitar Reinicio -----------------------
    def request_environment_reset(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_reset_time >= self.reset_cooldown:
            reset_msg = Bool()
            reset_msg.data = True
            self.reset_request_pub.publish(reset_msg)
            self.get_logger().info("Solicitud de reinicio enviada.")
            self.last_reset_time = current_time
        else:
            self.get_logger().info("Cooldown activo; no se solicita reset.")

    # ----------------------- Función de Recompensa -----------------------
    def compute_reward(self, planned_waypoint):
        current_pos = np.array([self.odom.position.x, self.odom.position.y])
        goal_pos = np.array([self.goal.position.x, self.goal.position.y])
        d_current = np.linalg.norm(current_pos - goal_pos)
        wp = np.array([planned_waypoint.position.x, planned_waypoint.position.y])
        d_final = np.linalg.norm(wp - goal_pos)
        progress_reward = (d_current - d_final) * 20.0
        safety_penalty = 0.0
        if self.obstacle_points is not None and self.obstacle_points.poses:
            d_list = [distance((wp[0], wp[1]), (p.position.x, p.position.y))
                      for p in self.obstacle_points.poses]
            if d_list:
                min_wp = min(d_list)
                if min_wp < 1.0:
                    safety_penalty += (1.0 - min_wp) * 50.0
        collision_penalty = 0.0
        if self.occupied_nodes and self.occupied_nodes.poses:
            for occ in self.occupied_nodes.poses:
                d = distance((self.odom.position.x, self.odom.position.y),
                             (occ.position.x, occ.position.y))
                if d < 0.5:
                    collision_penalty += 1000.0
        reward = progress_reward - safety_penalty - collision_penalty + STEP_PENALTY
        # Penalización extra si el modelo ha generado repetidamente el mismo path
        if self.same_path_count >= SAME_CANDIDATE_THRESHOLD:
            reward -= 20.0
        if d_current < GOAL_REACHED_DIST:
            reward += GOAL_REWARD
        return reward

    # ----------------------- Generación del Path End-to-End -----------------------
    def plan_path(self):
        # Definir el estado global que se usará como entrada (por ejemplo, la posición actual y la meta)
        global_state = np.array([
            self.odom.position.x,
            self.odom.position.y,
            self.goal.position.x,
            self.goal.position.y,
            0.0, 0.0, 0.0  # Placeholders para otras variables si se desean
        ], dtype=np.float32)
        global_state = np.expand_dims(global_state, axis=0)  # (1, GLOBAL_STATE_DIM)
        # El path planner genera una secuencia de waypoints (forma: (1, seq_length, 2))
        waypoints = self.path_planner(global_state).numpy()[0]
        # Convertir cada par (x,y) a un objeto Pose con orientación por defecto.
        path = []
        for wp in waypoints:
            p = Pose()
            p.position.x = float(wp[0])
            p.position.y = float(wp[1])
            p.position.z = 0.0
            p.orientation.w = 1.0
            path.append(p)
        return path

    # ----------------------- Bucle Reactivo: Actualización Continua del Path -----------------------
    def reactive_step(self):
        if self.odom is None or self.goal is None:
            return
        if self.state == "IDLE":
            # Generar un nuevo path completo
            path = self.plan_path()
            if len(path) == 0:
                self.get_logger().warn("No se pudo generar el path.")
                return
            self.current_path = path
            self.current_waypoint_index = 0
            self.get_logger().info(f"Path generado con {len(path)} waypoints.")
            self.state = "MOVING"
            self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        else:
            # En estado MOVING, se actualiza el waypoint actual según el índice.
            if self.current_path is not None and self.current_waypoint_index < len(self.current_path):
                current_wp = self.current_path[self.current_waypoint_index]
                self.current_waypoint = current_wp
                # Publicar el waypoint actual
                nav_points = PoseArray()
                nav_points.header.stamp = self.get_clock().now().to_msg()
                nav_points.header.frame_id = "map"
                nav_points.poses.append(current_wp)
                self.nav_point.publish(nav_points)
                # Visualizar todo el path
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "planned_path"
                marker.id = 0
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.1
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.points = [Point(x=p.position.x, y=p.position.y, z=0.0) for p in self.current_path]
                self.marker_pub.publish(marker)
        # Se podría agregar lógica adicional para replanificar si el path actual se vuelve obsoleto.

    # ----------------------- Bucle de Experiencia: Registro cuando se alcanza un waypoint -----------------------
    def experience_step(self):
        # Verificar si se está atascado (por ejemplo, si se repite el mismo waypoint)
        if self.state == "MOVING":
            current_wp = self.current_waypoint
            if distance((self.odom.position.x, self.odom.position.y),
                        (current_wp.position.x, current_wp.position.y)) < self.WAYPOINT_THRESHOLD:
                self.get_logger().info("Waypoint alcanzado. Registrando experiencia.")
                reward = self.compute_reward(current_wp)
                global_state = np.array([
                    self.odom.position.x,
                    self.odom.position.y,
                    self.goal.position.x,
                    self.goal.position.y,
                    0.0, 0.0, 0.0
                ], dtype=np.float32)
                value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
                done = 1 if distance((self.odom.position.x, self.odom.position.y),
                                      (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST else 0
                self.states.append(global_state)
                # Aquí se puede almacenar la acción, que ahora puede ser el índice del waypoint en el path.
                self.actions.append(self.current_waypoint_index)
                # Como ya no se utiliza la selección discreta de candidatos, se omite log_probs de un softmax.
                self.log_probs.append(0.0)
                self.values.append(value)
                self.rewards.append(reward)
                self.dones.append(done)
                self.actor_inputs.append(global_state)  # Usamos el estado global como input.
                self.steps += 1
                # Avanzar al siguiente waypoint del path.
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.current_path):
                    # Si se completó el path sin llegar al goal, se considera que se está atascado.
                    self.get_logger().warn("Path completado sin alcanzar el goal. Reiniciando candidato.")
                    self.request_environment_reset()
                    self.same_path_count += 1
                    self.state = "IDLE"
            else:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self.state_start_time > self.TIMEOUT_THRESHOLD:
                    self.get_logger().warn("Timeout en MOVING. Penalizando y reiniciando path.")
                    penalty_reward = -10.0
                    global_state = np.array([
                        self.odom.position.x,
                        self.odom.position.y,
                        self.goal.position.x,
                        self.goal.position.y,
                        0.0, 0.0, 0.0
                    ], dtype=np.float32)
                    value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
                    done = 0
                    self.states.append(global_state)
                    self.actions.append(self.current_waypoint_index)
                    self.log_probs.append(0.0)
                    self.values.append(value)
                    self.rewards.append(penalty_reward)
                    self.dones.append(done)
                    self.actor_inputs.append(global_state)
                    self.steps += 1
                    self.request_environment_reset()
                    self.state = "IDLE"

        if len(self.states) > MEMORY_WINDOW_SIZE:
            self.states.pop(0)
            self.actions.pop(0)
            self.log_probs.pop(0)
            self.values.pop(0)
            self.rewards.pop(0)
            self.dones.pop(0)
            self.actor_inputs.pop(0)

        if self.steps >= self.max_steps or (self.odom and self.goal and distance((self.odom.position.x, self.odom.position.y),
                                                    (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST):
            self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
            self.update_model()
            # Reset aleatorio: reiniciar el entorno cada cierto número aleatorio de episodios.
            if self.episode_count % self.reset_interval == 0:
                self.request_environment_reset()
                self.reset_interval = random.randint(2, 5)
            elapsed = time.time() - self.start_time
            avg_ep_time = elapsed / (self.episode_count + 1)
            remaining = (self.total_episodes - (self.episode_count + 1)) * avg_ep_time
            self.get_logger().info(f"Tiempo transcurrido: {elapsed:.1f}s, restante: {remaining:.1f}s")
            self.states = []
            self.actions = []
            self.log_probs = []
            self.values = []
            self.rewards = []
            self.dones = []
            self.actor_inputs = []
            self.steps = 0
            self.episode_count += 1
            self.actor_state = None
            self.state = "IDLE"
            progress_bar_length = 20
            completed_units = int((self.episode_count / self.total_episodes) * progress_bar_length)
            progress_bar = "[" + "#" * completed_units + "-" * (progress_bar_length - completed_units) + "]"
            self.get_logger().info(f"Episodios: {self.episode_count}/{self.total_episodes} {progress_bar}")

    # ----------------------- Cálculo de Ventajas y Actualización del Modelo -----------------------
    def compute_advantages(self, rewards, values, dones):
        advantages = np.zeros_like(rewards, dtype=np.float32)
        last_adv = 0.0
        for t in reversed(range(len(rewards))):
            next_value = values[t+1] if t+1 < len(values) else 0.0
            delta = rewards[t] + GAMMA * next_value * (1 - dones[t]) - values[t]
            advantages[t] = last_adv = delta + GAMMA * LAMBDA * (1 - dones[t]) * last_adv
        returns = advantages + np.array(values, dtype=np.float32)
        return advantages, returns

    def update_model(self):
        states = np.array(self.states, dtype=np.float32)
        actions = np.array(self.actions, dtype=np.int32)
        log_probs_old = np.array(self.log_probs, dtype=np.float32)
        rewards = np.array(self.rewards, dtype=np.float32)
        dones = np.array(self.dones, dtype=np.float32)
        advantages, returns = self.compute_advantages(rewards, self.values, dones)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        actor_inputs = []
        masks = []
        for cand_feat, m in self.actor_inputs:
            actor_inputs.append(cand_feat)
            masks.append(m)
        actor_inputs = np.array(actor_inputs, dtype=np.float32)
        masks = np.array(masks, dtype=bool)
        N = len(states)
        dataset = tf.data.Dataset.from_tensor_slices((states, actor_inputs, masks, actions, log_probs_old, returns, advantages))
        dataset = dataset.shuffle(N).batch(BATCH_SIZE)
        total_actor_loss = 0.0
        total_critic_loss = 0.0
        total_loss_value = 0.0
        batch_count = 0
        for epoch in range(TRAIN_EPOCHS):
            for batch in dataset:
                batch_states, batch_actor_inputs, batch_masks, batch_actions, batch_old_log_probs, batch_returns, batch_advantages = batch
                with tf.GradientTape(persistent=True) as tape:
                    logits, _ = self.path_planner(batch_actor_inputs)  # Aquí se podría ajustar la forma de entrada
                    # Para este ejemplo, no usamos softmax ya que la salida es continua.
                    # Se podría definir una función de pérdida que compare el path generado con una trayectoria óptima.
                    # Como placeholder, usaremos una pérdida simple.
                    path_loss = tf.reduce_mean(tf.square(batch_actor_inputs - logits))
                    values_pred = self.critic(batch_states)
                    critic_loss = tf.reduce_mean(tf.square(batch_returns - values_pred))
                    total_loss = path_loss + 0.5 * critic_loss
                actor_grads = tape.gradient(total_loss, self.path_planner.trainable_variables)
                critic_grads = tape.gradient(total_loss, self.critic.trainable_variables)
                self.actor_optimizer.apply_gradients(zip(actor_grads, self.path_planner.trainable_variables))
                self.critic_optimizer.apply_gradients(zip(critic_grads, self.critic.trainable_variables))
                total_actor_loss += path_loss.numpy()
                total_critic_loss += critic_loss.numpy()
                total_loss_value += total_loss.numpy()
                batch_count += 1
        avg_actor_loss = total_actor_loss / batch_count if batch_count > 0 else 0.0
        avg_critic_loss = total_critic_loss / batch_count if batch_count > 0 else 0.0
        avg_total_loss = total_loss_value / batch_count if batch_count > 0 else 0.0
        self.get_logger().info("Modelo PPO actualizado.")
        episode_reward = np.sum(self.rewards)
        episode_length = len(self.rewards)
        with self.summary_writer.as_default():
            tf.summary.scalar('actor_loss', avg_actor_loss, step=self.episode_count)
            tf.summary.scalar('critic_loss', avg_critic_loss, step=self.episode_count)
            tf.summary.scalar('total_loss', avg_total_loss, step=self.episode_count)
            tf.summary.scalar('episode_reward', episode_reward, step=self.episode_count)
            tf.summary.scalar('episode_length', episode_length, step=self.episode_count)
        self.get_logger().info(f"Métricas registradas para el episodio {self.episode_count}.")
        if not getattr(self, 'models_saved', False):
            self.save_models()
            self.models_saved = True
        if self.episode_count >= 400:
            self.get_logger().info("400 episodios completados. Finalizando nodo.")
            rclpy.shutdown()

    def save_models(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        actor_model_filename = f"actor_model_{timestamp}.keras"
        critic_model_filename = f"critic_model_{timestamp}.keras"
        self.path_planner.save(actor_model_filename)
        self.critic.save(critic_model_filename)
        self.get_logger().info(f"Modelos guardados: {actor_model_filename} y {critic_model_filename}.")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationEndToEndTrainer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
