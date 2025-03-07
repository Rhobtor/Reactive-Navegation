#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, PoseArray, Point, PoseStamped
from std_msgs.msg import Float64
from car_interfaces.msg import Graph
from std_srvs.srv import Empty
import time
import numpy as np
import logging
import math
from squaternion import Quaternion
import torch
import torch.nn as nn
import torch.optim as optim
import random
from collections import deque
import json  # Para guardar los datos de replay

# Parámetros de la simulación y del entorno
GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 2.0
MAX_FRONTIER_POINTS = 30  # Número máximo de nodos frontera a considerar en el estado
REPLAY_FILE = "replay_data.json"  # Archivo donde se guardarán los datos de replay

# Configuración del logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('GazeboTest')

#######################################
# Definición del modelo DQN y el agente
#######################################

class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, 64)
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, action_size)
    
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size   = state_size
        self.action_size  = action_size
        self.memory       = deque(maxlen=10000)
        self.gamma        = 0.99
        self.epsilon      = 1.0
        self.epsilon_decay = 0.995
        self.epsilon_min  = 0.01
        self.learning_rate = 0.001
        self.batch_size   = 32
        self.device       = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model        = DQN(state_size, action_size).to(self.device)
        self.optimizer    = optim.Adam(self.model.parameters(), lr=self.learning_rate)
        self.loss_fn      = nn.MSELoss()
    
    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
    
    def act(self, state):
        if np.random.rand() < self.epsilon:
            return random.randrange(self.action_size)
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.model(state_tensor)
        return torch.argmax(q_values).item()
    
    def replay(self):
        if len(self.memory) < self.batch_size:
            return
        minibatch = random.sample(self.memory, self.batch_size)
        states     = torch.FloatTensor([m[0] for m in minibatch]).to(self.device)
        actions    = torch.LongTensor([m[1] for m in minibatch]).to(self.device)
        rewards    = torch.FloatTensor([m[2] for m in minibatch]).to(self.device)
        next_states= torch.FloatTensor([m[3] for m in minibatch]).to(self.device)
        dones      = torch.FloatTensor([float(m[4]) for m in minibatch]).to(self.device)
        
        q_values = self.model(states).gather(1, actions.unsqueeze(1)).squeeze(1)
        with torch.no_grad():
            next_q_values = self.model(next_states).max(1)[0]
        targets = rewards + self.gamma * next_q_values * (1 - dones)
        
        loss = self.loss_fn(q_values, targets)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

#######################################
# Clase del entorno Gazebo
#######################################

class GazeboTest:
    def __init__(self, launch_file):
        # Variables para el objetivo y posición del robot
        self.goal_x = 0.0  
        self.goal_y = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        
        # Variables para almacenar datos recibidos de los tópicos
        self.total_entropy = 0.0
        self.ptos_entropy  = 0.0
        self.frontier      = None
        self.map_data      = None
        self.last_odom     = None
        
        # Estado del robot en cada acción
        self.set_self_state = ModelState()
        self.set_self_state.model_name = "car"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0
        
        # Inicializa rclpy y crea el nodo
        rclpy.init()
        self.node = rclpy.create_node('gazebo_control')
        
        # Subscripciones a tópicos
        self.odom_sub = self.node.create_subscription(ModelState, '/odom', self.odom_callback, 10)
        # self.map_2d   = self.node.create_subscription(Graph, '/ground_map', self.map_callback, 10)
        self.total_entropy_sub = self.node.create_subscription(Float64, '/total_entropy', self.total_entropy_callback, 10)
        self.ptos_entropy_sub  = self.node.create_subscription(Float64, '/frontier_entropies', self.ptos_entropy_callback, 10)
        self.frontier_sub = self.node.create_subscription(PoseArray, '/frontier_points', self.frontier_callback, 10)
        
        # Publicadores
        # self.vel_pub   = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub  = self.node.create_publisher(PoseStamped, '/goal', 10)
        # Para setear la posición del modelo (ej. reset o visualización)
        self.set_state = self.node.create_publisher(ModelState, '/gazebo/set_model_state', 10)
        
        # Lanzar Gazebo
        self.launch_gazebo(launch_file)
    
    # def map_callback(self, msg):
    #     self.map_data = msg

    def total_entropy_callback(self, msg):
        self.total_entropy = msg.data
    
    def ptos_entropy_callback(self, msg):
        self.ptos_entropy = msg.data
    
    def frontier_callback(self, msg):
        self.frontier = msg
    
    def odom_callback(self, msg):
        self.odom_x = msg.pose.position.x
        self.odom_y = msg.pose.position.y
        self.last_odom = msg
    
    # Construir el estado para el agente DRL
    def get_state(self):
        # Estado: [odom_x, odom_y, total_entropy, (x,y) de hasta MAX_FRONTIER_POINTS]
        state = [self.odom_x, self.odom_y, self.total_entropy]
        frontier_list = []
        if self.frontier is not None and len(self.frontier.poses) > 0:
            for pose in self.frontier.poses[:MAX_FRONTIER_POINTS]:
                frontier_list.extend([pose.position.x, pose.position.y])
            # Si hay menos de MAX_FRONTIER_POINTS, rellenar con ceros
            while len(frontier_list) < MAX_FRONTIER_POINTS * 2:
                frontier_list.extend([0.0, 0.0])
        else:
            frontier_list = [0.0] * (MAX_FRONTIER_POINTS * 2)
        state.extend(frontier_list)
        return np.array(state, dtype=np.float32)
    
    # Ejecuta una acción: la acción es el índice del nodo frontera seleccionado
    def step(self, action_index):
        done = False
        target_reached = False
        
        # Obtener la lista de nodos frontera disponibles
        frontier_coords = []
        if self.frontier is not None and len(self.frontier.poses) > 0:
            for pose in self.frontier.poses[:MAX_FRONTIER_POINTS]:
                frontier_coords.append((pose.position.x, pose.position.y))
        # Si no hay nodos disponibles, penalizar y terminar el episodio
        if len(frontier_coords) == 0:
            logger.warning("No hay nodos frontera disponibles.")
            return self.get_state(), -1.0, True, False
        
        # Si el índice está fuera del rango, se selecciona el primero
        if action_index >= len(frontier_coords):
            action_index = 0
        
        action_coord = frontier_coords[action_index]
        
        # Actualizar el objetivo con la coordenada seleccionada
        self.goal_x = action_coord[0]
        self.goal_y = action_coord[1]
        self.goal = PoseStamped()
        self.goal.header.stamp = self.node.get_clock().now().to_msg()
        self.goal.header.frame_id = "map"  # or the appropriate frame
        self.goal.pose.position.x = self.goal_x
        self.goal.pose.position.y = self.goal_y
        # Set the orientation if needed, e.g., identity quaternion:
        self.goal.pose.orientation.x = 0.0
        self.goal.pose.orientation.y = 0.0
        self.goal.pose.orientation.z = 0.0
        self.goal.pose.orientation.w = 1.0
        self.goal_pub.publish(self.goal)
        # self.publish_markers(action_coord)
        
        # Guardar la entropía previa para calcular la recompensa
        prev_entropy = self.total_entropy
        
        # Despausar la simulación para propagar el cambio
        self.unpause_simulation()
        time.sleep(TIME_DELTA)
        self.pause_simulation()
        
        # Actualizar la odometría
        if self.last_odom is not None:
            self.odom_x = self.last_odom.pose.position.x
            self.odom_y = self.last_odom.pose.position.y
        
        # Calcular la distancia al objetivo
        distance = np.linalg.norm([self.odom_x - self.goal_x, self.odom_y - self.goal_y])
        
        # Recompensa: si se alcanza el objetivo se asigna una recompensa grande;
        # de lo contrario se utiliza la disminución de entropía (o una penalización leve)
        if distance < GOAL_REACHED_DIST:
            reward = 10.0
            done = True
            target_reached = True
        else:
            reward = prev_entropy - self.total_entropy
            if reward == 0.0:
                reward = -0.1
        
        next_state = self.get_state()
        return next_state, reward, done, target_reached
    
    def publish_markers(self, action_coord):
        # Publica un marcador para visualizar el nodo frontera seleccionado en RViz
        marker = ModelState()
        marker.model_name = "selected_frontier"
        marker.pose.position.x = action_coord[0]
        marker.pose.position.y = action_coord[1]
        self.set_state.publish(marker)
    
    def launch_gazebo(self, launch_file):
        port = 11345
        logger.info(f"Lanzando ROS 2 con puerto {port}...")
        subprocess.Popen(["ros2", "launch", launch_file, f"port:={port}"])
        logger.info("Gazebo iniciado!")
        time.sleep(5)
        self.unpause_simulation()
    
    def unpause_simulation(self):
        unpause_client = self.node.create_client(Empty, '/unpause_physics')
        if not unpause_client.wait_for_service(timeout_sec=5.0):
            logger.error("Servicio /unpause_physics no disponible")
            return False
        request = Empty.Request()
        future = unpause_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            logger.info("Simulación reanudada!")
            return True
        else:
            logger.error("Error al reanudar la simulación")
            return False
    
    def pause_simulation(self):
        pause_client = self.node.create_client(Empty, '/pause_physics')
        if not pause_client.wait_for_service(timeout_sec=5.0):
            logger.error("Servicio /pause_physics no disponible")
            return False
        request = Empty.Request()
        future = pause_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            logger.info("Simulación pausada!")
            return True
        else:
            logger.error("Error al pausar la simulación")
            return False
    
    def reset(self):
        # Reinicia la simulación
        reset_client = self.node.create_client(Empty, '/reset_simulation')
        if not reset_client.wait_for_service(timeout_sec=5.0):
            logger.error("Servicio /reset_simulation no disponible")
        request = Empty.Request()
        future = reset_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            logger.info("Simulación reseteada!")
        else:
            logger.error("Error al resetear la simulación")
        
        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        object_state = self.set_self_state
        
        x = np.random.uniform(-4.5, 4.5)
        y = np.random.uniform(-4.5, 4.5)
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)
        
        self.odom_x = x
        self.odom_y = y
        
        self.change_goal()
        self.unpause_simulation()
        time.sleep(TIME_DELTA)
        self.pause_simulation()
        
        return self.get_state()
    
    def change_goal(self):
        # Establece un objetivo aleatorio en el entorno
        self.goal_x = np.random.uniform(-4.5, 4.5)
        self.goal_y = np.random.uniform(-4.5, 4.5)
        goal_state = PoseStamped()
        goal_state.header.stamp = self.node.get_clock().now().to_msg()
        goal_state.header.frame_id = "map"  # Or the appropriate frame
        goal_state.pose.position.x = self.goal_x
        goal_state.pose.position.y = self.goal_y
        # Set a default orientation (identity quaternion)
        goal_state.pose.orientation.x = 0.0
        goal_state.pose.orientation.y = 0.0
        goal_state.pose.orientation.z = 0.0
        goal_state.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_state)
    
    # Función de recompensa alternativa (si se desea modificar la heurística)
    def reward(self, target, action):
        if target == "arrive":
            return 10.0
        elif target == "colision":
            return -10.0
        else:
            return -0.1

#######################################
# Ciclo de entrenamiento del agente DQN
#######################################

def train_agent():
    launch_file = "/home/rhobtor/reactive/Reactive-Navegation/workspace_full_conected/src/car_python/launch/gazebo_mountain.launch.py"  # Actualiza con tu archivo launch
    env = GazeboTest(launch_file)
    
    # Definir el tamaño del estado: [odom_x, odom_y, total_entropy] + (x,y) de MAX_FRONTIER_POINTS
    state_size = 3 + (MAX_FRONTIER_POINTS * 2)
    action_size = MAX_FRONTIER_POINTS  # El agente elige entre hasta MAX_FRONTIER_POINTS
    agent = DQNAgent(state_size, action_size)
    
    num_episodes = 3
    max_steps = 2  # Número máximo de pasos por episodio

    # Lista para guardar datos de replay
    episodes_data = []
    
    for e in range(num_episodes):
        state = env.reset()
        total_reward = 0.0
        episode_steps = []  # Almacena cada paso del episodio
        
        for step in range(max_steps):
            current_state = env.get_state()
            action = agent.act(current_state)
            # Ejecutar la acción y obtener la respuesta del entorno
            next_state, reward, done, target_reached = env.step(action)
            agent.remember(current_state, action, reward, next_state, done)
            agent.replay()
            total_reward += reward

            # Guardar datos del paso para el replay
            step_data = {
                "timestamp": time.time(),  # Puede usarse time.time() o el índice 'step'
                "action": action,
                "goal": {"x": env.goal_x, "y": env.goal_y},
                "reward": reward
            }
            episode_steps.append(step_data)
            
            if done:
                break
        
        logger.info(f"Episode {e+1}/{num_episodes} - Total Reward: {total_reward}")
        # Guardar datos del episodio en la lista de replay
        episode_data = {
            "episode": e+1,
            "steps": episode_steps,
            "total_reward": total_reward
        }
        episodes_data.append(episode_data)
    
    # Guardar todos los datos de replay en un archivo JSON
    with open(REPLAY_FILE, 'w') as f:
        json.dump(episodes_data, f, indent=4)
    logger.info(f"Datos de replay guardados en {REPLAY_FILE}")
    
    # Guardar el modelo entrenado
    torch.save(agent.model.state_dict(), "dqn_model.pth")
    logger.info("Modelo DQN guardado.")

if __name__ == '__main__':
    try:
        train_agent()
    except Exception as e:
        logger.error(f"Error en la ejecución: {e}")
