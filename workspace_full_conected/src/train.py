#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64
from std_srvs.srv import Empty
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, optimizers, losses
import random
from collections import deque
from squaternion import Quaternion
import time
from std_msgs.msg import Bool
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('GazeboDQN')

MAX_FRONTIER_POINTS = 30
GOAL_REACHED_DIST = 2.5

class DQN(tf.keras.Model):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = layers.Dense(64, activation='relu')
        self.fc2 = layers.Dense(64, activation='relu')
        self.fc3 = layers.Dense(action_size)

    def call(self, x):
        x = self.fc1(x)
        x = self.fc2(x)
        return self.fc3(x)

class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=10000)
        self.gamma = 0.99
        self.epsilon = 1.0
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01
        self.learning_rate = 0.001
        self.batch_size = 32
        self.model = DQN(state_size, action_size)
        self.optimizer = optimizers.Adam(self.learning_rate)
        self.loss_fn = losses.MeanSquaredError()

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() < self.epsilon:
            return random.randrange(self.action_size)
        q_values = self.model(np.array([state]))
        return np.argmax(q_values.numpy()[0])

    def replay(self):
        if len(self.memory) < self.batch_size:
            return
        minibatch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*minibatch)
        states = np.array(states)
        next_states = np.array(next_states)

        with tf.GradientTape() as tape:
            q_values = self.model(states)
            q_values_taken = tf.reduce_sum(q_values * tf.one_hot(actions, self.action_size), axis=1)
            next_q_values = self.model(next_states)
            targets = rewards + self.gamma * np.max(next_q_values, axis=1) * (1 - np.array(dones))
            loss = self.loss_fn(targets, q_values_taken)

        grads = tape.gradient(loss, self.model.trainable_variables)
        self.optimizer.apply_gradients(zip(grads, self.model.trainable_variables))

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

class GazeboEnv(Node):
    def __init__(self):
        super().__init__('gazebo_dqn')
        self.goal_reached = False
        self.odom_x = self.odom_y = self.total_entropy = 0.0
        self.frontier = None

        self.create_subscription(Bool, '/goal_reached', self.goal_reached_cb, 10)
        self.create_subscription(ModelState, '/odom', self.odom_cb, 10)
        self.create_subscription(Float64, '/total_entropy', self.entropy_cb, 10)
        self.create_subscription(PoseArray, '/frontier_points', self.frontier_cb, 10)

        self.goal_pub = self.create_publisher(PoseArray, '/goal', 10)
        self.state_pub = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)

    def goal_reached_cb(self, msg):
        self.goal_reached = msg.data

    def odom_cb(self, msg):
        self.odom_x, self.odom_y = msg.pose.position.x, msg.pose.position.y

    def entropy_cb(self, msg):
        self.total_entropy = msg.data

    def frontier_cb(self, msg):
        self.frontier = msg

    def get_state(self):
        state = [self.odom_x, self.odom_y, self.total_entropy]
        points = self.frontier.poses if self.frontier else []
        for pose in points[:MAX_FRONTIER_POINTS]:
            state.extend([pose.position.x, pose.position.y])
        state += [0.0] * ((MAX_FRONTIER_POINTS - len(points)) * 2)
        return np.array(state)

    def send_goal(self, x, y):
        goal_msg = PoseArray()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        pose = Pose()
        pose.position.x, pose.position.y, pose.orientation.w = x, y, 1.0
        goal_msg.poses.append(pose)
        self.goal_pub.publish(goal_msg)
        logger.info(f'Nuevo objetivo enviado: x={x:.2f}, y={y:.2f}')

    def reset_env(self):
        reset_client = self.create_client(Empty, '/reset_simulation')
        reset_client.wait_for_service()
        reset_future = reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, reset_future)

        octomap_reset_client = self.create_client(Empty, '/octomap_server/reset')
        octomap_reset_client.wait_for_service()
        octomap_future = octomap_reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, octomap_future)

        time.sleep(2)  # Espera mayor para asegurar que tf2 actualice correctamente

        angle = np.random.uniform(-np.pi, np.pi)
        q = Quaternion.from_euler(0, 0, angle)
        state = ModelState(model_name='car', pose=Pose())
        state.pose.position.x = np.random.uniform(-4.5, 4.5)
        state.pose.position.y = np.random.uniform(-4.5, 4.5)
        state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = q.x, q.y, q.z, q.w
        self.state_pub.publish(state)

        time.sleep(2)  # Tiempo adicional tras posicionar robot

        return self.get_state()

def main():
    rclpy.init()
    env = GazeboEnv()
    state_size = 3 + MAX_FRONTIER_POINTS * 2
    action_size = MAX_FRONTIER_POINTS
    agent = DQNAgent(state_size, action_size)

    for episode in range(10):
        state = env.reset_env()
        total_reward = 0
        for step in range(10):
            while env.frontier is None or len(env.frontier.poses) == 0:
                rclpy.spin_once(env, timeout_sec=0.1)

            action = agent.act(state)
            frontier = env.frontier.poses[action % len(env.frontier.poses)]
            env.send_goal(frontier.position.x, frontier.position.y)

            env.goal_reached = False
            while not env.goal_reached:
                rclpy.spin_once(env, timeout_sec=0.1)

            next_state = env.get_state()
            reward = env.total_entropy - state[2]
            done = step == 9
            agent.remember(state, action, reward, next_state, done)
            agent.replay()

            state = next_state
            total_reward += reward
            if done:
                break
        logger.info(f'Episode {episode+1} reward: {total_reward}')

    agent.model.save_weights('dqn_model.h5')

if __name__ == '__main__':
    main()
