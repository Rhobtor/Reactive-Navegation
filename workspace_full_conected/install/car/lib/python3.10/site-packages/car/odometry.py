#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf_transformations
import tf2_ros
import math
import numpy as np
from sensor_msgs.msg import JointState

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        # Parámetros de frames
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Estado inicial de la odometría
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None  # Se inicializará con el tiempo de simulación
        self.v = 0.0
        self.w = 0.0

        # Variables para giro de la rueda
        self.wheel_radius = 0.4
        self.wheel_angle = 0.0

        self.sim_time = None  # Tiempo de simulación en segundos
        # Variables para control de cmd_vel
        self.last_cmd_vel_time = None  # Último instante (sim_time) en que se recibió cmd_vel
        self.cmd_vel_timeout = 1.0     # Tiempo (segundos) sin cmd_vel para modo estático

        # Publicadores y tf broadcaster
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Suscripciones: /cmd_vel para velocidades y /sim_time para el tiempo de simulación
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sim_time_sub = self.create_subscription(Float64, '/clock', self.sim_time_callback, 10)
        # Suscribirse a joint_states para obtener los encoders reales
        #self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.get_logger().info("Nodo de odometría iniciado (usando tiempo de simulación)")

    def get_current_sim_time(self):
        """Devuelve el tiempo de simulación actual como objeto rclpy.time.Time.
           Si aún no se ha recibido sim_time, se usa el reloj del sistema."""
        if self.sim_time is not None:
            return rclpy.time.Time(nanoseconds=int(self.sim_time * 1e9))
        else:
            return self.get_clock().now()

    def matrix_from_transform(self, translation, rpy):
        """Devuelve una matriz 4x4 a partir de traslación y ángulos de Euler (rpy)."""
        quat = tf_transformations.quaternion_from_euler(*rpy)
        mat = tf_transformations.quaternion_matrix(quat)
        mat[0:3, 3] = translation
        return mat

    def cmd_vel_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z
        # Actualizamos last_cmd_vel_time usando el tiempo de simulación (si está disponible)
        self.last_cmd_vel_time = self.get_current_sim_time()

    def sim_time_callback(self, msg: Float64):
        self.sim_time = msg.data  # tiempo de simulación en segundos
        current_time = self.get_current_sim_time()

        if self.last_time is None:
            self.last_time = current_time
            return

        # Calcular dt (en segundos) usando el tiempo de simulación
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) * 1e-9

        # Verificar si se ha recibido cmd_vel recientemente
        if (self.last_cmd_vel_time is None) or ((current_time.nanoseconds - self.last_cmd_vel_time.nanoseconds) * 1e-9 > self.cmd_vel_timeout):
            # self.get_logger().info("No se reciben cmd_vel: usando transformación estática")
            self.v = 0.0
            self.w = 0.0

        # Integración de la odometría (modelo diferencial simple)
        delta_x = self.v * math.cos(self.theta) * dt
        delta_y = self.v * math.sin(self.theta) * dt
        delta_theta = self.w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Actualización del ángulo de la rueda a partir de la distancia recorrida
        delta_distance = self.v * dt
        self.wheel_angle += delta_distance / self.wheel_radius

        # Publicar odometría y TF usando el timestamp del tiempo de simulación
        self.publish_odometry(current_time.to_msg())

        # Actualizar el último instante de simulación procesado
        self.last_time = current_time

    def publish_odometry(self, timestamp):
        # --- Publicación de odometría (odom -> base_link) ---
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        # --- Publicación de la transformación (odom -> base_link) ---
        t_base = TransformStamped()
        t_base.header.stamp = timestamp
        t_base.header.frame_id = self.odom_frame
        t_base.child_frame_id = self.base_frame
        t_base.transform.translation.x = self.x
        t_base.transform.translation.y = self.y
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.x = quat[0]
        t_base.transform.rotation.y = quat[1]
        t_base.transform.rotation.z = quat[2]
        t_base.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t_base)

        # # --- Transformación acumulada (odom -> Camara) ---
        # T_odom_base_camara = self.matrix_from_transform([self.x, self.y, 0.0], [0, 0, self.theta])
        # T_base_susp_camara = self.matrix_from_transform([0.925082, 0.0, 0.83922], [0, 0, 0])
        # # T_susp_dir_camara  = self.matrix_from_transform([0.0, 0.0, 0.0], [-1.5708, 0, -1.5708])
        # # T_total_camara = np.dot(np.dot(T_odom_base_camara, T_base_susp_camara), T_susp_dir_camara)
        # T_total_camara = np.dot(T_odom_base_camara, T_base_susp_camara)
        # trans_camara = T_total_camara[0:3, 3]
        # quat_camara = tf_transformations.quaternion_from_matrix(T_total_camara)

        # t_camara = TransformStamped()
        # t_camara.header.stamp = timestamp
        # t_camara.header.frame_id = self.odom_frame
        # t_camara.child_frame_id = "camera_optical_frame"
        # t_camara.transform.translation.x = trans_camara[0]
        # t_camara.transform.translation.y = trans_camara[1]
        # t_camara.transform.translation.z = trans_camara[2]
        # t_camara.transform.rotation.x = quat_camara[0]
        # t_camara.transform.rotation.y = quat_camara[1]
        # t_camara.transform.rotation.z = quat_camara[2]
        # t_camara.transform.rotation.w = quat_camara[3]
        # self.tf_broadcaster.sendTransform(t_camara)


        # # --- Transformación acumulada (odom -> Camara) ---
        # T_odom_base_camara = self.matrix_from_transform([self.x, self.y, 0.0], [0, 0, self.theta])
        # T_base_susp_camara = self.matrix_from_transform([0.925082, 0.0, 0.83922], [0, 0, 0])
        # # T_susp_dir_camara  = self.matrix_from_transform([0.0, 0.0, 0.0], [-1.5708, 0, -1.5708])
        # # T_total_camara = np.dot(np.dot(T_odom_base_camara, T_base_susp_camara), T_susp_dir_camara)
        # T_total_camara = np.dot(T_odom_base_camara, T_base_susp_camara)
        # trans_camara = T_total_camara[0:3, 3]
        # quat_camara = tf_transformations.quaternion_from_matrix(T_total_camara)

        # t_camara = TransformStamped()
        # t_camara.header.stamp = timestamp
        # t_camara.header.frame_id = self.odom_frame
        # t_camara.child_frame_id = "camera_optical_frame"
        # t_camara.transform.translation.x = trans_camara[0]
        # t_camara.transform.translation.y = trans_camara[1]
        # t_camara.transform.translation.z = trans_camara[2]
        # t_camara.transform.rotation.x = quat_camara[0]
        # t_camara.transform.rotation.y = quat_camara[1]
        # t_camara.transform.rotation.z = quat_camara[2]
        # t_camara.transform.rotation.w = quat_camara[3]
        # self.tf_broadcaster.sendTransform(t_camara)

        # --- Transformación acumulada (odom -> rim_front_left_1) ---
        T_odom_base_FL = self.matrix_from_transform([self.x, self.y, 0.0], [0, 0, self.theta])
        T_base_susp_FL = self.matrix_from_transform([1.925, 0.875, -0.45], [0, 0, 0])
        T_susp_dir_FL  = self.matrix_from_transform([0.0, 0.7125, 0.0], [0, 0, 0])
        T_dir_rim_FL   = self.matrix_from_transform([0.0, 0.1625, 0.0], [0, self.wheel_angle, 0])
        T_total_FL = np.dot(np.dot(np.dot(T_odom_base_FL, T_base_susp_FL), T_susp_dir_FL), T_dir_rim_FL)
        trans_FL = T_total_FL[0:3, 3]
        quat_FL = tf_transformations.quaternion_from_matrix(T_total_FL)

        t_FL = TransformStamped()
        t_FL.header.stamp = timestamp
        t_FL.header.frame_id = self.odom_frame
        t_FL.child_frame_id = "rim_front_left_1"
        t_FL.transform.translation.x = trans_FL[0]
        t_FL.transform.translation.y = trans_FL[1]
        t_FL.transform.translation.z = trans_FL[2]
        t_FL.transform.rotation.x = quat_FL[0]
        t_FL.transform.rotation.y = quat_FL[1]
        t_FL.transform.rotation.z = quat_FL[2]
        t_FL.transform.rotation.w = quat_FL[3]
        self.tf_broadcaster.sendTransform(t_FL)

        # --- Transformación acumulada (odom -> rim_front_right_1) ---
        T_odom_base_FR = self.matrix_from_transform([self.x, self.y, 0.0], [0, 0, self.theta])
        T_base_susp_FR = self.matrix_from_transform([1.925, 0.625, -0.45], [0, 0, 0])
        T_susp_dir_FR  = self.matrix_from_transform([0.0, -0.7125, 0.0], [0, 0, 0])
        T_dir_rim_FR   = self.matrix_from_transform([0.0, -0.1625, 0.0], [0, self.wheel_angle, 0])
        T_total_FR = np.dot(np.dot(np.dot(T_odom_base_FR, T_base_susp_FR), T_susp_dir_FR), T_dir_rim_FR)
        trans_FR = T_total_FR[0:3, 3]
        quat_FR = tf_transformations.quaternion_from_matrix(T_total_FR)

        t_FR = TransformStamped()
        t_FR.header.stamp = timestamp
        t_FR.header.frame_id = self.odom_frame
        t_FR.child_frame_id = "rim_front_right_1"
        t_FR.transform.translation.x = trans_FR[0]
        t_FR.transform.translation.y = trans_FR[1]
        t_FR.transform.translation.z = trans_FR[2]
        t_FR.transform.rotation.x = quat_FR[0]
        t_FR.transform.rotation.y = quat_FR[1]
        t_FR.transform.rotation.z = quat_FR[2]
        t_FR.transform.rotation.w = quat_FR[3]
        self.tf_broadcaster.sendTransform(t_FR)

        # --- Transformación acumulada (odom -> rim_back_right_1) ---
        T_odom_base_BR = self.matrix_from_transform([self.x, self.y, 0.0], [0, 0, self.theta])
        T_base_susp_BR = self.matrix_from_transform([0.575, 0.625, -0.45], [0, 0, 0])
        T_susp_dir_BR  = self.matrix_from_transform([0.0, -0.7125, 0.0], [0, 0, 0])
        T_dir_rim_BR   = self.matrix_from_transform([0.005, -0.1625, 0.0], [0, self.wheel_angle, 0])
        T_total_BR = np.dot(np.dot(np.dot(T_odom_base_BR, T_base_susp_BR), T_susp_dir_BR), T_dir_rim_BR)
        trans_BR = T_total_BR[0:3, 3]
        quat_BR = tf_transformations.quaternion_from_matrix(T_total_BR)

        t_BR = TransformStamped()
        t_BR.header.stamp = timestamp
        t_BR.header.frame_id = self.odom_frame
        t_BR.child_frame_id = "rim_back_right_1"
        t_BR.transform.translation.x = trans_BR[0]
        t_BR.transform.translation.y = trans_BR[1]
        t_BR.transform.translation.z = trans_BR[2]
        t_BR.transform.rotation.x = quat_BR[0]
        t_BR.transform.rotation.y = quat_BR[1]
        t_BR.transform.rotation.z = quat_BR[2]
        t_BR.transform.rotation.w = quat_BR[3]
        self.tf_broadcaster.sendTransform(t_BR)

        # --- Transformación acumulada (odom -> rim_back_left_1) ---
        T_odom_base_BL = self.matrix_from_transform([self.x, self.y, 0.0], [0, 0, self.theta])
        T_base_susp_BL = self.matrix_from_transform([0.575, 0.875, -0.45], [0, 0, 0])
        T_susp_dir_BL  = self.matrix_from_transform([0.0, 0.7125, 0.0], [0, 0, 0])
        T_dir_rim_BL   = self.matrix_from_transform([0.005, 0.1625, 0.0], [0, self.wheel_angle, 0])
        T_total_BL = np.dot(np.dot(np.dot(T_odom_base_BL, T_base_susp_BL), T_susp_dir_BL), T_dir_rim_BL)
        trans_BL = T_total_BL[0:3, 3]
        quat_BL = tf_transformations.quaternion_from_matrix(T_total_BL)

        t_BL = TransformStamped()
        t_BL.header.stamp = timestamp
        t_BL.header.frame_id = self.odom_frame
        t_BL.child_frame_id = "rim_back_left_1"
        t_BL.transform.translation.x = trans_BL[0]
        t_BL.transform.translation.y = trans_BL[1]
        t_BL.transform.translation.z = trans_BL[2]
        t_BL.transform.rotation.x = quat_BL[0]
        t_BL.transform.rotation.y = quat_BL[1]
        t_BL.transform.rotation.z = quat_BL[2]
        t_BL.transform.rotation.w = quat_BL[3]
        self.tf_broadcaster.sendTransform(t_BL)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
