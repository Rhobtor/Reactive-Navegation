#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        # Parámetros de frames: asegúrate que coincidan con lo usado en SLAM Toolbox
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Estado inicial de la pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        # Publicador de odometría y TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Suscripción al tópico joint_states (puedes adaptar si usas otro)
        self.joint_sub = self.create_subscription(LaserScan, '/scan', self.joint_state_callback, 10)
        self.joint_sub = self.create_subscription(LaserScan, '/', self.joint_state_callback, 10)
        self.get_logger().info("Nodo de odometría iniciado")
        self.get_logger().info("Nodo de odometría iniciado")

    def joint_state_callback(self, msg: LaserScan):
        # Usamos el timestamp del mensaje de joint_states
        now = msg.header.stamp
        if self.last_time is None:
            self.last_time = now
            return

        # Calculamos la duración (dt) en segundos a partir de los timestamps
        dt = (now.sec - self.last_time.sec) + (now.nanosec - self.last_time.nanosec) * 1e-9

        # Aquí deberías extraer de msg o de tus sensores los datos reales de velocidad.
        # En este ejemplo simplificado se usan valores ficticios (v = 0.5 m/s, w = 0.1 rad/s).
        # Para un robot de 4 ruedas con dirección deberás adaptar la cinemática (por ejemplo, usando el modelo de bicicleta).
        v = 0.5  # velocidad lineal [m/s]
        w = 0.1  # velocidad angular [rad/s]

        # Integración simple de la odometría (modelo diferencial básico)
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publicamos odometría y TF usando el mismo timestamp
        self.publish_odometry(now)

        self.last_time = now

    def publish_odometry(self, timestamp):
        # Creamos el mensaje de odometría
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Calculamos la orientación a partir de theta
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Para este ejemplo, publicamos las velocidades fijas
        odom.twist.twist.linear.x = 0.5
        odom.twist.twist.angular.z = 0.1

        self.odom_pub.publish(odom)

        # Creamos y publicamos la transformación TF con el mismo timestamp
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
