import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from nav_msgs.msg import Odometry as FilteredOdometry
import numpy as np

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # Parámetros del filtro EKF (puedes ajustarlos desde el archivo YAML)
        self.declare_parameter('odometry_noise', [0.1, 0.1, 0.1])  # Ruido de odometría
        self.declare_parameter('imu_noise', [0.01, 0.01, 0.01])  # Ruido IMU
        self.declare_parameter('ekf_update_frequency', 10.0)  # Frecuencia de actualización en Hz

        # Obtener los valores de los parámetros
        self.odometry_noise = self.get_parameter('odometry_noise').get_parameter_value().double_array_value
        self.imu_noise = self.get_parameter('imu_noise').get_parameter_value().double_array_value
        self.ekf_update_frequency = self.get_parameter('ekf_update_frequency').get_parameter_value().double_value

        # Inicialización de variables de estado
        self.x = np.zeros(6)  # Estado del EKF: [x, y, theta, vx, vy, vtheta]
        self.P = np.eye(6)  # Matriz de covarianza

        # Suscripción a los tópicos de odometría e IMU
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odometry',
            self.odom_callback,
            10
        )
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publicador de la odometría filtrada
        self.filtered_odometry_publisher = self.create_publisher(
            FilteredOdometry,
            '/odometry/filtered',
            10
        )

    def odom_callback(self, msg):
        """Recibe la odometría y aplica el EKF"""
        # Extraer la odometría (posición y velocidad)
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear

        # Integración del estado (por ejemplo, simple modelo de odometría)
        dt = 1.0 / self.ekf_update_frequency  # El intervalo de tiempo

        # Aquí es donde se aplicaría el EKF para fusionar las mediciones de odometría
        # con el IMU (suponiendo que los datos de velocidad y orientación estén disponibles)
        
        # Actualizar el estado basado en la odometría (esto es solo un ejemplo)
        self.x[0] += velocity.x * dt  # Actualizar posición X
        self.x[1] += velocity.y * dt  # Actualizar posición Y
        self.x[2] += velocity.z * dt  # Actualizar orientación (suponiendo que z es theta)

        # Publicar la odometría filtrada
        filtered_odom = FilteredOdometry()
        filtered_odom.header.stamp = self.get_clock().now().to_msg()
        filtered_odom.header.frame_id = 'map'
        filtered_odom.pose.pose.position.x = self.x[0]
        filtered_odom.pose.pose.position.y = self.x[1]
        filtered_odom.pose.pose.orientation.z = self.x[2]
        self.filtered_odometry_publisher.publish(filtered_odom)

    def imu_callback(self, msg):
        """Recibe el IMU y aplica el EKF"""
        # Extraer datos de IMU (orientación y velocidad angular)
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity

        # Fusionar datos de IMU con el EKF (esto es solo un ejemplo simple)
        # Aquí es donde el EKF usaría la orientación (yaw, pitch, roll) y la velocidad angular
        # para mejorar la estimación de la posición y orientación

        # Actualización del estado del EKF basado en los datos del IMU
        dt = 1.0 / self.ekf_update_frequency
        self.x[2] += angular_velocity.z * dt  # Actualizar la orientación

        # Publicar el estado actualizado del filtro (si es necesario)
        self.get_logger().info(f"EKF State: {self.x}")

def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFNode()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
