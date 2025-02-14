#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu

class SyncCameraNode(Node):
    def __init__(self):
        super().__init__('sync_camera_node')
        self.latest_imu_stamp = None

        # Suscribirse al IMU para obtener el timestamp
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        # Suscribirse a los mensajes de la cámara
        self.create_subscription(Image, '/depth_camera/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/depth_camera/camera_info', self.camera_info_callback, 10)

        # Publicadores para los mensajes "sincronizados"
        self.image_pub = self.create_publisher(Image, '/depth_camera/image_raw_sync', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/depth_camera/camera_info_sync', 10)
        self.get_logger().info("Nodo de sincronización de cámara iniciado.")

    def imu_callback(self, msg: Imu):
        # Guardar el último timestamp recibido del IMU
        self.latest_imu_stamp = msg.header.stamp

    def image_callback(self, msg: Image):
        if self.latest_imu_stamp is not None:
            # Reemplazar el timestamp con el del IMU
            msg.header.stamp = self.latest_imu_stamp
        self.image_pub.publish(msg)

    def camera_info_callback(self, msg: CameraInfo):
        if self.latest_imu_stamp is not None:
            # Reemplazar el timestamp con el del IMU
            msg.header.stamp = self.latest_imu_stamp
        self.camera_info_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SyncCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
