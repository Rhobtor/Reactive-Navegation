import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class LidarXYAndZFilter(Node):
    def __init__(self):
        super().__init__('lidar_xy_z_filter')
        # Suscripción a la nube de puntos original
        self.subscription = self.create_subscription(
            PointCloud2,
            '/scan_cloud',
            self.listener_callback,
            10)
        # Publicador para la nube de puntos filtrada
        self.publisher = self.create_publisher(PointCloud2, '/scan_cloud_filtered', 10)

    def listener_callback(self, msg):
        # Obtiene los nombres de los campos disponibles, por ejemplo: ['x', 'y', 'z', 'intensity']
        fields_names = [field.name for field in msg.fields]
        try:
            x_index = fields_names.index("x")
            y_index = fields_names.index("y")
            z_index = fields_names.index("z")
        except ValueError as e:
            self.get_logger().error("No se encontraron los campos necesarios: " + str(e))
            return

        # Lee todos los puntos del mensaje (todos los campos)
        points = list(pc2.read_points(msg, skip_nans=True))
        if not points:
            self.get_logger().info("No se recibieron puntos en el mensaje")
            return

        # Convierte cada punto a una lista y luego a un array 2D
        points_array = np.array([list(p) for p in points], dtype=np.float32)
        if points_array.ndim == 1:
            points_array = points_array.reshape(1, -1)

        # Define los límites de la región que ocupa el coche (en el plano xy)
        # Ajusta estos valores según las dimensiones y posición de tu modelo
        x_min = -1.6  # límite izquierdo
        x_max =  1.6  # límite derecho
        y_min = -1.1  # límite trasero
        y_max =  1.6  # límite delantero

        # Define los límites para la coordenada z
        z_min = -6.0  # Evita puntos muy cercanos al suelo
        z_max = 5.0  # Evita puntos que toquen el modelo

        # Crea una máscara para filtrar puntos que estén DENTRO de la caja (modelo) en xy
        mask_xy = ~((points_array[:, x_index] > x_min) & (points_array[:, x_index] < x_max) &
                    (points_array[:, y_index] > y_min) & (points_array[:, y_index] < y_max))
        # Crea una máscara para conservar solo puntos con z en el rango deseado
        mask_z = (points_array[:, z_index] > z_min) & (points_array[:, z_index] < z_max)

        # La máscara final es la combinación de ambas condiciones
        mask = mask_xy & mask_z
        filtered_points = points_array[mask]

        # Reconstruye la nube de puntos filtrada, conservando la misma estructura (todos los campos)
        filtered_points_list = [tuple(point) for point in filtered_points]
        filtered_cloud = pc2.create_cloud(msg.header, msg.fields, filtered_points_list)

        # Publica la nube filtrada
        self.publisher.publish(filtered_cloud)
        # self.get_logger().info(
        #     f"Publicado nube filtrada: {filtered_points.shape[0]} puntos de {points_array.shape[0]}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarXYAndZFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
