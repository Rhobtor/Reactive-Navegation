import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ProjectedMapSubscriber(Node):
    def __init__(self):
        super().__init__('projected_map_subscriber')
        
        # Suscripción al mapa proyectado
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/projected_map',  # Tópico del mapa proyectado
            self.map_callback,
            10
        )
        
        # Publicador para los marcadores en RViz
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)

        self.get_logger().info('Nodo suscrito al tópico /projected_map')

    def map_callback(self, msg):
        # Procesar el mapa proyectado recibido
        self.get_logger().info('Mapa proyectado recibido')

        # Procesamos los datos de la celda ocupada (por ejemplo, los nodos ocupados o las fronteras)
        self.process_map(msg)

    def process_map(self, msg):
        if not msg.data:
            self.get_logger().warning('Datos del mapa vacíos')
            return

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Creación del marcador
        marker = Marker()
        marker.header.frame_id = "map"  # El marco de referencia del mapa en RViz
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "map_boundaries"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0  # Orientación para no rotar
        marker.scale.x = 1.0  # Escala de la esfera
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0  # Opacidad de la esfera
        marker.color.r = 1.0  # Rojo
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Agregar puntos que estén ocupados o que representen una frontera
        for y in range(height):
            for x in range(width):
                index = y * width + x
                if msg.data[index] == 100:  # Celda ocupada
                    point = Point()
                    point.x = x * resolution  # Convertir a coordenadas reales
                    point.y = y * resolution
                    point.z = 0.0  # Altura del punto, si es un mapa 2D
                    marker.points.append(point)

                    # Agregar depuración para las coordenadas
                    self.get_logger().info(f"Coordenada (x, y): ({point.x}, {point.y})")

        # Publicar el marcador en RViz
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ProjectedMapSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
