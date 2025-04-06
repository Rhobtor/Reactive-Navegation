import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class OctomapResetter(Node):
    def __init__(self):
        super().__init__('octomap_resetter')
        self.reset_client = self.create_client(Empty, 'octomap_server/reset')
        # Esperar hasta que el servicio esté disponible.
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio reset de octomap_server...')
        # Configura un timer para reinicializar el octomap periódicamente, por ejemplo, cada 5 segundos.
        self.create_timer(5.0, self.reset_octomap)
        self.get_logger().info("Nodo OctomapResetter iniciado, reiniciando octomap cada 5 segundos.")

    def reset_octomap(self):
        # Aquí podrías agregar condiciones basadas en la detección de cambios en el entorno
        req = Empty.Request()
        self.reset_client.call_async(req)
        self.get_logger().info("Octomap reseteado.")

def main(args=None):
    rclpy.init(args=args)
    node = OctomapResetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
