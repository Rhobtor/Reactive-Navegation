import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import numpy as np
import matplotlib.pyplot as plt

class Octomap2DDisplay(Node):
    def __init__(self):
        super().__init__('octomap_2d_display')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/occupied_cells_vis_array',  # Tópico a suscribirse
            self.callback,
            10)
        self.fig, self.ax = plt.subplots()
        plt.ion()  # Habilitar modo interactivo para actualizaciones en tiempo real

        # Configuración de altura mínima y máxima (ajustar según tu entorno)
        self.min_height = 0.1  # Altura mínima (en metros) para considerar un objeto
        self.max_height = 2.0  # Altura máxima (en metros) para evitar ruido alto

    def callback(self, msg):
        # Procesar los datos del MarkerArray para obtener puntos en 2D, filtrando el suelo
        points = []
        for marker in msg.markers:
            for point in marker.points:
                if self.min_height <= point.z <= self.max_height:  # Filtrar según Z
                    points.append([point.x, point.y])
        
        if not points:
            self.get_logger().warn("No valid markers received.")
            return
        
        points = np.array(points)
        
        # Proyectar puntos en un gráfico 2D
        self.ax.clear()
        self.ax.scatter(points[:, 0], points[:, 1], s=1, c='blue')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('2D Octomap Display (Objects Only)')
        plt.pause(0.01)  # Actualizar el gráfico en tiempo real

def main(args=None):
    rclpy.init(args=args)
    node = Octomap2DDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    plt.close()

if __name__ == '__main__':
    main()
