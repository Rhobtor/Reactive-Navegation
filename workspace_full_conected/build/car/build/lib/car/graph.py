#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import networkx as nx
import matplotlib.pyplot as plt
import math
import time

class GroundGraphBuilder(Node):
    def __init__(self):
        super().__init__('ground_graph_builder')
        # Suscribirse al topic que publica los puntos del suelo
        self.subscription = self.create_subscription(
            PoseArray,
            'ground_points',
            self.points_callback,
            10
        )
        self.subscription  # evitar warning de variable no usada

        # Parámetro: distancia máxima para conectar dos puntos (en metros)
        self.declare_parameter("connection_threshold", 1.0)
        self.connection_threshold = self.get_parameter("connection_threshold").value

        # Grafo vacío para acumular los puntos
        self.G = nx.Graph()

        # Configurar matplotlib en modo interactivo
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.get_logger().info("GroundGraphBuilder iniciado. Se acumularán puntos durante 10 segundos.")

        # Variables para controlar el tiempo de actualización
        self.start_time = time.time()
        self.active = True
        # Timer para revisar el tiempo cada 0.1 s
        self.timer = self.create_timer(0.1, self.check_time)

    def check_time(self):
        # Si han pasado más de 10 segundos y aún estamos activos, finaliza la actualización
        if time.time() - self.start_time > 10.0 and self.active:
            self.active = False
            self.get_logger().info("10 segundos transcurridos. Grafo finalizado.")
            self.draw_and_save_graph(final=True)
            self.timer.cancel()  # Detener el timer

    def points_callback(self, msg: PoseArray):
        if not self.active:
            return  # Ignorar nuevos mensajes una vez finalizado el período
        # En este ejemplo, asumimos que el mensaje PoseArray contiene la lista completa de puntos
        # hasta el momento. Se reconstruye el grafo con esos puntos.
        self.G.clear()
        for idx, pose in enumerate(msg.poses):
            # Agregar nodo con la posición (x, y)
            self.G.add_node(idx, pos=(pose.position.x, pose.position.y))
        # Conectar nodos: para cada par, si la distancia es menor o igual al umbral, se añade una arista.
        nodes = list(self.G.nodes(data=True))
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                pos_i = nodes[i][1]['pos']
                pos_j = nodes[j][1]['pos']
                dist = math.sqrt((pos_i[0] - pos_j[0])**2 + (pos_i[1] - pos_j[1])**2)
                if dist <= self.connection_threshold:
                    self.G.add_edge(nodes[i][0], nodes[j][0])
        # Actualizar visualización en modo interactivo
        self.draw_and_save_graph(final=False)

    def draw_and_save_graph(self, final=False):
        # Obtener posiciones de los nodos
        pos = nx.get_node_attributes(self.G, 'pos')
        self.ax.clear()
        nx.draw(self.G, pos, with_labels=True, node_color='skyblue', node_size=500,
                edge_color='r', font_size=10, ax=self.ax)
        self.ax.set_title("Grafo de puntos del suelo (ventana de 10s)")
        plt.draw()
        plt.pause(0.001)
        if final:
            filename = 'ground_graph_final.png'
            plt.savefig(filename)
            self.get_logger().info(f"Grafo final guardado en {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = GroundGraphBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
