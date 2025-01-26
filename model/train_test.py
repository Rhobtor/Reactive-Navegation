from gazeob_test import GazeboTest
import rclpy
from rclpy.node import Node


def main():
    rclpy.init()  # Inicializa el sistema ROS 2

    launch_file = "/home/rhobtor/reactive/Reactive-Navegation/workspace_full_conected/src/car_python/launch/gazebo_no_gui.launch.py"
    
    # Crea un nodo GazeboTest y ejecuta la simulación
    test = GazeboTest(launch_file)

    # Aquí puedes agregar más lógica, como suscripciones a tópicos, servicios, etc.

    rclpy.spin(test)  # Ejecuta el nodo ROS 2 (permite que el nodo mantenga la comunicación activa)

    # Cierra el nodo y ROS 2 correctamente
    test.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()