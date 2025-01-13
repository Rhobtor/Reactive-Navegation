import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickMonitorNode(Node):
    def __init__(self):
        super().__init__('joystick_monitor')
        # Suscripción al tema /joy
        self.joy_subscriber = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info("Joystick Monitor Node iniciado. Esperando entradas...")

    def joy_callback(self, msg: Joy):
        # Imprime los valores de los ejes
        self.get_logger().info(f"Ejes: {msg.axes}")
        # Imprime los estados de los botones
        self.get_logger().info(f"Botones: {msg.buttons}")

def main(args=None):
    rclpy.init(args=args)
    joystick_monitor = JoystickMonitorNode()
    rclpy.spin(joystick_monitor)
    joystick_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
