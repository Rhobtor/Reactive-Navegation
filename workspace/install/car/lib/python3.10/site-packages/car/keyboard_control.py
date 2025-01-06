import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from pynput import keyboard

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Float64, '/wheel_torque_command', 10)
        self.torque = 0.0  
        self.get_logger().info("Use W/S/D to control the wheel torque")

        # Escucha al teclado (requiere pynput)
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
    
    def on_press(self, key):
        try:
            if key.char == 'w':
                self.torque = 1.0
                self.get_logger().info("Forward")
            elif key.char == 's':
                self.torque = -1.0
                self.get_logger().info("Backward")
            elif key.char == 'd':
                self.torque = 0.0
                self.get_logger().info("Stop")
            msg = Float64()
            msg.data = self.torque
            self.publisher.publish(msg)
        except AttributeError:
            pass
def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    rclpy.spin(keyboard_control)
    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()