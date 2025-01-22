#include "move_ros.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv); // Corrige a rclcpp
    auto node = std::make_shared<move_ros>(); // Usa std::shared_ptr para crear el nodo correctamente

    node->run(); // Llama a la función run en el nodo
    rclcpp::spin(node); // Hace que el nodo ejecute su ciclo de vida
    rclcpp::shutdown(); // Finaliza la inicialización de rclcpp
    return 0;
}
