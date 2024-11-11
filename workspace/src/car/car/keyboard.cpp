#include <ros/ros.h>                    // Incluye la biblioteca de ROS para manejar nodos y tópicos
#include <geometry_msgs/Twist.h>        // Mensaje Twist para comandos de velocidad
#include <termios.h>                    // Biblioteca para manipular entrada de teclado en modo sin bloqueo
#include <unistd.h>                     // Biblioteca para operaciones de entrada/salida

// Función para leer una tecla presionada sin esperar a que el usuario presione "Enter"
char getKey() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);     // Guarda la configuración actual del terminal
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);   // Configura el terminal en modo sin bloqueo y sin eco
    newt.c_cc[VMIN] = 1;                // Mínimo número de caracteres de entrada
    newt.c_cc[VTIME] = 0;               // Tiempo de espera en decisegundos
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Aplica la nueva configuración
    ch = getchar();                     // Lee un solo carácter de la entrada estándar
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restaura la configuración original del terminal
    return ch;                          // Retorna el carácter presionado
}

int main(int argc, char** argv) {
    // Inicializa el nodo de ROS llamado "teleop_keyboard"
    ros::init(argc, argv, "teleop_keyboard");
    ros::NodeHandle nh;

    // Crea un publicador en el tópico "/cmd_vel" que enviará mensajes de tipo Twist
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    geometry_msgs::Twist msg;            // Crea un mensaje Twist para comandos de velocidad
    char key;                            // Variable para almacenar la tecla presionada

    ROS_INFO("Use WASD to control the car."); // Muestra instrucciones en la consola

    // Bucle principal para verificar las teclas presionadas y mover el robot
    while (ros::ok()) {
        key = getKey();                  // Llama a getKey() para obtener la tecla presionada

        // Switch para interpretar cada tecla y asignar velocidades al mensaje Twist
        switch (key) {
            case 'w':                    // Tecla 'w': avanzar
                msg.linear.x = 0.5;      // Velocidad lineal en eje X (avanza)
                msg.angular.z = 0.0;     // Sin giro
                break;
            case 's':                    // Tecla 's': retroceder
                msg.linear.x = -0.5;     // Velocidad lineal negativa en eje X (retrocede)
                msg.angular.z = 0.0;     // Sin giro
                break;
            case 'a':                    // Tecla 'a': girar a la izquierda
                msg.linear.x = 0.0;      // Sin movimiento en línea recta
                msg.angular.z = 0.5;     // Velocidad angular en eje Z (gira a la izquierda)
                break;
            case 'd':                    // Tecla 'd': girar a la derecha
                msg.linear.x = 0.0;      // Sin movimiento en línea recta
                msg.angular.z = -0.5;    // Velocidad angular negativa en eje Z (gira a la derecha)
                break;
            case 'q':                    // Tecla 'q': salir del programa
                ros::shutdown();         // Cierra el nodo de ROS
                break;
            default:                     // Cualquier otra tecla: detener el robot
                msg.linear.x = 0.0;      // Velocidad lineal cero
                msg.angular.z = 0.0;     // Velocidad angular cero
                break;
        }

        pub.publish(msg);                // Publica el mensaje Twist con las velocidades actuales
        ros::spinOnce();                 // Mantiene el nodo en ejecución y procesando
    }

    return 0;
}
