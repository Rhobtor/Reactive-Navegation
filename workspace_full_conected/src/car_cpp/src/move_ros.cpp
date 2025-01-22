#include "move_ros.hpp" 
 
move_ros::move_ros() : Node("move_ros")
{ 
    // INICIALIZACIONES 
    pos_actual = {0, 0, 0}; 
    orientacion_actual_q = {0, 0, 0, 0}; 
    orientacion_actual_E = 0; 
    orientacion_prueba = 0; 
    nueva_posicion_final = {}; 
    bool_odom = false; 
    octomap_recibido = false; 
    distancia_punto_destino={}; 
    distancia_anterior_punto_destino=1000; //inicializar con valor alto 
    angulo_punto_destino={}; 
    dif_angular=1; 
    no_avanzar=0; 
    evaluar_recta=0; 
    interaciones_ejecucion_totales=0; 
    punto_nuevo=0; 
    no_punto_valido=0; 
 
    // PARÁMETROS 
    radio_ext = 3;      //para la generación de puntos aleatorios, distancia máxima desde el centro del robot 
    radio_int = 0.7;    //para la generación de puntos aleatorios, radio interior central que obviamos por ser el robot (físicamente) 
    num_puntos = 20;    //numero de puntos aleatorios que creamos 
    distancia_calculo=0.25;  //intervalo entre un punto y otro de la recta generada entre punto origen y punto final 
    tolerancia_angular=0.01;  //valor de aproximación al angulo destino 
    tolerancia_distancia=0.5;  // valor de aproximación a la distancia destino 
    tolerancia_distancia_final=0.7;  // valor de aproximación a la distancia destino 
    radio_circunferencia=0.5; 
    numPuntos_circunferencia=10; 
    PuntoFIN3d = {0, 0, 0};  //Punto de destino 
      
    // SUBSCRIBERS AND PUBLISHERS 
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("octomap_topic", 10, std::bind(&move_ros::octomapCallback, this, std::placeholders::_1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry_topic", 10, std::bind(&move_ros::odometryCallback, this, std::placeholders::_1));
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
    visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 1);

} 
void move_ros::octomapCallback(const octomap_msgs::msg::Octomap::ConstPtr& octomap_msg)
{
    octomap_recibido = true;
    tree = octomap_msgs::msgToMap(*octomap_msg); 
    octree = dynamic_cast<octomap::OcTree*>(tree);
    obstacle_detected_p.set_mapa(octree);
}

void move_ros::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry_) {
    odometry = odometry_; // Suponiendo que `odometry` es un miembro de la clase
}
 
void move_ros::run () 
{     // Crear un ejecutor para manejar callbacks
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(this->shared_from_this());
    rclcpp::Rate rate(10.0);
    while (rclcpp::ok()) 
    {
        executor->spin_some(); // Procesar callbacks disponibles
        rate.sleep();          // Mantener la frecuencia
 
        std::cout << "INICIO"<< std::endl; 
        interaciones_ejecucion_totales = interaciones_ejecucion_totales+1; 
 
        inicio:
        // introducir punto destino 
        if(PuntoFIN3d.x()==0 && PuntoFIN3d.y()==0 && PuntoFIN3d.z()==0) 
        { 
            std::cout << "Introducir punto destino: " << std::endl; 
            std::cout << "Introduce posición x: "; 
            std::cin >> pos_fin_x; 
            std::cout <<std::endl; 
            std::cout << "Introduce posición y: "; 
            std::cin >> pos_fin_y; 
            std::cout <<std::endl;             
            pos_fin_z= 0.4; 
            PuntoFIN3d = {pos_fin_x, pos_fin_y, pos_fin_z};  //Punto de destino 
            punto_nuevo=1; 
        } 
        if (punto_nuevo==1) 
        {     
            do 
                {  
                    std::cout << "P: " << std::endl; 
                    executor->spin_some();

 
                    bool_odom =  odometry_p.set_puntoOdom(odometry,pos_actual,orientacion_actual_q); 
                    pos_actual3d= utilities_p.VectorToPoint3d(pos_actual); 
                    orientacion_actual_E = utilities_p.CuaternionToEulerAngles(orientacion_actual_q); 
                    distancia_punto_destino= utilities_p.calculateDistance(pos_actual3d, PuntoFIN3d); 
                    angulo_punto_destino = utilities_p.calculateAngle(pos_actual3d,PuntoFIN3d); 
                    dif_angular = angulo_punto_destino - orientacion_actual_E; 
                    while(dif_angular>M_PI) { dif_angular = dif_angular - 2*M_PI;} 
                    while(dif_angular<-M_PI) { dif_angular = dif_angular + 2*M_PI;} 
                    Comando_velocidad = motion_p.motionControlAngle(distancia_punto_destino,dif_angular); 
                    vel_pub_->publish(Comando_velocidad); 
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                }while (dif_angular < -tolerancia_angular || dif_angular > tolerancia_angular); 
                punto_nuevo=0; 
                no_punto_valido=0; 
        } 
         // Crear el MarkerArray
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker punto_visualizacion;
        //visualización punto destino 
        punto_visualizacion= visualization_p.PointColor(PuntoFIN3d,1000, 0.54, 0.17, 0.86); 


        // Add the single Marker to the MarkerArray
        marker_array.markers.push_back(punto_visualizacion);

        // Publish the MarkerArray
        visualization_pub_->publish(marker_array);
        
 
// 1. COGER EL PUNTO DE LA UBICACIÓN ACTUAL DEL ROBOT 
        executor->spin_some();  
        bool_odom = odometry_p.set_puntoOdom(odometry,pos_actual,orientacion_actual_q); 
        pos_actual3d= utilities_p.VectorToPoint3d(pos_actual); 
        orientacion_actual_E= utilities_p.CuaternionToEulerAngles(orientacion_actual_q); 
     
        if (bool_odom==true ) 
        { 
// 2. GENERAR PUNTOS (tipo octomap::point 3d) ALEATORIOS DENTRO DE UN RADIO DETERMINADO 
            no_avanzar = 0; 
            Vector_puntos_aleatorios.clear(); 
            Vector_distancias_pto_final.clear(); 
            vector_circunferencePoints.clear(); 
            Vector_puntos_aleatorios.resize(num_puntos); 
            Vector_distancias_pto_final.resize(num_puntos); 
            vector_circunferencePoints.resize(numPuntos_circunferencia); 
 
            for (int iter=0; iter<num_puntos; iter++) 
            { 
                //GENERAR PUNTO ALEATORIO 
                std::cout << "Posición actual:  "<< pos_actual[0] << " " << pos_actual[1] << " " << pos_actual[2] << " " << std::endl; 
                std::cout << "Orientación actual: "<< orientacion_actual_E << std::endl; 
                RandomPoint = utilities_p.generateRandomPoint(pos_actual,orientacion_actual_E,radio_ext,radio_int); 
                RandomPoint3d = utilities_p.VectorToPoint3d(RandomPoint); 
                std::cout << "Punto aleatorio3d:  "<< RandomPoint3d.x() << " " << RandomPoint3d.y() << " " << RandomPoint3d.z() << "   "; 
 
// 3. VER SI EL PUNTO ALEATORIO ES VÁLIDO 
                //VER SI ESTE PUNTO ESTA OCUPADO 
                occupation = obstacle_detected_p.obstaculo_cerca(RandomPoint3d); 
                std::cout << "Ocupación del punto aleatorio:  "<< occupation << std::endl; 
 
                //SI NO ESTÁ OCUPADO VER SI HAY ALGÚN PUNTO OCUPADO EN LA CIRCUNFERENCIA DE ALREDEDOR 
                // SI ESTÁ OCUPADO VACIAR 
                if (occupation==false) 
                { 
                    vector_circunferencePoints.clear(); 
                    vector_circunferencePoints = utilities_p.createCircunferencePoints3d(RandomPoint3d,radio_circunferencia,numPuntos_circunferencia); 
                    for (int v1=0; v1<vector_circunferencePoints.size(); v1++) 
                    { 
                        //std::cout << "t4" << std::endl; 
                        circunference_occupation = obstacle_detected_p.obstaculo_cerca(vector_circunferencePoints[v1]); 
                        if (circunference_occupation==true) 
                        { 
                            std::cout << "Punto ocupado en la circunferencia cercana" << std::endl; 
 
                            //visualizacion de puntos (rojo) 
                            punto_visualizacion= visualization_p.PointColor(RandomPoint3d,iter,1,0,0);
                            // Add the single Marker to the MarkerArray
                            marker_array.markers.push_back(punto_visualizacion);

                            // Publish the MarkerArray
                            visualization_pub_->publish(marker_array); 
                        
 
                            Vector_puntos_aleatorios[iter] = {}; 
                            evaluar_recta=0; 
                            break; 
                        } 
                        else 
                        { 
                            evaluar_recta=1; 
                        } 
                    } 
                    if (evaluar_recta==1) 
                    { 
                        //SI NO ESTÁ OCUPADO VER SI HAY ALGÚN PUNTO OCUPADO EN LA LÍNEA RECTA QUE LOS UNE 
                        // SI ESTÁ OCUPADO VACIAR 
                        //RESETEAMOS Y CREAMOS EL VECTOR DE PUNTOS INTERMEDIOS 
                        vector_intermediatePoints.clear(); 
                        vector_intermediatePoints = utilities_p.createVectorIntermediatePoints3d(pos_actual3d,RandomPoint3d,distancia_calculo); 
 
                        // MIRAR CADA PUNTO DE ESE VECTOR PARA. SI OCUPADO VACIAMOS Y SALIMOS DEL BUCLE SINO METEMOS EL PUNTO EN EL VECTOR   
                        for (int v=0; v<vector_intermediatePoints.size(); v++) 
                        { 
                            inter_occupation = obstacle_detected_p.obstaculo_cerca(vector_intermediatePoints[v]); 
                            if (inter_occupation==true) 
                            { 
                                //visualizacion de puntos (rojo) 
                                punto_visualizacion= visualization_p.PointColor(RandomPoint3d,iter,1,0,0); 
                                // Add the single Marker to the MarkerArray
                                marker_array.markers.push_back(punto_visualizacion);

                                // Publish the MarkerArray
                                visualization_pub_->publish(marker_array);
                                
 
                                Vector_puntos_aleatorios[iter] = {}; 
                                break; 
                            } 
                            else 
                            { 
                                //visualizacion de puntos (celeste) 
                                punto_visualizacion= visualization_p.PointColor(RandomPoint3d,iter,0.70,0.70,0.70);
                                // Add the single Marker to the MarkerArray
                                marker_array.markers.push_back(punto_visualizacion);

                                // Publish the MarkerArray
                                visualization_pub_->publish(marker_array); 
                                
 
                                Vector_puntos_aleatorios[iter] = RandomPoint3d; 
                            } 
                        } 
                    } 
                } 
                else 
                { 
                    Vector_puntos_aleatorios[iter] = {}; 
                    Vector_distancias_pto_final[iter] = {}; 
                     
                    //visualizacion de puntos (cambio a rojo) 
                    punto_visualizacion= visualization_p.PointColor(RandomPoint3d,iter,1,0,0);
                    // Add the single Marker to the MarkerArray
                    marker_array.markers.push_back(punto_visualizacion);

                    // Publish the MarkerArray
                    visualization_pub_->publish(marker_array); 

 
                    
                } 
 
// 4. SI EL PUNTO ALEATORIO ES VÁLIDO CALCULAR LA DISTANCIA DESDE EL PUNTO ALEATORIO AL FINAL 
                // SI EL PUNTO ALEATORIO ES VÁLIDO (NO OCUPADO Y LINEA QUE LO UNE NO OCUPADA) CALCULAR DISTANCIA 
                // SI NO, PONER O 
                if (Vector_puntos_aleatorios[iter](0) != 0 && Vector_puntos_aleatorios[iter](1) != 0) 
                { 
                    distancia_pto_final = utilities_p.calculateDistance(RandomPoint3d,PuntoFIN3d); 
                    Vector_distancias_pto_final[iter] = distancia_pto_final; 
                } 
                else  
                { 
                    Vector_distancias_pto_final[iter] = {}; 
                } 
            rclcpp::sleep_for(std::chrono::milliseconds(50)); 
            } 
            std::cout << "vector creado "<< std::endl; 
                        for (int t5=0;  t5<Vector_puntos_aleatorios.size(); t5++) 
                                { 
                                    std::cout << "vector puntos aleatorios resultante  " << t5 << " "; 
                                    std::cout << "(" << Vector_puntos_aleatorios[t5].x() << ", " << Vector_puntos_aleatorios[t5].y() << ", " << Vector_puntos_aleatorios[t5].z() << ")   " << Vector_distancias_pto_final[t5] << std::endl; 
                                }   
            std::cout << "  "<< std::endl; 
 
// 5. COMPARAR DISTANCIAS Y ELEGIR LA MÁS CERCANA AL PUNTO FINAL 
            indice_min_distancia = utilities_p.minimumValueIndex(Vector_distancias_pto_final); 
            std::cout << "indice distancia mínima "<< indice_min_distancia << std::endl; 
             
            //visualizacion de puntos (verde) 
            punto_visualizacion= visualization_p.PointColor(Vector_puntos_aleatorios[indice_min_distancia],indice_min_distancia,0,1,0);
            // Add the single Marker to the MarkerArray
            marker_array.markers.push_back(punto_visualizacion);

            // Publish the MarkerArray
            visualization_pub_->publish(marker_array); 
            
 
            linea_visualizacion = visualization_p.createLineMarker(pos_actual3d,Vector_puntos_aleatorios[indice_min_distancia],interaciones_ejecucion_totales); 
            // Add the Marker to the MarkerArray
            marker_array.markers.push_back(linea_visualizacion);

            // Publish the MarkerArray
            visualization_pub_->publish(marker_array);

 
             
            if (indice_min_distancia == -1) 
            { 
                std::cout << "NO HAY NINGÚN PUNTO VÁLIDO" << std::endl; 
                bool_odom = odometry_p.set_puntoOdom(odometry,pos_actual,orientacion_actual_q); 
                orientacion_actual_E = utilities_p.CuaternionToEulerAngles(orientacion_actual_q); 
                orientacion_prueba = orientacion_actual_E + 1.5; 
                do 
                {  
                    executor->spin_some();
                    bool_odom = odometry_p.set_puntoOdom(odometry,pos_actual,orientacion_actual_q); 
                    orientacion_actual_E = utilities_p.CuaternionToEulerAngles(orientacion_actual_q); 
                    dif_angular = orientacion_prueba - orientacion_actual_E; 
                    while(dif_angular>M_PI) { dif_angular = dif_angular - 2*M_PI;} 
                    while(dif_angular<-M_PI) { dif_angular = dif_angular + 2*M_PI;} 
                    Comando_velocidad = motion_p.motionControlAngle(distancia_punto_destino,dif_angular); 
                    vel_pub_->publish(Comando_velocidad);  
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                }while (dif_angular < -tolerancia_angular || dif_angular > tolerancia_angular); 
                punto_nuevo=0; 
                no_punto_valido=0; 
                no_punto_valido=1; 
                //PuntoFIN3d = {0, 0, 0};  
                goto inicio; 
            } 
         
// 6. COMANDAR LA VELOCIDAD 
            //CALCULO PREVIO DE DISTANCIA Y ÁNGULO AL PUNTO DESTINO (RandomPoint3d elegido) 
            
            pos_destino3d = Vector_puntos_aleatorios[indice_min_distancia]; 
            std::cout << "(" << pos_destino3d.x() << ", " << pos_destino3d.y() << ", " << pos_destino3d.z() << ")" << std::endl; 
             
            bool_odom = odometry_p.set_puntoOdom(odometry,pos_actual,orientacion_actual_q); 
            pos_actual3d= utilities_p.VectorToPoint3d(pos_actual); 
            orientacion_actual_E = utilities_p.CuaternionToEulerAngles(orientacion_actual_q); 
             
            distancia_punto_destino = utilities_p.calculateDistance(pos_actual3d, pos_destino3d); 
            angulo_punto_destino = utilities_p.calculateAngle(pos_actual3d,pos_destino3d); 

            dif_angular = angulo_punto_destino - orientacion_actual_E; 
            while(dif_angular>M_PI) { dif_angular = dif_angular - 2*M_PI;} 
            while(dif_angular<-M_PI) { dif_angular = dif_angular + 2*M_PI;} 
            std::cout << "distancia  " << distancia_punto_destino << "  angulo  " << angulo_punto_destino << " dif angular:  " << dif_angular << std::endl; 
            std:: cout << " orient angular " << orientacion_actual_E << std::endl; 
            std:: cout << " dif_angular" << dif_angular << std::endl; 
 
            // PRIMERO COMANDO EL GIRO  
            while (dif_angular < -tolerancia_angular || dif_angular > tolerancia_angular) 
            { 
                std::cout << "dif_angular " << dif_angular << std::endl; 
                //LEER DEL TOPIC Y CONVERTIR EL CUATERNIO A EULER 
                executor->spin_some();
                std::cout << "MOVIMIENTO ANGULO"; 
                std::cout << "(" << pos_destino3d.x() << ", " << pos_destino3d.y() << ", " << pos_destino3d.z() << ")" << std::endl; 
                bool_odom = odometry_p.set_puntoOdom(odometry,pos_actual,orientacion_actual_q); 
                pos_actual3d= utilities_p.VectorToPoint3d(pos_actual); 
                orientacion_actual_E = utilities_p.CuaternionToEulerAngles(orientacion_actual_q); 
                std::cout << "Orientacion actual Euler: " << orientacion_actual_E << std::endl; 
 
                // CALCULAR LA DISTANCIA Y EL ÁNGULO QUE DEBE GIRAR 
                distancia_punto_destino= utilities_p.calculateDistance(pos_actual3d, pos_destino3d); 
                angulo_punto_destino = utilities_p.calculateAngle(pos_actual3d,pos_destino3d); 
                std::cout << "distancia  " << distancia_punto_destino << "  angulo  " << angulo_punto_destino << std::endl; 
 
                // VER LA DIFERENCIA DEL ANGULO ACTUAL Y ANGULO FINAL  
                dif_angular = angulo_punto_destino - orientacion_actual_E; 
                while(dif_angular>M_PI) { dif_angular = dif_angular - 2*M_PI;} 
                while(dif_angular<-M_PI) { dif_angular = dif_angular + 2*M_PI;} 
                std::cout << "dif_angular" << dif_angular << std::endl; 
 
                 
                Comando_velocidad = motion_p.motionControlAngle(distancia_punto_destino,dif_angular); 
                std::cout << "Mensaje Twist " << Comando_velocidad.linear.x << "  " << Comando_velocidad.angular.z << std::endl; 
                 
                std::ofstream archivo_csv; 
                archivo_csv.open("/home/violeta/catkin_ws/velocidad.csv", std::ofstream::out | std::ofstream::app); 

 
                archivo_csv << " giro: "<< Comando_velocidad.angular.z << std::endl; 
 
                // COMANDAR LA VELOCIDAD ANGULAR 
                vel_pub_->publish(Comando_velocidad);  
 
                //visualization_pub_.publish(punto_visualizacion);  
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            } 
 
            std::cout << "Distancia punto destino: " << distancia_punto_destino << "  no_avanzar: " <<  no_avanzar <<std::endl; 
            distancia_anterior_punto_destino=1000; //inicializar con un valor alto 
             
            // DESPUES COMANDO DE AVANCE 
            while (distancia_punto_destino>tolerancia_distancia && no_avanzar==0) 
            { 
                // UNA VEZ QUE HEMOS GIRADO VERIFICAR SI EL PUNTO SIGUE LIBRE Y SI LA RECTA QUE LOS UNE TAMBIÉN 
                occupation = obstacle_detected_p.obstaculo_cerca(pos_destino3d); 
                std::cout << "Ocupación del punto destino:  "<< occupation << std::endl; 
 
                // SI NO ESTÁ OCUPADO VER SI HAY ALGÚN PUNTO OCUPADO EN LA LÍNEA RECTA QUE LOS UNE 
                // SI ESTÁ OCUPADO ACTIVAMOS "NO AVANZAR" 
                if (occupation==false) 
                {    
                    //RESETEAMOS Y CREAMOS LA CIRCUNFERENCIA DE ALREDEDOR DEL PUNTO 
                    vector_circunferencePoints.clear(); 
                    vector_circunferencePoints = utilities_p.createCircunferencePoints3d(RandomPoint3d,radio_circunferencia,numPuntos_circunferencia); 
 
                    for (int v1=0; v1<vector_circunferencePoints.size(); v1++) 
                    { 
                        std::cout << v1 << std::endl; 
                        inter_occupation = obstacle_detected_p.obstaculo_cerca(vector_circunferencePoints[v1]); 
                        if (inter_occupation==true) 
                        { 
                            no_avanzar = 1; 
                            std::cout << "Hay obstáculo no avanzar circunf" << std::endl; 
                            goto stop; 
                        } 
                    } 
                    if (no_avanzar!=1) 
                    { 
                        //RESETEAMOS Y CREAMOS EL VECTOR DE PUNTOS INTERMEDIOS 

 
                        vector_intermediatePoints.clear(); 
                        vector_intermediatePoints = utilities_p.createVectorIntermediatePoints3d(pos_actual3d,pos_destino3d,distancia_calculo); 
 
                        // MIRAR CADA PUNTO DE ESE VECTOR PARA. SI OCUPADO ACTIVAMOS "NO AVANZAR". 
                        for (int v=0; v<vector_intermediatePoints.size(); v++) 
                        { 
                            inter_occupation = obstacle_detected_p.obstaculo_cerca(vector_intermediatePoints[v]); 
                            if (inter_occupation==true) 
                            { 
                                no_avanzar = 1; 
                                std::cout << "Hay obstáculo no avanzar" << std::endl; 
                                goto stop; 
                            } 
                        } 
                    } 
                } 
                else 
                { 
                    std::cout << "Hay obstáculo no avanzar" << std::endl; 
                    no_avanzar = 1; 
                    goto stop; 
                } 
 
                //LEER DEL TOPIC Y CONVERTIR EL PUNTO A PUNTO3D (OCTOMAP) 
                executor->spin_some();
                std::cout << "MOVIMIENTO DISTANCIA"; 
                std::cout << "(" << pos_destino3d.x() << ", " << pos_destino3d.y() << ", " << pos_destino3d.z() << ")" << std::endl; 
                bool_odom = odometry_p.set_puntoOdom(odometry,pos_actual,orientacion_actual_q); 
                pos_actual3d= utilities_p.VectorToPoint3d(pos_actual); 
                orientacion_actual_E = utilities_p.CuaternionToEulerAngles(orientacion_actual_q); 
                std::cout << "Posición actual:  "<< pos_actual3d(0) << " " << pos_actual3d(1) << " " << pos_actual3d(2) << " " << std::endl; 
                 
                // CALCULAR LA DISTANCIA Y EL ÁNGULO QUE DEBE GIRAR  
                distancia_punto_destino = utilities_p.calculateDistance(pos_actual3d, pos_destino3d); 
                angulo_punto_destino = utilities_p.calculateAngle(pos_actual3d,pos_destino3d); 
                dif_angular = angulo_punto_destino - orientacion_actual_E; 
                std::cout << "dif_angular" << dif_angular << std::endl; 
                std::cout << "distancia  " << distancia_punto_destino << "  angulo  " << angulo_punto_destino << std::endl; 

 
 
                // COMPARAR CON LA DISTANCIA ANTERIOR PARA VERIFICAR QUE NO SE ESTÁ ALEJANDO 
                if (distancia_punto_destino > distancia_anterior_punto_destino+0.001) 
                { 
                    std::cout << "SE ALEJA" << std::endl; 
                    goto stop; 
                } 
 
                distancia_anterior_punto_destino= distancia_punto_destino; 
                 
                Comando_velocidad = motion_p.motionControlDistance(distancia_punto_destino,dif_angular); 
                std::cout << "Mensaje Twist " << Comando_velocidad.linear.x << "  " << Comando_velocidad.angular.z << std::endl; 
                 
                // // exportación de datos al archivo csv 
                // std::ofstream archivo_csv; 
                // archivo_csv.open("/home/violeta/catkin_ws/velocidad.csv", std::ofstream::out | std::ofstream::app); 
                // archivo_csv << " avance: "<< Comando_velocidad.linear.x<< std::endl; 
                 
                // COMANDAR LA VELOCIDAD LINEAL 
                vel_pub_->publish(Comando_velocidad);  
                rclcpp::sleep_for(std::chrono::milliseconds(50));
                //std::cout << "wait " << std::endl; 
            } 
             
            stop: 
            //delete points  
            borrar_puntos = visualization_p.deleteAllPoints(); 
            //visualization_pub_.publish(borrar_puntos); 
 
            Comando_velocidad = motion_p.motionControlSTOP(); 
 
            // std::ofstream archivo_csv; 
            // archivo_csv.open("/home/violeta/catkin_ws/velocidad.csv", std::ofstream::out | std::ofstream::app); 
            // archivo_csv << " STOP "<< std::endl; 
 
            vel_pub_->publish(Comando_velocidad); 
            std::cout << "STOP " << std::endl; 
            std::cout << " ------------------------------------------------- "<< std::endl; 
            //ros::Duration(20.0).sleep(); 
 
            distancia_final = utilities_p.calculateDistance(pos_actual3d,PuntoFIN3d); 
            std::cout << "Disttancia hasta llegar al punto destino: "<< distancia_final << std::endl; 

 
            // esta parte hay que quitarla y poner la vuelta al punto de inicio como senal de rescate exitoso
            if (distancia_final<tolerancia_distancia_final) 
            { 
                std::cout << "Destino final alcanzado " << std::endl; 
                nueva_posicion_final = {}; 
                std::cout << "elección " << nueva_posicion_final << std::endl; 
 
                while (nueva_posicion_final!= 's' && nueva_posicion_final!= 'n') 
                { 
                    std::cout << "¿Desea introducir otro punto destino? [s/n] " << std::endl; 
                    std::cin >> nueva_posicion_final; 
                    std::cout << "elección " << nueva_posicion_final << std::endl; 
                } 
                if (nueva_posicion_final == 's') 
                { 
                    PuntoFIN3d = {0, 0, 0};  //Punto de destino 
                } 
                else if (nueva_posicion_final == 'n') 
                { 
                    break; 
                } 
            } 
        } 
        else {std::cout << "Odometry es un puntero nulo." << std::endl;} 
    } 
} 
 
move_ros::~move_ros(){}