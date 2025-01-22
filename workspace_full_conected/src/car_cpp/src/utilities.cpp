#include "utilities.hpp" 
 
utilities::utilities() 
{ 
}; 
// DE VECTOR A UNTO 3D 
octomap::point3d utilities::VectorToPoint3d (std::vector<double> vector){ 
    octoPoint3d = {vector[0],vector[1],vector[2]}; 
    return octoPoint3d; 
} 
 
// DE PUNTO 3D A VECTOR 
std::vector<double> utilities::Point3dToVector(octomap::point3d octoPoint3d){ 
    vector = {octoPoint3d(0), octoPoint3d(1), octoPoint3d(2)}; 
    return vector; 
} 
 
// GENERAR PUNTO ALEATORIO 2D (con offset z=0.4) 

 
std::vector<double> utilities::generateRandomPoint(std::vector<double> centro, double 
orientacion, double radio_ext, double radio_int){ 
                        //std::cout << "rand" << rand() << std::endl; 
    min_theta = orientacion - (1*M_PI/3); 
    max_theta = orientacion + (1*M_PI/3); 
 
    theta = min_theta + (static_cast<double>(rand()) / RAND_MAX) * (max_theta - min_theta); //modifico para que solo sean los 180º frontales 
    distance = sqrt(static_cast<double>(rand()) / RAND_MAX) * (radio_ext-radio_int)+radio_int;  //modifico para que excluya el circulo unidad central que es el propio robot 
 
    x_centro = centro[0] + distance * cos(theta); 
    y_centro = centro[1] + distance * sin(theta); 
 
    randomPoint = {x_centro,y_centro,0.4}; 
    return randomPoint; 
} 
 
// CALCULAR DISTANCIA 
double utilities::calculateDistance(octomap::point3d puntoA, octomap::point3d 
puntoB){ 
 
    diferenciaX = std::abs(puntoA(0) - puntoB(0)); 
    diferenciaY = std::abs(puntoA(1)- puntoB(1)); 
    diferenciaZ = std::abs(puntoA(2) - puntoB(2)); 
 
    distancia = sqrt(diferenciaX*diferenciaX + diferenciaY*diferenciaY + 
diferenciaZ*diferenciaZ); 
 
    return distancia; 
} 
 
// CALCULO DE VECTOR DE PUNTOS QUE FORMA LA RECTA QUE UNE DOS PUNTOS 
std::vector<octomap::point3d> 
utilities::createVectorIntermediatePoints3d(octomap::point3d puntoInicio3d, 
octomap::point3d puntoFin3d, double incremento){ 
 
    distanciaTotal = calculateDistance(puntoInicio3d,puntoFin3d); 
    cantidadPuntos = static_cast<int>(distanciaTotal/incremento); 
    intermediateVectorPoints3d.clear(); 
    for(int i = 0; i<=cantidadPuntos; i++) 
    { 
        double t = static_cast<double>(i)/cantidadPuntos; 
        intermediatePoint3d(0)= (puntoInicio3d(0)+ t*(puntoFin3d(0)-puntoInicio3d(0)))+0.5; 
        intermediatePoint3d(1)= puntoInicio3d(1)+ t*(puntoFin3d(1)-puntoInicio3d(1)); 
        intermediateVectorPoints3d.push_back(intermediatePoint3d); 
 
 
    } 
    for(int i = 0; i<=cantidadPuntos; i++) 
    { 
        double t = static_cast<double>(i)/cantidadPuntos; 
        intermediatePoint3d(0)= puntoInicio3d(0)+ t*(puntoFin3d(0)-puntoInicio3d(0)); 
        intermediatePoint3d(1)= puntoInicio3d(1)+ t*(puntoFin3d(1)-puntoInicio3d(1)); 
        intermediateVectorPoints3d.push_back(intermediatePoint3d); 
 
    } 
    for(int i = 0; i<=cantidadPuntos; i++) 
    { 
        double t = static_cast<double>(i)/cantidadPuntos; 
        intermediatePoint3d(0)= (puntoInicio3d(0)+ t*(puntoFin3d(0)-puntoInicio3d(0)))-0.5; 
        intermediatePoint3d(1)= puntoInicio3d(1)+ t*(puntoFin3d(1)-puntoInicio3d(1)); 
        intermediateVectorPoints3d.push_back(intermediatePoint3d); 
 
    } 
    return intermediateVectorPoints3d; 
} 
 
// CÁLCULO DE CIRCUNFERENCIA ALREDEDOR DEL PUNTO ELEGIDO 
std::vector<octomap::point3d> utilities::createCircunferencePoints3d(octomap::point3d 
centro, double radio, int numPuntos){ 
    vector_puntos_circunferencia.clear(); 
    for (int i = 0; i<numPuntos; i++) 
    { 
        angulo_circunferencia = 2*M_PI*i/numPuntos; 
        punto_circunferencia(0) = centro(0) + radio * cos(angulo_circunferencia); 
        punto_circunferencia(1) = centro(1) + radio * sin(angulo_circunferencia); 
        vector_puntos_circunferencia.push_back(punto_circunferencia); 
    }     
    return vector_puntos_circunferencia; 
} 
// BUSCAR EL VALOR MÍNIMO DE UN VECTOR Y DEVOLVER EL ÍNDICE 
int utilities::minimumValueIndex(std::vector<double> vectorDistances){ 
    if (vectorDistances.empty()) 
    { 
        std::cout << "Vector distancias is empty" << std::endl; 
        return -1; 
    } 
    minimumDistance = -1; 
    minimumIndex = -1; 
 
    // iniciar el valor de comparación con el primer valor no nulo del vector 
    for (int t=0; t<vectorDistances.size(); ++t)  
    { 
        if (vectorDistances[t] != 0)  
        { 

 
            minimumDistance = vectorDistances[t]; 
            minimumIndex = t; 
            break; 
        } 
    } 
     
    // si una vez buscado el primer valor no nulo el valor de mínima distancia sigue siendo 0 es porque no hay ningun punto válido 
    if (minimumDistance == 0) 
    { 
        return -1; 
    } 
    else 
    { 
        for (int i=1; i<vectorDistances.size(); ++i)  
        { 
            if (vectorDistances[i] < minimumDistance && vectorDistances[i] != 0)  
            { 
                minimumDistance = vectorDistances[i]; 
                minimumIndex = i; 
            } 
        } 
        return minimumIndex; 
    }    
} 
 
// CALCULO DEL ANGULO AL PUNTO FINAL (para comandar velocidad) 
double utilities::calculateAngle (octomap::point3d StartPoint3d, octomap::point3d FinalPoint3d)
{ 
    deltaX = (FinalPoint3d(0)-StartPoint3d(0)); 
    deltaY = (FinalPoint3d(1)-StartPoint3d(1)); 
    angle = atan2(deltaY,deltaX); 
 
    return angle; 
} 
 
// PASAR DE CUATERNIO A ANGULO DE EULER 
double utilities::CuaternionToEulerAngles (std::vector<double> cuaternion_orientation)
{ 
    qx = cuaternion_orientation[0]; 
    qy = cuaternion_orientation[1]; 
    qz = cuaternion_orientation[2]; 
    qw = cuaternion_orientation[3]; 
 
    Euler_Angle = atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz)); 
     
    return Euler_Angle; 
} 
utilities::~utilities(){} 