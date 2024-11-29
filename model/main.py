
import numpy as np
from test import PatrollingGraphRoutingProblem


def generate_sample_maps(size, resolution):
    """
    Genera mapas de navegación, altura e importancia de ejemplo.
    
    Args:
        size (tuple): Tamaño del mapa (alto, ancho).
        resolution (int): Resolución para escalar el mapa.
    
    Returns:
        tuple: Mapas de navegación, altura e importancia.
    """
    navigation_map = np.ones(size)  # Todo es transitable inicialmente
    height_map = np.random.rand(*size) * 2  # Alturas entre 0 y 2 metros
    importance_map = [np.random.rand(*size) for _ in range(3)]  # Tres capas de importancia
    return navigation_map, height_map, importance_map

def create_test_path(graph, initial_position, steps=10):
    """
    Genera una ruta de prueba en el grafo para el agente.
    
    Args:
        graph (networkx.Graph): Grafo de navegación.
        initial_position (int): Nodo inicial.
        steps (int): Número de pasos en la ruta.
    
    Returns:
        dict: Rutas para los agentes (multiagente).
    """
    path = {0: []}  # Asumimos un solo agente (ID=0)
    current_position = initial_position

    for _ in range(steps):
        neighbors = list(graph.neighbors(current_position))
        if not neighbors:
            break
        next_position = np.random.choice(neighbors)
        path[0].append(next_position)
        current_position = next_position
    
    return path

def main():
    # Parámetros del entorno
    map_size = (50, 50)  # Tamaño del mapa
    scale = 5  # Resolución del grafo
    n_agents = 1  # Número de agentes
    max_distance = 100  # Distancia máxima (batería)
    initial_positions = np.array([0])  # Nodo inicial
    final_positions = np.array([500])  # Nodo final hipotético
    # Inicialización anterior...
    agent_states = {
        agent_id: {
            "position": initial_positions[agent_id],
            "distance_traveled": 0,
            "inclination": 0,
            "orientation": 0,  # Inicialmente sin giro
            "perception": {}  # Información observable
        }
        for agent_id in range(n_agents)
    }


    # navigation_map = np.genfromtxt('../maps/output/heightmap_traversability.txt', delimiter=' ')
    # high_map= np.genfromtxt('../maps/output/heightmap_z_values.txt', delimiter=' ')
    # N_agents = 1
    # initial_positions = np.array([10,20,30,40])[:N_agents]
	# #final_positions = np.genfromtxt('../maps/output/interest_points.txt', delimiter=' ')
    # final_positions = np.array([40,20,30,40])[:N_agents]
    
    # scale = 40



    # Generar mapas
    navigation_map, height_map, importance_map = generate_sample_maps(map_size, scale)

    # Inicializar el problema
    problem = PatrollingGraphRoutingProblem(
        navigation_map=navigation_map,
        high_map=height_map,
        importance_map=importance_map,
        scale=scale,
        n_agents=n_agents,
        max_distance=max_distance,
        initial_positions=initial_positions,
        final_positions=final_positions
    )

    # Crear una ruta de prueba
    test_path = create_test_path(problem.G, initial_positions[0], steps=15)

    # Evaluar la ruta
    rewards = problem.evaluate_path(test_path, agent_states,render=True)

    # Mostrar resultados
    print("Recompensas obtenidas:", rewards)
    # print("Camino seguido por el agente:", problem.waypoints[0])
    # # print("Contenido del grafo:")
    # # for node in problem.G.nodes:
    # #     print(f"Nodo {node}: {problem.G.nodes[node]}")

    # # Renderizar el camino final
    # problem.render()
    # print("Simulación completada.")

if __name__ == "__main__":
    main()