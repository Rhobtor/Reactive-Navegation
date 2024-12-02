
import numpy as np
from test import PatrollingGraphRoutingProblem
import random
import networkx as nx
import matplotlib.pyplot as plt

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
        # Obtener las posiciones de los puntos transitables (valor 1)
    valid_positions = np.column_stack(np.where(navigation_map == 1))

    # Seleccionar un punto aleatorio entre las posiciones válidas
    random_point = valid_positions[np.random.randint(len(valid_positions))]
    return navigation_map, height_map, random_point

def create_test_path(graph, initial_position, steps=10, max_distance=10, max_slope=1):
    """
    Crea una trayectoria de prueba para el agente, asegurándose de que los movimientos
    solo ocurran entre nodos cercanos y con pendiente razonable.

    Args:
        graph (networkx.Graph): El grafo que representa el mapa.
        initial_position (tuple): Las coordenadas del nodo inicial (x, y).
        steps (int): El número de pasos en el camino.
        max_distance (float): La distancia máxima entre nodos vecinos que se considera válida.
        max_slope (float): La pendiente máxima que se considera válida para el movimiento.

    Returns:
        dict: Un diccionario con el camino seguido por el agente.
    """
    path = {0: []}
    current_position = initial_position
    visited_nodes = set()
    visited_nodes.add(current_position)

    for _ in range(steps):
        neighbors = list(graph.neighbors(current_position))
        if not neighbors:
            break

        # Filtrar vecinos que están dentro del alcance máximo de distancia y pendiente
        valid_neighbors = []
        for neighbor in neighbors:
            # Calcular la distancia y pendiente entre los nodos
            distance = np.linalg.norm(np.array(current_position) - np.array(neighbor))
            slope = abs(graph.nodes[current_position]['height'] - graph.nodes[neighbor]['height'])
            
            # Solo considerar vecinos cercanos y con pendiente razonable
            if distance <= max_distance and slope <= max_slope:
                valid_neighbors.append(neighbor)

        if valid_neighbors:
            # Da preferencia a los vecinos no visitados
            unvisited_neighbors = [neighbor for neighbor in valid_neighbors if neighbor not in visited_nodes]
            if unvisited_neighbors:
                next_position = random.choice(unvisited_neighbors)
            else:
                next_position = random.choice(valid_neighbors)  # Vuelve a un nodo ya visitado si es necesario

            path[0].append(next_position)
            visited_nodes.add(next_position)
            current_position = next_position
        else:
            # Si no hay vecinos válidos, el camino se detiene
            break

    return path


 
def display_node_properties(problem):
    """
    Muestra las propiedades de cada nodo en el grafo, incluyendo posición, altura y conexiones.
    """
    for node, attributes in problem.nodes(data=True):
        print(f"Nodo {node}:")
        print(f"  Posición: {attributes['position']}")
        print(f"  Altura: {attributes['height']}")
        print(f"  Vecinos: {list(problem.neighbors(node))}")
        print("")

# Función para visualizar los tres mapas
def plot_maps(navigation_map, height_map, importance_map):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    # Mapa de navegación
    axes[0].imshow(navigation_map, cmap='Greens', interpolation='nearest')
    axes[0].set_title('Mapa de Navegación')
    axes[0].set_xticks([])
    axes[0].set_yticks([])
    
    # Mapa de altura
    axes[1].imshow(height_map, cmap='viridis', interpolation='nearest')
    axes[1].set_title('Mapa de Altura')
    axes[1].set_xticks([])
    axes[1].set_yticks([])
    
    # Mapa de importancia
    axes[2].imshow(importance_map, cmap='coolwarm', interpolation='nearest')
    axes[2].set_title('Mapa de Importancia')
    axes[2].set_xticks([])
    axes[2].set_yticks([])
    
    plt.tight_layout()
    plt.show()

def select_random_point(navigation_map):
    """
    Selecciona un punto aleatorio en el mapa de navegación donde sea transitable (valor 1).
    """
    # Obtener las posiciones de los puntos transitables (valor 1)
    valid_positions = np.column_stack(np.where(navigation_map == 1))

    # Seleccionar un punto aleatorio entre las posiciones válidas
    random_point = valid_positions[np.random.randint(len(valid_positions))]

    return random_point  # Devuelve las coordenadas del punto seleccionado

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


    navigation_map = np.genfromtxt('../maps/output/heightmap_traversability.txt', delimiter=' ')
    high_map= np.genfromtxt('../maps/output/heightmap_z_values.txt', delimiter=' ')
    N_agents = 1
    initial_positions = np.array([10,20,30,40])[:N_agents]
	#final_positions = np.genfromtxt('../maps/output/interest_points.txt', delimiter=' ')
    final_positions = np.array([40,20,30,40])[:N_agents]
    
    scale = 40

    destination = select_random_point(navigation_map)
    plt.imshow(navigation_map, cmap='gray', interpolation='nearest')
    plt.scatter(destination[1], destination[0], color='red', label="Destino")
    plt.title("Mapa de Navegación con Destino Aleatorio")
    plt.colorbar()
    plt.legend()
    plt.show()
    # Inicializar el problema
    problem = PatrollingGraphRoutingProblem(
        navigation_map=navigation_map,
        high_map=high_map,
        importance_map=destination,
        scale=scale,
        n_agents=n_agents,
        max_distance=max_distance,
        initial_positions=initial_positions,
        final_positions=final_positions
    )

    # Crear una ruta de prueba
    test_path = create_test_path(problem.G, initial_positions[0], steps=10, max_distance=10, max_slope=1)
    print(test_path)
    display_node_properties(problem.G)
    # Posiciones para dibujar el grafo
# Crear el subgrafo del camino
    path_edges = [(test_path[0][i], test_path[0][i + 1]) for i in range(len(test_path[0]) - 1)]
    subgraph = problem.G.edge_subgraph(path_edges).copy()

    # Visualizar el grafo y el camino


    pos = nx.spring_layout(problem.G)  # Layout del grafo completo
    nx.draw(problem.G, pos, with_labels=True, node_size=300, node_color="lightgrey", edge_color="lightgrey")
    nx.draw(subgraph, pos, with_labels=True, node_size=500, node_color="lightblue", edge_color="blue")
    plt.show()
    

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