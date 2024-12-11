
import numpy as np
from test import PatrollingGraphRoutingProblem
import random
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib
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
            if (distance <= max_distance).all and (slope <= max_slope).all:
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

# def select_random_point(navigation_map):
#     """
#     Selecciona un punto aleatorio en el mapa de navegación donde sea transitable (valor 1).
#     """
#     # Obtener las posiciones de los puntos transitables (valor 1)
#     valid_positions = np.column_stack(np.where(navigation_map == 1))

#     # Seleccionar un punto aleatorio entre las posiciones válidas
#     random_point = valid_positions[np.random.randint(len(valid_positions))]

#     return random_point  # Devuelve las coordenadas del punto seleccionado



def select_random_node(navigation_map, graph):
    """
    Selecciona un nodo aleatorio del grafo basado en el mapa de navegación.
    
    - navigation_map: Mapa 2D de navegación con valores 1 para áreas transitables.
    - graph: Grafo con nodos que tienen atributos 'coords' o 'position'.
    
    Devuelve el índice del nodo seleccionado aleatoriamente.
    """
    # Obtener posiciones válidas del mapa (valor 1)
    valid_positions = np.column_stack(np.where(navigation_map == 1))

    # Seleccionar un punto aleatorio de las posiciones válidas
    random_point = valid_positions[np.random.randint(len(valid_positions))]

    # Escalar el punto seleccionado
    random_point_scaled = random_point[::-1]  # Si las coordenadas están en formato y,x

    # Encontrar el nodo correspondiente en el grafo
    for node, attrs in graph.nodes(data=True):
        if np.array_equal(attrs['coords'], random_point_scaled):  # Comparar coordenadas
            return node

    raise ValueError("No se encontró un nodo correspondiente a las coordenadas seleccionadas.")


def plot_graph(G: nx.Graph, path: list = None, ax=None, cmap_str='Reds', draw_nodes=True):

	if ax is None:
		plt.figure()
		ax = plt.gca()

	positions = nx.get_node_attributes(G, 'position')
	positions = {key: np.asarray([value[0], -value[1]]) for key, value in positions.items()}

	if draw_nodes:
		nx.draw(G, pos=positions, with_labels = True, node_color='gray', arrows=True, ax=ax)

	if path is not None:
		cmap = matplotlib.colormaps[cmap_str]
		red_shades = cmap(np.linspace(0, 1, len(path)))
		nx.draw_networkx_nodes(G, pos=positions, nodelist=path, node_color=red_shades, ax=ax)

	return ax


def path_length(G: nx.Graph, path: list) -> float:

	length = 0

	for i in range(len(path)-1):
		length += G[path[i]][path[i+1]]['weight']

	return length


# def random_shorted_path(G: nx.Graph, p0: int, p1:int) -> list:

# 	random_G = G.copy()
# 	for edge in random_G.edges():
# 		random_G[edge[0]][edge[1]]['weight'] = np.random.rand()
    
# 	return nx.shortest_path(random_G, p0, p1, weight='weight')[1:]

def random_shorted_path(G:nx.Graph,p0:int,p1:int):
    random_G =G.copy()
    for edge in random_G.edges():   
        random_G[edge[0]][edge[1]]['weight'] = np.random.rand()

    # print(f"Nodes in G: {list(random_G.nodes)}")
    # print(f"Number of nodes: {random_G.number_of_nodes()}")
    # print(f"Source node: {p0}, Destination node: {p1}")
    
    return nx.shortest_path(random_G, p0, p1, weight='weight')[1:]


def create_random_path_from_nodes(G : nx.Graph, start_node: int, distance: float, final_node: int = None) -> list:
    """ Select random nodes and create random path to reach them """

    path = []
    remain_distance = distance

    # Append the start node
    path.append(start_node)

    while path_length(G, path) < distance:

        # Select a random node
        next_node = np.random.choice(G.nodes())

        # Obtain a random path to reach it
        new_path = random_shorted_path(G, path[-1], next_node)
        path.extend(new_path)

        # Compute the distance of path
        remain_distance -= path_length(G, new_path)

    # Append the shortest path to the start node
    G_random = G.copy()
    # Generate random weights
    for edge in G_random.edges():
        G_random[edge[0]][edge[1]]['weight'] = np.random.rand()

    # Append the shortest path to the start node
    if final_node is not None:
        path.extend(nx.shortest_path(G_random, path[-1], final_node, weight='weight')[1:])
    else:
        path.extend(nx.shortest_path(G_random, path[-1], start_node, weight='weight')[1:])

    return path[1:]

def visualize_graph(G):
    pos = nx.get_node_attributes(G, 'position')
    nx.draw(G, pos, node_size=20, with_labels=False)
    plt.show()
    # print(f"El grafo tiene {len(G.nodes)} nodos.")
    # for node in G.nodes(data=True):  # Obtiene nodos con sus atributos
    #     node_id = node[0]
    #     attributes = node[1]
    #     print(f"Nodo {node_id}: {attributes}")



def check_path(graph, node_start, node_end):
    """
    Comprueba si hay un camino entre dos nodos en un grafo y devuelve el camino si existe.
    
    Parameters:
        graph (nx.Graph): El grafo en el que se buscará el camino.
        node_start (hashable): Nodo inicial.
        node_end (hashable): Nodo final.

    Returns:
        list: Una lista con los nodos que forman el camino si existe, de lo contrario devuelve None.
    """
    try:
        # Usa NetworkX para encontrar el camino más corto (si existe)
        path = nx.shortest_path(graph, source=node_start, target=node_end)
        return path
    except nx.NetworkXNoPath:
        # No hay camino entre los nodos
        print(f"No hay camino entre los nodos {node_start} y {node_end}.")
        return None
    except nx.NodeNotFound as e:
        # Alguno de los nodos no está en el grafo
        print(f"Error: {e}")
        return None
    

def main():
    # Parámetros del entorno

    max_distance = 100  # Distancia máxima (batería)



    # navigation_map , high= generate_sample_maps(map_size,scale)
    navigation_map = np.genfromtxt('../maps/output/heightmap_traversability.txt', delimiter=' ')
    high_map= np.genfromtxt('../maps/output/heightmap_z_values.txt', delimiter=' ')
    n_agents = 1
    
    initial_position=0
	#final_positions = np.genfromtxt('../maps/output/interest_points.txt', delimiter=' ')
    # final_position = np.array([40,20,30,40])[:N_agents]
    final_position=42
    scale = 1


    # Inicialización anterior...
    agent_states = {

            "position": initial_position,
            "distance_traveled": 0,
            "inclination": 0,
            "orientation": 0,  # Inicialmente sin giro
            "perception": {}  # Información observable
        }




    # Inicializar el problema
    problem = PatrollingGraphRoutingProblem(
        navigation_map=navigation_map,
        high_map=high_map,
        scale=scale,
        n_agents=n_agents,
        max_distance=max_distance,
        initial_positions=initial_position,
        final_positions=final_position
    )
    
    path=check_path(problem.G,initial_position,final_position)
    print(path)
    #visualize_graph(problem.G)
    # next_node = np.random.choice(problem.G.nodes())
    # print(next_node)
    # random_shorted_path(problem.G,initial_position,next_node)
    #path=create_random_path_from_nodes(problem.G,initial_position,150,final_position)
    #print(problem.G.nodes[final_position])
    #print(path)
    problem.evaluate_path(path, render=True)
    plot_graph(problem.G, path=path, draw_nodes=True)


if __name__ == "__main__":
    main()