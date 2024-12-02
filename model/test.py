import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from sklearn.metrics import mean_squared_error as mse

class PatrollingGraphRoutingProblem:
    """
    Clase que modela un problema de patrullaje utilizando un grafo generado
    a partir de un mapa de navegación, alturas y puntos de importancia.
    """
    
    def __init__(self, navigation_map, high_map, importance_map, scale, n_agents, max_distance, 
                 initial_positions, final_positions=None):
        """
        Inicializa el problema de patrullaje.

        Args:
            navigation_map (np.ndarray): Mapa binario de navegación (1 = transitable, 0 = obstáculo).
            high_map (np.ndarray): Mapa de alturas.
            importance_map (np.ndarray): Mapa de importancia (valores asociados a puntos del mapa).
            scale (int): Resolución de reducción para el mapa.
            n_agents (int): Número de agentes (robots).
            max_distance (float): Distancia máxima que puede recorrer un agente.
            initial_positions (np.ndarray): Posiciones iniciales de los agentes.
            final_positions (np.ndarray, opcional): Posiciones finales objetivo.
        """
        self.navigation_map = navigation_map
        self.high_map = high_map
        self.importance_map = importance_map
        self.scale = scale
        self.n_agents = n_agents
        self.max_distance = max_distance
        self.initial_positions = initial_positions
        self.final_positions = final_positions
        self.width = 50   # Ancho del espacio de navegación
        self.height = 50
        self.agent_positions = initial_positions.copy()
        self.agent_distances = {agent_id: 0 for agent_id in range(n_agents)}
        self.coverage_radius = 10  # Radio de visión de los agentes
        self.rewards = np.zeros(n_agents)
        
        # Crear el grafo a partir del mapa
        self.G = self.create_graph_from_map()
        print(f"Graph G created: {self.G}")
        # Waypoints para seguimiento
        self.waypoints = {agent_id: [self.G.nodes[pos]['position']] for agent_id, pos in enumerate(initial_positions)}
        
        # Variables para renderizado
        self.fig = None
        self.colors = ['red', 'blue', 'green', 'yellow', 'orange']
        self.markers = ['o', 'v', '*', 'p', '>']
    
               # Inicialización anterior...
        self.agent_states = {
            agent_id: {
                "position": initial_positions[agent_id],
                "distance_traveled": 0,
                "inclination": 0,
                "orientation": 0,  # Inicialmente sin giro
                "perception": {}  # Información observable
            }
            for agent_id in range(n_agents)
        }




    def create_graph_from_map(self):
        """
        Crea un grafo a partir del mapa de navegación y alturas, considerando restricciones de pendiente.
        Los vecinos se generan basándose en la proximidad en coordenadas.
        """
        G = nx.Graph()
        
        # Escalado del mapa
        scaled_map = self.navigation_map[::self.scale, ::self.scale]
        
        # Encuentra las posiciones visitables
        visitable_positions = np.column_stack(np.where(scaled_map == 1))  # Posiciones donde el mapa es '1' (visitables)
        
        # Añadir nodos al grafo con sus coordenadas y altura
        for i, position in enumerate(visitable_positions):
            pos_x, pos_y = position * self.scale
            G.add_node(i, position=(pos_x, pos_y), height=self.high_map[pos_x, pos_y])
        
        # Crear conexiones entre los nodos basadas en proximidad (distancia) y pendiente
        for i, node_a in enumerate(visitable_positions):
            for j, node_b in enumerate(visitable_positions):
                if i >= j:
                    continue

                # Calculamos las coordenadas reales (en píxeles o unidades escaladas)
                pos_a = node_a * self.scale
                pos_b = node_b * self.scale
                
                # Calculamos la distancia euclidiana entre las posiciones de los nodos
                distance = np.linalg.norm(pos_a - pos_b)
                
                # Calculamos la diferencia de altura (pendiente)
                slope = abs(G.nodes[i]['height'] - G.nodes[j]['height'])

                # Solo añadir la arista si la distancia y la pendiente cumplen las restricciones
                if distance <= self.coverage_radius and slope <= 1:
                    G.add_edge(i, j, weight=distance)

        return G





    def calculate_observation(self, agent_id):
        """
        Simula la observación de un agente con una cámara, limitada a dos nodos en la dirección seleccionada.

        Args:
            agent_id (int): ID del agente que observa.

        Returns:
            dict: Información de los nodos observables desde el agente.
        """
        current_position = self.agent_states[agent_id]["position"]
        observations = {}
        direction = None

        # Verificar el nodo hacia el cual se mueve el agente
        if "last_movement" in self.agent_states[agent_id]:
            last_move = self.agent_states[agent_id]["last_movement"]
            direction = (
                np.array(self.G.nodes[last_move]["position"]) -
                np.array(self.G.nodes[current_position]["position"])
            )
            direction = direction / np.linalg.norm(direction)  # Normalizar vector

        # Explorar nodos vecinos
        for neighbor in self.G.neighbors(current_position):
            neighbor_pos = self.G.nodes[neighbor]['position']
            vector_to_neighbor = np.array(neighbor_pos) - np.array(self.G.nodes[current_position]['position'])
            distance = np.linalg.norm(vector_to_neighbor)
            if direction is None or np.dot(vector_to_neighbor, direction) > 0:  # Misma dirección
                if distance <= self.coverage_radius:  # En el radio de percepción
                    observations[neighbor] = {
                        "position": neighbor_pos,
                        "height": self.G.nodes[neighbor]['height'],
                        "distance": distance
                    }
                    # Explorar el siguiente nodo conectado
                    for next_neighbor in self.G.neighbors(neighbor):
                        if next_neighbor != current_position:
                            observations[next_neighbor] = {
                                "position": self.G.nodes[next_neighbor]['position'],
                                "height": self.G.nodes[next_neighbor]['height'],
                                "distance": np.linalg.norm(
                                    np.array(self.G.nodes[next_neighbor]['position']) - np.array(neighbor_pos)
                                )
                            }
        return observations

    def step(self, new_positions):
        """
        Realiza un paso del entorno, moviendo a los agentes y calculando recompensas.

        Args:
            new_positions (dict): Diccionario con los IDs de los agentes como claves y posiciones a las que deben moverse.

        Returns:
            state (dict): Estados actualizados de los agentes.
            rewards (np.ndarray): Recompensas de los agentes tras el paso.
            done (bool): True si todos los agentes han terminado.
        """
        rewards = np.zeros(self.n_agents)
        done = False

        for i in range(self.n_agents):
            current_pos = self.agent_states[i]["position"]
            new_pos = new_positions.get(i, current_pos)

            # Validar movimiento y actualización del grafo
            if new_pos == -1 or not self.G.has_edge(current_pos, new_pos):
                continue

            # Registrar movimiento
            self.agent_states[i]["last_movement"] = new_pos
            # Actualizar distancia y posición
            edge_data = self.G.get_edge_data(current_pos, new_pos)
            self.agent_states[i]["distance_traveled"] += edge_data['weight']
            self.agent_states[i]["position"] = new_pos

            # Calcular pendiente e inclinación
            current_height = self.G.nodes[current_pos]['height']
            new_height = self.G.nodes[new_pos]['height']
            slope = abs(current_height - new_height)
            self.agent_states[i]["inclination"] = slope
            self.agent_states[i]["orientation"] = np.arctan2(new_height - current_height, edge_data['weight'])

            # Calcular percepción del entorno
            perception = self.calculate_observation(i)
            self.agent_states[i]["perception"] = perception

            # Asignar recompensas basadas en percepción y pendiente
            rewards[i] += len(perception)  # Recompensa por cantidad de nodos observables
            if slope <= 1:
                rewards[i] += 1  # Recompensa por transición válida
            else:
                rewards[i] -= 1  # Penalización por pendiente alta

        done = all(
            self.agent_states[i]["distance_traveled"] > self.max_distance
            for i in range(self.n_agents)
        )
        return self.agent_states, rewards, done

    def render(self):
        """
        Renderiza el entorno y muestra la posición actual de los agentes.
        """
        if not hasattr(self, 'fig') or self.fig is None:
            self.fig, self.ax = plt.subplots()

        self.ax.clear()
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(0, self.height)
        self.ax.set_title("Simulación de navegación reactiva")

        for agent_id, state in self.agent_states.items():
            node_id = state["position"]  # Este es el nodo que tienes, como 0, 1, etc.
            if node_id in self.G.nodes:
                x, y = self.G.nodes[node_id]['position']
            self.ax.plot(x, y, 'bo', label=f'Agente {agent_id} (Inclinación: {state["inclination"]:.2f})')

        self.ax.legend()
        plt.pause(0.5)
        plt.show()




    def evaluate_path(self,path, agent_states,render=False):
        """
        Evalúa un camino dado en el grafo considerando la distancia, inclinación y percepción.

        Args:
            path (list): Lista de nodos que conforman el camino.
            graph (nx.Graph): Grafo que contiene la información de las rutas.
            agent_states (dict): Estados actuales de los agentes.

        Returns:
            float: Puntuación del camino (mayor es mejor).
        """
        total_distance = 0
        total_inclination = 0
        total_perception = 0

        for i in range(len(path) - 1):
            current_node = path[i]
            next_node = path[i + 1]

            # Distancia entre nodos
            if self.G.get_edge_data(current_node, next_node):
                edge_data = self.G.get_edge_data(current_node, next_node)
                total_distance += edge_data['weight']
            else:
                continue  # Si no hay conexión, ignorar

            # Inclinación entre nodos
            current_height = self.G.nodes[current_node]['height']
            next_height = self.G.nodes[next_node]['height']
            total_inclination += abs(current_height - next_height)

        # Evaluar percepción en el último nodo del camino
        
        # Iterar sobre cada agente en agent_states
        for agent_id, state in agent_states.items():
            # Asegurarse de que agent_id esté en el diccionario path y que el valor sea una lista
            if agent_id in path and isinstance(path[agent_id], list) and path[agent_id]:
                last_node = path[agent_id][-1]  # Obtener el último nodo para este agente
                
                # Evaluar percepción en el último nodo
                if np.linalg.norm(
                    np.array(self.G.nodes[last_node]['position']) - np.array(state["position"])
                ) <= state["perception"].get('radius', 10):  # Ejemplo de radio de percepción
                    total_perception += 1
            else:
                # Manejo de casos donde el agent_id no tiene una lista válida en path
                print(f"El agent_id {agent_id} no tiene un camino válido en 'path'.")

        # Ponderar los criterios de evaluación
        score = -total_distance  # Penalizar distancias más largas
        score -= total_inclination * 0.5  # Penalizar inclinaciones altas (ajusta el peso según el caso)
        score += total_perception * 2  # Premiar buena percepción

        if render:
            self.render()

        return score

