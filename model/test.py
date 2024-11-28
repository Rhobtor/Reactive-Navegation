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
        
        # Waypoints para seguimiento
        self.waypoints = {agent_id: [self.G.nodes[pos]['position']] for agent_id, pos in enumerate(initial_positions)}
        
        # Variables para renderizado
        self.fig = None
        self.colors = ['red', 'blue', 'green', 'yellow', 'orange']
        self.markers = ['o', 'v', '*', 'p', '>']
    
    def create_graph_from_map(self):
        """
        Crea un grafo a partir del mapa de navegación y alturas, considerando restricciones de pendiente.
        """
        G = nx.Graph()
        scaled_map = self.navigation_map[::self.scale, ::self.scale]
        visitable_positions = np.column_stack(np.where(scaled_map == 1))

        for i, position in enumerate(visitable_positions):
            pos_x, pos_y = position * self.scale
            G.add_node(i, position=(pos_x, pos_y), height=self.high_map[pos_x, pos_y])
        
        for i, node_a in enumerate(visitable_positions):
            for j, node_b in enumerate(visitable_positions):
                if i >= j:
                    continue
                distance = np.linalg.norm(node_a - node_b)
                slope = abs(G.nodes[i]['height'] - G.nodes[j]['height'])
                if distance <= self.coverage_radius and slope <= 1:  # Restricción de pendiente
                    G.add_edge(i, j, weight=distance)
        
        return G

    def step(self, new_positions):
        """
        Realiza un paso del entorno, moviendo a los agentes y calculando recompensas.

        Args:
            new_positions (dict): Diccionario con los IDs de los agentes como claves y posiciones a las que deben moverse.

        Returns:
            rewards (np.ndarray): Recompensas de los agentes tras el paso.
            done (bool): True si todos los agentes han terminado.
        """
        rewards = np.zeros(self.n_agents)
        done = False

        for i in range(self.n_agents):
            current_pos = self.agent_positions[i]
            new_pos = new_positions[i]
            
            if new_pos == -1 or not self.G.has_edge(current_pos, new_pos):
                continue
            
            # Actualizar la posición y calcular la recompensa
            edge_data = self.G.get_edge_data(current_pos, new_pos)
            self.agent_distances[i] += edge_data['weight']
            self.agent_positions[i] = new_pos
            
            # Calcular la pendiente y recompensa
            slope = abs(self.G.nodes[current_pos]['height'] - self.G.nodes[new_pos]['height'])
            if slope <= 1:
                rewards[i] += 1  # Recompensa por transición válida
            else:
                rewards[i] -= 1  # Penalización por pendiente alta

        done = all(distance > self.max_distance for distance in self.agent_distances.values())
        return rewards, done
    
    def render(self):
        """
        Renderiza el entorno y muestra la posición actual de los agentes.
        """
        # Inicializar figura y eje si no están definidos
        if not hasattr(self, 'fig') or self.fig is None:
            self.fig, self.ax = plt.subplots()

        # Limpiar el gráfico
        self.ax.clear()
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(0, self.height)
        self.ax.set_title("Simulación de navegación reactiva")

        # Dibujar posiciones de los agentes
        if isinstance(self.agent_positions, dict):
            # Si es un diccionario
            for agent_id, position in self.agent_positions.items():
                if isinstance(position, (list, tuple, np.ndarray)) and len(position) == 2:
                    x, y = position
                    self.ax.plot(x, y, 'bo', label=f'Agente {agent_id}')
                else:
                    raise ValueError(f"Posición del agente {agent_id} no válida: {position}")
        elif isinstance(self.agent_positions, np.ndarray):
            # Si es un arreglo NumPy bidimensional
            if self.agent_positions.ndim == 2 and self.agent_positions.shape[1] == 2:
                for agent_id, position in enumerate(self.agent_positions):
                    x, y = position
                    self.ax.plot(x, y, 'bo', label=f'Agente {agent_id}')
            else:
                raise ValueError("agent_positions como arreglo NumPy debe tener formato (n, 2).")
        else:
            raise TypeError("agent_positions debe ser un diccionario o un arreglo NumPy.")

        self.ax.legend()
        plt.pause(0.5)





    def evaluate_path(self, paths, render=False):
        """
        Evalúa la ruta seguida por los agentes.

        Args:
            paths (dict): Diccionario con los IDs de los agentes como claves y listas de nodos visitados como valores.
            render (bool): Si True, renderiza el grafo después de evaluar.

        Returns:
            dict: Recompensas totales por agente.
        """
        rewards = {agent_id: 0 for agent_id in paths.keys()}
        self.agent_positions = {i: None for i in range(self.n_agents)}

        print("Evaluando rutas...")

        for agent_id, path in paths.items():
            for i, next_node in enumerate(path):
                new_positions = {agent_id: next_node}
                print(new_positions)
                # Obtener la posición (x, y) del nodo en el grafo
                pos_x, pos_y = self.G.nodes[next_node]['position']
                print(pos_x,pos_y)
                print(self.agent_positions[agent_id])
                self.agent_positions[agent_id]=(pos_x, pos_y)
                print(self.agent_positions[agent_id])


                # # Realizar un paso del entorno
                step_rewards, _ = self.step(new_positions)
                rewards[agent_id] += step_rewards[agent_id]

                print(f"Agente {agent_id} - Paso {i}: Nodo {next_node} -> Recompensa: {step_rewards[agent_id]} -> Total: {rewards[agent_id]}")

                if render:
                    self.render()

        return rewards

