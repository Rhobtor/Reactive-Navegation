import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from sklearn.metrics import mean_squared_error as mse
from ShekelGroundTruth import shekel
from GaussianProcessModel import GaussianProcessModel

class PatrollingGraphRoutingProblem:
    """
    Clase que modela un problema de patrullaje utilizando un grafo generado
    a partir de un mapa de navegación, alturas y puntos de importancia.
    """
    
    def __init__(self, navigation_map, high_map, scale, n_agents, max_distance, 
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
        self.rewards = 0
        self.model = GaussianProcessModel(navigation_map = navigation_map)
        self.ground_truth = shekel(self.navigation_map, max_number_of_peaks=6, seed = 0, dt=0.05)
        # Crear el grafo a partir del mapa
        self.G = self.create_graph_from_map()

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

            G.add_node(i,position=position[::-1]*self.scale,coords=position*self.scale)
            x_index, y_index = position * self.scale
            x_index, y_index = position * self.scale
            height_value = self.high_map[x_index, y_index]
            nx.set_node_attributes(G, {i:height_value},'high')
            nx.set_node_attributes(G, {i: 0}, 'distance_reward')

        for i, position in enumerate(visitable_positions):
            for j, other_position in enumerate(visitable_positions):
                if i != j:
                    if np.linalg.norm(position - other_position) <= np.sqrt(2):
                        G.add_edge(i, j, weight=np.linalg.norm(position - other_position)*self.scale)
                        
        return G

    def reset(self):
            # Reset all the variables of the scenario #

            self.ground_truth.reset()
            self.model.reset()

            self.agent_positions = self.initial_positions.copy()
            self.agent_pos_ant= self.agent_positions
            
            # Reset the rewards #
            self.rewards = {}


            self.waypoints = {agent_id: [list(self.G.nodes[initial_position]['position'])] for agent_id, initial_position in zip(range(self.n_agents), self.initial_positions)}
            self.agent_distances = {agent_id: 0 for agent_id in range(self.n_agents)}

            # Input the initial positions to the model
            new_position_coordinates = np.array([self.G.nodes[new_position]['position'] for new_position in self.agent_positions])
            new_samples = self.ground_truth.read(new_position_coordinates)





            # Update the model
            self.model.update(new_position_coordinates, new_samples)

    def update_maps(self):
        """ Update the idleness and information maps """

        # Input the initial positions to the model

        new_position_coordinates = np.array([self.G.nodes[new_position]['position'] for new_position in self.agent_positions if new_position != -1])
        
        # Check if no new positions are available
        if new_position_coordinates.shape[0] != 0:
            new_samples = self.ground_truth.read(new_position_coordinates)

            # Update the model
            self.model.update(new_position_coordinates, new_samples)
            self.information_map = self.model.predict() # esta es la y , la w en el en paper



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


    def step(self, new_positions: np.ndarray):

        # Check if the new positions are neighbors of the current positions of the agents
        for i in range(self.n_agents):

            if new_positions[i] == -1:
                continue

            if new_positions[i] not in list(self.G.neighbors(self.agent_positions[i])):
                raise ValueError('The new positions are not neighbors of the current positions of the agents')

        reward=0
        # Update the positions of the agents
        for i in range(self.n_agents):

            node_id = self.agent_positions[i]
            
            if node_id in self.G.nodes:

                pos1 = np.array(self.G.nodes[self.agent_positions[i]]['coords'])               
                pos2 = np.array(self.G.nodes[self.final_positions]['coords'])    
                self.distance = np.linalg.norm(pos1 - pos2)



        self.agent_positions = new_positions.copy()

        
            
        for i in range(self.n_agents):

            node_id = self.agent_positions[i]
            if node_id in self.G.nodes:
                pos1 = np.array(self.G.nodes[self.agent_positions[i]]['coords'])         
                pos2 = np.array(self.G.nodes[self.final_positions]['coords'])    
                self.distance_new = np.linalg.norm(pos1 - pos2)
            
            if self.distance_new < self.distance:
                reward=reward+1


        
        self.rewards = reward
   

        
        
        
        self.agent_pos_ant= self.agent_positions #casi al final



        self.update_maps()

        done = np.asarray([agent_distance > self.max_distance for agent_distance in self.agent_distances.values()]).all()

        done = done or np.asarray([agent_position == -1 for agent_position in self.agent_positions]).all()
        
        # Return the rewards
        return self.rewards, done


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




    def evaluate_path(self,path,render=False):
        """ Evaluate a path """
        
        self.reset()

        if render:
            self.render()
        
        done = False
        t = 0

        final_rewards = 0
        while not done:
            next_positions = [0] * len(path) 
            
            for i in range(len(path)):
                
                next_positions[i] = path[i]
            
            #print(next_positions)
            new_rewards, done = self.step(next_positions)


            final_rewards+=new_rewards

            if render:
                self.render()
            
            t += 1
        return final_rewards


