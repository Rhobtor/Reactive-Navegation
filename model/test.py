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
        self.agent_positions = initial_positions
        self.agent_distances = {agent_id: 0 for agent_id in range(n_agents)}
        self.coverage_radius = 10  # Radio de visión de los agentes
        self.rewards = 0
        self.model = GaussianProcessModel(navigation_map = navigation_map)
        self.ground_truth = shekel(self.navigation_map, max_number_of_peaks=6, seed = 0, dt=0.05)
        # Crear el grafo a partir del mapa
        self.G = self.create_graph_from_map()

        # Waypoints para seguimiento
        self.waypoints = [self.G.nodes[self.agent_positions]['position']]
        
        # Variables para renderizado
        self.fig = None
        self.colors = ['red', 'blue', 'green', 'yellow', 'orange']
        self.markers = ['o', 'v', '*', 'p', '>']
    
        # Inicialización anterior...
        self.agent_states = {
   
                "position": initial_positions,
                "distance_traveled": 0,
                "inclination": 0,
                "orientation": 0,  # Inicialmente sin giro
                "perception": {}  # Información observable
            }
            
        



    # def create_graph_from_map(self):
    #     """
    #     Crea un grafo a partir del mapa de navegación y alturas, considerando restricciones de pendiente.
    #     Los vecinos se generan basándose en la proximidad en coordenadas.
    #     """
    #     G = nx.Graph()
        
    #     # Escalado del mapa
    #     scaled_map = self.navigation_map[::self.scale, ::self.scale]
        
    #     # Encuentra las posiciones visitables
    #     visitable_positions = np.column_stack(np.where(scaled_map == 1))  # Posiciones donde el mapa es '1' (visitables)
        
    #     # Añadir nodos al grafo con sus coordenadas y altura
    #     for i, position in enumerate(visitable_positions):

    #         G.add_node(i,position=position[::-1]*self.scale,coords=position*self.scale)
    #         x_index, y_index = position * self.scale
    #         x_index, y_index = position * self.scale
    #         height_value = self.high_map[x_index, y_index]
    #         nx.set_node_attributes(G, {i:height_value},'high')
    #         nx.set_node_attributes(G, {i: 0}, 'distance_reward')

    #     for i, position in enumerate(visitable_positions):
    #         for j, other_position in enumerate(visitable_positions):
    #             if i != j:
    #                 if np.linalg.norm(position - other_position) <= np.sqrt(2):
    #                     G.add_edge(i, j, weight=np.linalg.norm(position - other_position)*self.scale)
                        
    #     return G

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

        # Altura máxima del mapa (para invertir el eje Y)
        max_y = scaled_map.shape[0] * self.scale

        # Añadir nodos al grafo con sus coordenadas ajustadas
        for i, position in enumerate(visitable_positions):
            # Invertir eje Y
            adjusted_position = position * self.scale
            adjusted_position[1] = max_y - adjusted_position[1]  # Ajustar eje Y

            G.add_node(i, position=adjusted_position[::-1], coords=adjusted_position)
            x_index, y_index = position * self.scale
            height_value = self.high_map[x_index, y_index]
            nx.set_node_attributes(G, {i: height_value}, 'high')
            nx.set_node_attributes(G, {i: 0}, 'distance_reward')

        # Añadir las aristas según la proximidad
        for i, position in enumerate(visitable_positions):
            for j, other_position in enumerate(visitable_positions):
                if i != j:
                    if np.linalg.norm(position - other_position) <= np.sqrt(2):
                        G.add_edge(i, j, weight=np.linalg.norm(position - other_position) * self.scale)

        return G




    def reset(self):
            # Reset all the variables of the scenario #

            self.ground_truth.reset()
            self.model.reset()

            self.agent_positions = self.initial_positions
            self.agent_pos_ant= self.agent_positions
            
            # Reset the rewards #
            self.rewards = {}


            self.waypoints = (self.G.nodes[self.initial_positions]['position'])
            self.agent_distances = {agent_id: 0 for agent_id in range(self.n_agents)}

            # Input the initial positions to the model
            new_position_coordinates = np.array([self.G.nodes[self.initial_positions]['position']])
            #print(new_position_coordinates)
            new_samples = self.ground_truth.read(new_position_coordinates)





            # Update the model
            self.model.update(new_position_coordinates, new_samples)

    def update_maps(self):
        """ Update the idleness and information maps """

        # Input the initial positions to the model

        new_position_coordinates = np.array([self.G.nodes[self.new_position]['position']])
        
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
        for i in range(len(new_positions)):
            if new_positions[i] == -1:
                continue

            # Convert list with single element to int (in case new_positions[i] is a list)
            self.new_position = new_positions[i][0] if isinstance(new_positions[i], list) else new_positions[i]
            
            # Verifica si la nueva posición es un vecino de la posición actual
            if self.new_position not in list(self.G.neighbors(self.agent_positions)):
                raise ValueError('The new positions are not neighbors of the current positions of the agents')

        reward=0
        # Convert list with single element to int (in case new_positions[i] is a list)
        self.new_position = new_positions[i][0] if isinstance(new_positions[i], list) else new_positions[i]
        
        node_id = self.agent_positions
       
        if node_id in self.G.nodes:

            pos1 = np.array(self.G.nodes[self.agent_positions]['coords'])               
            pos2 = np.array(self.G.nodes[self.final_positions]['coords'])    
            self.distance = np.linalg.norm(pos1 - pos2)



        self.agent_positions = self.new_position

        
            
        
        
        node_id_new = self.agent_positions
        
        if node_id_new in self.G.nodes:
            pos1 = np.array(self.G.nodes[self.agent_positions]['coords'])         
            pos2 = np.array(self.G.nodes[self.final_positions]['coords'])    
            self.distance_new = np.linalg.norm(pos1 - pos2)
        
        if self.distance_new < self.distance:
            reward=reward+1


        
        self.rewards = reward

        
        self.agent_pos_ant= self.agent_positions #casi al final

        # Update the waypoints
        
        if self.new_position != -1:
            # Append the position from the node
            self.waypoints= [self.G.nodes[self.new_position]['position']]
            self.update_maps()
        # self.waypoints = [self.G.nodes[self.agent_positions]['position']]
        

        done = np.asarray([agent_distance > self.max_distance for agent_distance in self.agent_distances.values()]).all()
        if self.new_position == -1:
            # Append the position from the node
            done=True
        # done = done or np.asarray([self.agent_positions == -1])
        
        # Return the rewards
        return self.rewards, done


    # def render(self):
    #     if self.fig is None:

    #         self.fig, self.ax = plt.subplots(1, 2, figsize=(10, 10))

    #         self.d1 = self.ax[0].imshow(self.navigation_map, cmap='gray', vmin=0, vmax=1)
    #         # self.d2 = self.ax[1].imshow(self.ground_truth.read(), cmap='gray', vmin=0, vmax=1)

    #         self.agents_render_pos = []
    #         for i in range(self.n_agents):
                
    #             print(f"agent positiontttt: {self.agent_positions}")
    #             agent_position_coords = np.array(self.G.nodes[self.agent_positions[i]]['coords'])
    #             print(f"agent position: {agent_position_coords}")
    #             self.agents_render_pos.append(self.ax[0].plot(agent_position_coords[0], agent_position_coords[1], color=self.colors[i], marker=self.markers[i], markersize=10, alpha=0.35)[0])

    #     else:

    #         for i in range(self.n_agents):
                
    #             traj = np.asarray(self.waypoints[i])
    #             # Plot the trajectory of the agent
    #             self.agents_render_pos[i].set_data(traj[:,0], traj[:,1])

    #         self.d1.set_data(self.navigation_map)
    #         # self.d2.set_data(self.ground_truth.read())

            
    #     self.fig.canvas.draw()
    #     plt.pause(0.01)

    def render(self):
        if self.fig is None:
            self.fig, self.ax = plt.subplots(1, 2, figsize=(10, 10))

            # Mostrar el mapa de navegación
            self.d1 = self.ax[0].imshow(self.navigation_map, cmap='gray', vmin=0, vmax=1)

            # Invertir el eje Y para alinear el origen en la esquina inferior izquierda
            self.ax[0].invert_yaxis()

            # Configurar límites de los ejes
            self.ax[0].set_xlim(0, self.navigation_map.shape[1])
            self.ax[0].set_ylim(0, self.navigation_map.shape[0])

            self.agents_render_pos = []
            for i in range(self.n_agents):
                print(f"agent positiontttt: {self.agent_positions}")

                # Ajustar las coordenadas del agente
                agent_position_coords = self.G.nodes[self.agent_positions]['coords']
                agent_position_coords[1] = self.navigation_map.shape[0] - agent_position_coords[1]  # Ajustar eje Y
                print(f"agent position: {agent_position_coords}")

                self.agents_render_pos.append(
                    self.ax[0].plot(
                        agent_position_coords[0],
                        agent_position_coords[1],
                        color=self.colors[i],
                        marker=self.markers[i],
                        markersize=10,
                        alpha=0.35
                    )[0]
                )
        else:
            for i in range(self.n_agents):
                traj = np.asarray(self.waypoints)
                
                # # Ajustar las coordenadas de la trayectoria
                traj[:, 1] = self.navigation_map.shape[0] - traj[:, 1]  # Ajustar eje Y
                
                # Actualizar la trayectoria del agente
                self.agents_render_pos[i].set_data(traj[:, 0], traj[:, 1])

            self.d1.set_data(self.navigation_map)

        self.fig.canvas.draw()
        plt.pause(0.01)



    def evaluate_path(self,path,render=False):
        """ Evaluate a path """
        
        self.reset()

        # if render:
        #     self.render()
        
        done = False
        t = 0

        final_rewards = 0
        print(path)
        while not done:
            if t+1 < len(path):  # Verificar que t no exceda el tamaño del path
                next_positions = [path[t+1]]  # Tomar solo la posición actual del path
            else:
                next_positions = [-1]  # Usar -1 si t excede el tamaño del path

            print(f"Next position: {next_positions}")  # Mostrar la posición actual
            
            # Ejecutar la acción con la posición actual
            new_rewards, done = self.step(next_positions)
            
            print(f"Rewards: {new_rewards}, Done: {done}")  # Mostrar los resultados de la acción
            final_rewards += new_rewards  # Acumular las recompensas
            
            t += 1  # Avanzar al siguiente paso
            print(f"actual position: {self.agent_positions}")

            if render and self.agent_positions!=-1:
               
                self.render()
                
          
        return final_rewards


