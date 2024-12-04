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
            # pos_x, pos_y = position * self.scale
            # G.add_node(i, position=(pos_x, pos_y), height=self.high_map[pos_x, pos_y])
            G.add_node(i,position=position[::-1]*self.scale,coords=position*self.scale)
            # x_index, y_index = position * self.scale
            
            #     # If self.high_map is a 1D array, calculate the corresponding index
            # map_width = int(len(self.high_map) ** 0.5)  # Assuming square grid
            # flat_index = x_index * map_width + y_index
            
            # if 0 <= flat_index < len(self.high_map):
            #     high = self.high_map[flat_index]  # Access height value
            # else:
            #     high = None  # Handle out-of-bounds case
            
            # nx.set_node_attributes(G, {i: high}, 'height')
            

        for i, position in enumerate(visitable_positions):
            for j, other_position in enumerate(visitable_positions):
                if i != j:
                    if np.linalg.norm(position - other_position) <= np.sqrt(2):
                        G.add_edge(i, j, weight=np.linalg.norm(position - other_position)*self.scale)
                        
        # # Crear conexiones entre los nodos basadas en proximidad (distancia) y pendiente
        # for i, node_a in enumerate(visitable_positions):
        #     for j, node_b in enumerate(visitable_positions):
        #         if i >= j:
        #             continue

        #         # Calculamos las coordenadas reales (en píxeles o unidades escaladas)
        #         pos_a = node_a * self.scale
        #         pos_b = node_b * self.scale
                
        #         # Calculamos la distancia euclidiana entre las posiciones de los nodos
        #         distance = np.linalg.norm(pos_a - pos_b)
                
        #         # Calculamos la diferencia de altura (pendiente)
        #         slope = abs(G.nodes[i]['height'] - G.nodes[j]['height'])

        #         # Solo añadir la arista si la distancia y la pendiente cumplen las restricciones
        #         if distance <= self.coverage_radius and slope <= 1:
        #             G.add_edge(i, j, weight=distance)

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

    # def step(self, new_positions):
    #     """
    #     Realiza un paso del entorno, moviendo a los agentes y calculando recompensas.

    #     Args:
    #         new_positions (dict): Diccionario con los IDs de los agentes como claves y posiciones a las que deben moverse.

    #     Returns:
    #         state (dict): Estados actualizados de los agentes.
    #         rewards (np.ndarray): Recompensas de los agentes tras el paso.
    #         done (bool): True si todos los agentes han terminado.
    #     """
    #     rewards = np.zeros(self.n_agents)
    #     done = False

    #     for i in range(self.n_agents):
    #         current_pos = self.agent_states[i]["position"]
    #         new_pos = new_positions.get(i, current_pos)

    #         # Validar movimiento y actualización del grafo
    #         if new_pos == -1 or not self.G.has_edge(current_pos, new_pos):
    #             continue

    #         # Registrar movimiento
    #         self.agent_states[i]["last_movement"] = current_pos
    #         # Actualizar distancia y posición
    #         self.agent_distances[i] += self.G[self.agent_positions[i]][new_positions[i]]['weight']
    #         self.agent_states[i] += edge_data['weight']
    #         self.agent_states[i]["position"] = new_pos

    #         # Calcular pendiente e inclinación
    #         current_height = self.G.nodes[current_pos]['height']
    #         new_height = self.G.nodes[new_pos]['height']
    #         slope = abs(current_height - new_height)
    #         self.agent_states[i]["inclination"] = slope
    #         self.agent_states[i]["orientation"] = np.arctan2(new_height - current_height,new_positions-s['weight'])

    #         # Calcular percepción del entorno
    #         perception = self.calculate_observation(i)
    #         self.agent_states[i]["perception"] = perception

    #         # Asignar recompensas basadas en percepción y pendiente
    #         rewards[i] += len(perception)  # Recompensa por cantidad de nodos observables
    #         if slope <= 1:
    #             rewards[i] += 1  # Recompensa por transición válida
    #         else:
    #             rewards[i] -= 1  # Penalización por pendiente alta

    #     done = all(
    #         self.agent_states[i]["distance_traveled"] > self.max_distance
    #         for i in range(self.n_agents)
    #     )
    #     return self.agent_states, rewards, done


    def step(self, new_positions: np.ndarray):

        # Check if the new positions are neighbors of the current positions of the agents
        for i in range(self.n_agents):

            if new_positions[i] == -1:
                continue

            if new_positions[i] not in list(self.G.neighbors(self.agent_positions[i])):
                raise ValueError('The new positions are not neighbors of the current positions of the agents')

            # Compute the distance traveled by the agents using the edge weight
            self.agent_distances[i] += self.G[self.agent_positions[i]][new_positions[i]]['weight']
            self.G.nodes[self.agent_positions[i]]['values']=0,1

        # Update the positions of the agents
        
        self.agent_positions = new_positions.copy()
        for i in range(self.n_agents):
            # Procesamos el reward #
            self.rho_next[i] = self.G.nodes.get(self.agent_positions[i], {'value': 0.0})['value'] 
            self.rho_act[i] = self.G.nodes.get(self.agent_pos_ant[i], {'value': 0.0})['value']
	
            node_id = self.agent_positions[i]
            if node_id in self.G.nodes:
                self.G.nodes[node_id]['rh_reward'] = self.rho_next[i] - self.rho_act[i]
 
        reward = np.array([0]*len(self.G.nodes[1]['importance']), dtype = float)
        idle = [self.G.nodes[node]['rh_reward'] for node in self.agent_positions if node in self.G.nodes]
        imp = [self.G.nodes[node]['importance'] for node in self.agent_positions if node in self.G.nodes]
        #reward = [0.0] * len(imp[0])  # Initialize reward with zeros
        for imp_index in range(len(self.G.nodes[1]['importance'])):
            for ship_index in range(len(idle)):
                if imp_index < len(imp[ship_index]) and ship_index < len(idle):
                    reward[imp_index] += np.array(idle[ship_index]) * np.array(imp[ship_index][imp_index])

        # print('bbb',reward)
        for node in range(1, len(self.G)):
            if node in self.agent_positions:
                self.G.nodes[node]['importance'] = list(np.array(self.G.nodes[node]['importance']) - 0.2*np.array(self.G.nodes[node]['importance']))
                for index in range(len(self.G.nodes[node]['importance'])):
                    if self.G.nodes[node]['importance'][index] < 0:
                        self.G.nodes[node]['importance'][index] = 0
        
        
        self.rewards = reward
   
        for node_index in self.G.nodes:
            current_value = self.G.nodes[node_index]['value']  # Obtén el valor actual del atributo 'value'
            new_value = min([current_value + 0.05, 1]) # Calcula el nuevo valor
            
            self.G.nodes[node_index]['value'] = new_value  # Act

        for i in range(self.n_agents):
            if self.agent_positions[i] in self.L.nodes:
                self.G.nodes[self.agent_positions[i]]['value'] = 0.1
        
        
        
        
        
        self.agent_pos_ant= self.agent_positions #casi al final


        # Update the waypoints
        for agent_id, new_position in enumerate(new_positions):
            if new_position != -1:
                # Append the position from the node
                self.waypoints[agent_id].append(list(self.G.nodes[new_position]['position']))

        # Update the idleness and information maps with the rewards
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
        
        # self.reset()

        if render:
            self.render()
        
        done = False
        t = 0

        final_rewards = np.array([0]*len(self.G.nodes[1]['importance']), dtype = float)
        while not done:
            next_positions = np.zeros_like(self.agent_positions)
            
            for i in range(self.n_agents):
                if t < len(path[i]):
                    
                    next_positions[i] = path[i][t]
                else:
                    next_positions[i] = -1

            new_rewards, done = self.step(next_positions)

            #print('esto',new_rewards)
            final_rewards+=new_rewards
            # print('esttta',final_rewards)
            # for key in new_rewards.keys():
            # 	final_rewards[key] += new_rewards[key]
            # print(new_rewards)
            if render:
                self.render()
            
            t += 1
        return final_rewards


