import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import matplotlib
# from AlgaeBloomGroundTruth import algae_bloom
from ShekelGroundTruth import shekel
from itertools import cycle
from GaussianProcessModel import GaussianProcessModel
from sklearn.metrics import mean_squared_error as mse
import pandas as pd


class PatrollingGraphRoutingProblem:

	def __init__(self, navigation_map: np.ndarray, 
			  	high_map:np.ndarray,
				scale: int, 
				n_agents: int, 
				max_distance: float, 
				initial_positions: np.ndarray,
				ground_truth: str ,
				final_positions: np.ndarray = None
				 ):

		self.navigation_map = navigation_map
		self.high_map= high_map
		self.scale = scale
		self.n_agents = n_agents # for now only 1
		self.max_distance = max_distance # depend the baterry
		self.initial_positions = initial_positions #initial position of the robot
		self.final_positions = final_positions # position of people to rescue
		self.waypoints = {agent_id: [] for agent_id in range(n_agents)} # waypoints of vehicle
		# max_importance_map = tuple([np.sum(item) for item in importance_map])
		self.rho_next = {} #recompensas de actuacion siguiente del modelo en step
		self.rho_act = {} # recompensas de actuacion actual del modelo en step
		self.coverage_radius = 10
		# Create the graph
		self.G = create_graph_from_map(self.navigation_map, self.scale,self.high_map)
		benchmark = ground_truth
		# Create the grund truth #
		# """ Create the benchmark """
		if benchmark == 'shekel':
			self.ground_truth = shekel(self.navigation_map, max_number_of_peaks=6, seed = 0, dt=0.05)
		# elif benchmark == 'algae_bloom':
		# 	self.ground_truth = algae_bloom(self.navigation_map, dt=0.5, seed=0)
		# else:
		# 	raise ValueError('Unknown benchmark')

		self.model = GaussianProcessModel(navigation_map = navigation_map)
		self.model.reset()
		self.information_map = np.zeros_like(self.navigation_map)

		# Rendering variables #
		self.fig = None
		self.colors = ['red', 'blue', 'green', 'yellow', 'orange', 'purple', 'pink', 'brown', 'gray', 'olive', 'cyan', 'magenta']
		self.markers = ['o', 'v', '*', 'p', '>', 's', 'p', '*', 'h', 'H', 'D', 'd']

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



	def compute_coverage_mask(self, position: np.ndarray) -> np.ndarray:
		""" Obtain a circular mask centered in position with a radius of coverage_radius """

		mask = np.zeros_like(self.navigation_map)
		
		# Compute the positions 
		x, y = np.meshgrid(np.arange(self.navigation_map.shape[1]), np.arange(self.navigation_map.shape[0]))
		x = x - position[0]
		y = y - position[1]
		r = np.sqrt(x**2 + y**2)

		# Create the mask
		mask[r <= self.coverage_radius] = 1

		return mask.astype(bool)

	def step(self, new_positions: np.ndarray):
		"""
		Realiza un paso en el entorno: mueve los agentes, calcula las distancias
		y actualiza la recompensa según las restricciones de altura e inclinación.
		
		Args:
			new_positions (np.ndarray): Nuevas posiciones de los agentes.
		
		Returns:
			tuple: Recompensas y estado de finalización (done).
		"""
		# Check if the new positions are neighbors of the current positions of the agents
		for i in range(self.n_agents):

			if new_positions[i] == -1:
				continue

			if new_positions[i] not in list(self.G.neighbors(self.agent_positions[i])):
				raise ValueError('The new positions are not neighbors of the current positions of the agents')

			# Compute the distance traveled by the agents using the edge weight
			self.agent_distances[i] += self.G[self.agent_positions[i]][new_positions[i]]['weight']  # Peso relacionado con la batería

		# Update the positions of the agents
		self.agent_positions = new_positions.copy()

		# Recompensa: verificación de obstáculos y caminos transitables
		rewards = np.zeros(self.n_agents)

		for i in range(self.n_agents):
			current_position = self.agent_positions[i]

			if current_position == -1:
				continue

			# Verificar si el agente puede cruzar el obstáculo
			neighbors = list(self.G.neighbors(current_position))
			reward = 0

			for neighbor in neighbors:
				current_height = self.G.nodes[current_position]['height']
				neighbor_height = self.G.nodes[neighbor]['height']
				
				# Calcular la diferencia de altura (inclinación)
				height_diff = abs(current_height - neighbor_height)

				if height_diff > 1:  # Si la inclinación es mayor a 1m, no puede pasar
					reward -= 1  # Penalización por no poder cruzar el obstáculo
				elif height_diff <= 1:  # Si la inclinación es aceptable
					reward += 1  # Recompensa por cruzar el obstáculo

			rewards[i] = reward

		# Actualizar los waypoints de los agentes
		for agent_id, new_position in enumerate(new_positions):
			if new_position != -1:
				# Añadir la posición del nodo al waypoint
				self.waypoints[agent_id].append(list(self.G.nodes[new_position]['position']))

		# Actualización de los mapas y otras variables
		self.update_maps()

		# Comprobar si todos los agentes han superado la distancia máxima
		done = np.asarray([agent_distance > self.max_distance for agent_distance in self.agent_distances.values()]).all()

		# También verificar si algún agente ha llegado a una posición no válida (-1)
		done = done or np.asarray([agent_position == -1 for agent_position in self.agent_positions]).all()

		# Retornar las recompensas y el estado de finalización (done)
		return rewards, done

	def evaluate_path(self, multiagent_path: dict, render=False) -> dict:
		""" Evaluate a path considering the new terrain passability conditions """

		self.reset()

		if render:
			self.render()
		
		done = False
		t = 0

		# Initialize rewards based on node importance
		final_rewards = np.array([0] * len(self.G.nodes), dtype=float)
		
		while not done:
			next_positions = np.zeros_like(self.agent_positions)

			for i in range(self.n_agents):
				if t < len(multiagent_path[i]):
					next_positions[i] = multiagent_path[i][t]
				else:
					next_positions[i] = -1

			new_rewards, done = self.step(next_positions)

			# Update the final rewards for all agents
			final_rewards += new_rewards

			if render:
				self.render()

			t += 1

		return final_rewards


	def render(self):
		if self.fig is None:
			self.fig, self.ax = plt.subplots(1, 2, figsize=(10, 10))

			# Display the information map and ground truth (if available)
			self.d1 = self.ax[0].imshow(self.information_map, cmap='gray', vmin=0, vmax=1)
			self.d2 = self.ax[1].imshow(self.ground_truth.read(), cmap='gray', vmin=0, vmax=1)

			# Initialize agent render positions
			self.agents_render_pos = []
			for i in range(self.n_agents):
				# Obtain the agent position from the graph node to actual coordinates
				agent_position_coords = self.G.nodes[self.agent_positions[i]]['position']
				self.agents_render_pos.append(self.ax[0].plot(agent_position_coords[0], agent_position_coords[1], color=self.colors[i], marker=self.markers[i], markersize=10, alpha=0.35)[0])

		else:
			# Update agent trajectories in the render
			for i in range(self.n_agents):
				traj = np.asarray(self.waypoints[i])
				# Plot the trajectory of the agent
				self.agents_render_pos[i].set_data(traj[:, 0], traj[:, 1])

			# Update the maps being shown
			self.d1.set_data(self.information_map)
			self.d2.set_data(self.ground_truth.read())
		
		# Redraw the canvas
		self.fig.canvas.draw()
		plt.pause(0.01)


def create_graph_from_map(navigation_map: np.ndarray, resolution: int, height_map: np.ndarray):
	""" Create a graph from a navigation map, considering obstacles' height and slope passability """

	# Obtain the scaled navigation map
	scaled_navigation_map = navigation_map[::resolution, ::resolution]

	# Obtain the positions of the nodes
	visitable_positions = np.column_stack(np.where(scaled_navigation_map == 1))

	# Create the graph
	G = nx.Graph()

	# Add the nodes
	for i, position in enumerate(visitable_positions):
		# Store the position in original resolution
		G.add_node(i, position=position[::-1]*resolution, coords=position*resolution)
		
		# Set default node attributes
		nx.set_node_attributes(G, {i: 1}, 'value')
		
		# Get the importance values for this position
		x_index, y_index = position * resolution
		
		# Set default reward
		nx.set_node_attributes(G, {i: 0}, 'rh_reward')
		
		# Get the height value for this position
		height_value = height_map[x_index, y_index]
		nx.set_node_attributes(G, {i: height_value}, 'height')
	# for i, position in enumerate(visitable_positions):
	# 	for j, other_position in enumerate(visitable_positions):
	# 		if i != j:
	# 			if np.linalg.norm(position - other_position) <= np.sqrt(2):
	# 				G.add_edge(i, j, weight=np.linalg.norm(position - other_position)*resolution)
	# Add the edges, considering passability based on obstacle height and slope
	for i, position in enumerate(visitable_positions):
		for j, other_position in enumerate(visitable_positions):
			if i != j:
				# Calculate the distance between the positions
				distance = np.linalg.norm(position - other_position)
				
				# Check if the terrain between these positions is passable
				x1, y1 = position * resolution
				x2, y2 = other_position * resolution
				
				height_1 = height_map[x1, y1]
				height_2 = height_map[x2, y2]
				
				# Check height difference for slope passability
				height_diff = abs(height_1 - height_2)
				
				# If the height difference is too large (more than 1 meter), the slope is not passable
				if height_diff > 1:
					continue
				
				# If the obstacle height is greater than 3 meters, it is impassable
				if height_1 > 3 or height_2 > 3:
					continue
				
				# Add edge with weight proportional to the distance
				if distance <= np.sqrt(2):
					G.add_edge(i, j, weight=distance * resolution)

	return G



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


def random_shorted_path(G: nx.Graph, p0: int, p1:int) -> list:

	random_G = G.copy()
	for edge in random_G.edges():
		random_G[edge[0]][edge[1]]['weight'] = np.random.rand()

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

def create_multiagent_random_paths_from_nodes(G, initial_positions, distance, final_positions=None):

		if final_positions is not None:
			multiagent_path = {agent_id: create_random_path_from_nodes(G, initial_positions[agent_id], distance, final_positions[agent_id]) for agent_id in range(len(initial_positions))}
		else:
			multiagent_path = {agent_id: create_random_path_from_nodes(G, initial_positions[agent_id], distance) for agent_id in range(len(initial_positions))}
		#print(multiagent_path)
		return multiagent_path




if __name__ == '__main__':

	np.random.seed(0)

	navigation_map = np.genfromtxt('../maps/output/heightmap_traversability.txt', delimiter=' ')
	high_map= np.genfromtxt('../maps/output/heightmap_z_values.txt', delimiter=' ')
	N_agents = 1
	initial_positions = np.array([10,20,30,40])[:N_agents]
	#final_positions = np.genfromtxt('../maps/output/interest_points.txt', delimiter=' ')
	final_positions = np.array([40,20,30,40])[:N_agents]

	scale = 50

	environment = PatrollingGraphRoutingProblem(navigation_map = navigation_map,
											 	high_map=high_map,
												n_agents=N_agents, 
												initial_positions=initial_positions,
												final_positions=final_positions,
												scale=scale,
												max_distance=350.0,
												ground_truth='shekel'
												

	)


	# path = create_multiagent_random_paths_from_nodes(environment.G, initial_positions, 150, final_positions)
	path = create_multiagent_random_paths_from_nodes(environment.G, initial_positions, 150,final_positions)


	#environment.evaluate_path(path, render=True)
	
	a=environment.evaluate_path(path, render=True)
	reward=[]
	for valor in a:
		reward=valor
		print('f',reward)
	print('d',a)
	#new_reward.append(tuple(new_rewards))
	# environment.evaluate_path(path_crossed, render=True)

	plt.pause(1000)

	# Plot the graph to visualize the crossing
	fig, axs = plt.subplots(2, 2, figsize=(10, 5))
	plot_graph(environment.G, path=path[0], draw_nodes=True, ax=axs[0,0])
	plot_graph(environment.G, path=path[1], draw_nodes=True, ax=axs[0,1], cmap_str='Greens')
	# plot_graph(environment.G, path=path[2], draw_nodes=True, ax=axs[1,0], cmap_str='Blues')
	# plot_graph(environment.G, path=path_crossed[0], draw_nodes=True, ax=axs[1,0])
	# plot_graph(environment.G, path=path_crossed[1], draw_nodes=True, ax=axs[1,1], cmap_str='Greens')
	plt.show()




















