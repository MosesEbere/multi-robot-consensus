import math
import numpy as np
import networkx as nx
import copy
from itertools import product
from utils.utils import *
from utils.init_pos import t1, t2, t3, t4, t5, t6, create_random, create_circle

from utils.stubborn_agent import StubbornAgent


np.random.seed(42)


class Graph:
    
    def __init__(self, topo, params, comm_range=math.inf, directed=False, stubborn_nodes=[],  
                 move='random', comm_theta=np.pi, stubborn_pos=None, connection=[-1], map_dimensions=None, scale_vel=0.01):
        
        self.G = nx.DiGraph()

        self.directed = directed

        # unpack params
        self.n, r = params[:2]
        for i in range(self.n):
            self.G.add_node(i)

        origin, self.bounds = np.array(params[-2]), np.array(params[-1])
        self.comm_range = comm_range
        self.comm_theta = comm_theta
        self.stubborn_nodes = stubborn_nodes
        self.stubborn_pos = stubborn_pos
        self.time = 0
        self.connection = connection
        self.scale_vel = scale_vel

        #NOTE: I added this from Khawaja's code:
        self.positions = {i: None for i in range(self.n)}
        self.graph_init = False
        self.velocities = {i: (0,0) for i in range(self.n)}

        # Create the graph based on the topology
        # if topo=='t1':
        #     self.G, self.positions = t1(G)
        # elif topo=='t2':
        #     self.G, self.positions = t2(G)
        # elif topo=='t3':
        #     self.G, self.positions = t3(G)
        # elif topo=='t4':
        #     self.G, self.positions = t4(G)
        # elif topo=='t5':
        #     self.G, self.positions = t5(G)
        # elif topo=='t6':
        #     self.G, self.positions = t6(G)

        # NOTE: These were reintroduced specifically to setup edges when there are no stubborn agents.
        # This is because update_connectivity() is not called when there are no stubborn agents.
        if topo == 'random' and len(self.stubborn_nodes) == 0:
            self.G, self.positions =  create_random(self.G, self.n, self.bounds)
            # NOTE: Added to obtain relevant results.
            self.connect_all()
            # self.connect_k_closest(3)
        elif topo == 'circle' and len(self.stubborn_nodes) == 0:
            self.G, self.positions = create_circle(self.G, self.n, r,origin)


        # self.n = len(self.G.nodes()) # NOTE: Testing to ensure that the stubborn agent doesn't replace others. 

        print(f"{TextColors.GREEN}Robots initialized as {topo}{TextColors.RESET}")

        # self.is_graph_converge()

        # NOTE: The stubborn agent is added to the graph.
        # Add stubborn nodes, if any
        if len(self.stubborn_nodes) > 0:
            self.create_stubborn_nodes(k=len(self.stubborn_nodes), pos=copy.deepcopy(self.stubborn_pos), connection=connection)
            self.stubborn_center = copy.deepcopy(self.stubborn_pos)

            #print, create stubborn nodes
            print(f"{TextColors.GREEN}Stubborn nodes added at {self.stubborn_pos}{TextColors.RESET}")

            #print the length of n
            print(f"{TextColors.GREEN}Number of nodes: {self.n}{TextColors.RESET}")

        #initialize the stubborn agent class
        self.stubborn_agent = StubbornAgent(copy.deepcopy(self.stubborn_pos), self.bounds, self.n, copy.deepcopy(self.positions), 
                                            map_dimensions=map_dimensions if map_dimensions is not None else self.bounds, k=len(self.stubborn_nodes))

        self.velocities = {}   

    #_______________________  Movers  ____________________________

    def rendezvous(self, speed, tolerance):
        '''
        Args:
        - speed: The step size for the rendezvous update.
        - tolerance: The tolerance to determine when the nodes have converged.
        '''

        # Compute the out-degree matrix D (use out_degree for directed graphs)
        D = np.diag([self.G.out_degree(n) for n in self.G.nodes()])
        
        # Compute the adjacency matrix A
        A = nx.adjacency_matrix(self.G).toarray()

        # Initialize the identity matrix I
        I = np.eye(self.G.number_of_nodes())

        # Compute the matrix (I + D)^-1 * (I + A)
        update_matrix = np.linalg.inv(I + D) @ (I + A)

        # Initialize positions as a matrix for efficient computation
        positions_matrix = np.array([self.positions[node] for node in self.G.nodes()])

        # Perform the rendezvous
        new_positions_matrix = update_matrix @ positions_matrix
            
        positions_matrix = new_positions_matrix

        # Update the positions dictionary
        for i, node in enumerate(self.G.nodes()):
            self.positions[node] += speed * (new_positions_matrix[i] - self.positions[node])

    def discrete_consensus(self, speed, tolerance):
        '''
        Args:
        - tolerance: The tolerance to determine when the nodes have converged.
        '''

        # Compute the adjacency matrix A
        A = np.transpose(nx.adjacency_matrix(self.G).toarray())
        
        # Compute the out-degree for each node
        out_degree = np.array([self.G.out_degree(n) for n in self.G.nodes()])

        # Initialize the state vector for each node
        x = np.array([self.positions[node] for node in self.G.nodes()])

        x_next = np.copy(x)
        for i in range(len(self.G.nodes())):
            # Compute the weighted sum of the differences from neighbors
            weighted_diff_sum = np.sum(A[i, :].reshape(-1, 1) * (x - x[i]), axis=0)
            
            # Update the state of node i using the consensus algorithm
            x_next[i] = x[i] + (1 / (1 + out_degree[i])) * weighted_diff_sum

        # Update the positions dictionary with the final state
        for i, node in enumerate(self.G.nodes()):
            self.positions[node] += speed * (x_next[i] - self.positions[node])

    def formation_consensus(self, epsilon: np.array):
        """
        Perform formation consensus using the algorithm described in the paper.

        Args:
            - epsilon: A vector with the desired (formation) positions for each node.
        """

        # Compute the adjacency matrix A
        A = np.transpose(nx.adjacency_matrix(self.G).toarray())

        # Initialize the state vector for each node
        x = np.array([self.positions[node] for node in self.G.nodes()])

        # Initialize the speed vector for each node.
        speed = np.zeros_like(x)
        for i in range(len(self.G.nodes())):
            # Compute the weighted sum of the differences from neighbors
            speed[i] = np.sum(A[i, :].reshape(-1, 1) * (x - x[i] - (epsilon - epsilon[i])), axis=0)

        vels = {i: (0,0) for i in range(self.n)}

        for i, node in enumerate(self.G.nodes()):
            # Assuming unit timesteps. 
            vels[node] += self.scale_vel * speed[i] 
        
        self.velocities = vels
        return vels
    #_______________________  Graph Utils  _________________________

    def connect_all(self):
        for node1 in self.G.nodes():
            for node2 in self.G.nodes():
                if node1!=node2:
                    self.G.add_edge(node1, node2)

    def connect_k_closest(self, k):
        # Delete all edges
        self.G.remove_edges_from(list(self.G.edges()))
        for node in self.G.nodes():
            # Get the k closest nodes to the current node
          
            # Get the distances to all other nodes
            distances = {other_node: self.distance(node, other_node) for other_node in self.G.nodes() if other_node != node}
          
            # Sort the nodes by distance
            closest_nodes = sorted(distances.keys(), key=lambda x: distances[x])[:k]
          
            # Connect to them
            for neighbor in closest_nodes:
                self.G.add_edge(node, neighbor)
                self.G.add_edge(neighbor, node)

    def add_stubborn(self, goal, connection=[-1]):

        self.G.add_node(self.n, pos=(goal[0], goal[1]))
        self.positions[self.n] = (goal[0], goal[1])
        self.stubborn_nodes.append(self.n)

        # add connection from all nodes not to
        for node in self.G.nodes():
            if len(connection) == 1 and connection[0] == -1 and node != self.n:
                self.G.add_edge(self.n, node) # NOTE: The stubborn agent doesn't receive from other agents!
            elif node in connection and node != self.n:
                self.G.add_edge(self.n, node) # NOTE: The stubborn agent doesn't receive from other agents!
        self.n+=1    

        self.center = goal
        self.time = 0

    def update_stubborn_goal(self):
        # self.center = self.stubborn_goal
        self.radius = 10

        angle = 2 * np.pi * self.time
        self.stubborn_goal = (self.center[0] + self.radius * math.cos(angle), self.center[1] + self.radius * math.sin(angle))

        # goal = self.center + self.radius * np.array([np.cos(angle), np.sin(angle)])
        self.positions[self.n-1] = self.stubborn_goal
        self.time += 0.01

    def calculate_separation_forces(self, separation_threshold, repulsion_strength):
        """
        Calculate separation forces for nodes that are closer than a specified threshold.

        Args:
        - separation_threshold: The distance below which nodes will repel each other.
        - repulsion_strength: The strength of the repulsion force.

        Returns:
        - separation_forces: A dictionary of numpy arrays representing the separation force for each node.
        """
        # Initialize a dictionary to store the separation forces for each node
        separation_forces = {node: np.zeros(2) for node in self.G.nodes}

        edges = self.get_edges()            

        # NOTE: Apply the separation force also from the stubborn agent to others but not from others to the stubborn agent.

        # Iterate over all node pairs to calculate separation forces
        for node1, node2 in edges:
                # NOTE: Since the stubborn agents are VISIBLE to others, they should also repel others but not be repelled by others.
            # if not(self.is_stubborn(node1)) and not(self.is_stubborn(node2)):
                pos1 = np.array(self.positions[node1])
                pos2 = np.array(self.positions[node2])
                distance = np.linalg.norm(pos1 - pos2)

                # Check if nodes are closer than the separation threshold
                if distance < separation_threshold and distance > 0:  # Ensure distance is not zero to avoid division by zero
                    # Calculate a repulsion vector from node2 to node1
                    repulsion_vector = (pos1 - pos2) / distance  # Normalize the vector
                    # Calculate repulsion force magnitude inversely proportional to the distance
                    repulsion_force = repulsion_vector * repulsion_strength * (separation_threshold - distance) / separation_threshold

                    # Accumulate the repulsion force for both nodes, in opposite directions
                    if not(self.is_stubborn(node1)):
                        separation_forces[node1] += repulsion_force
                    separation_forces[node2] -= repulsion_force

        return separation_forces

    def update_connectivity(self):
        """
        Update the edges of the graph based on the distance between nodes.
        
        Args:
        - G: The graph representing the network.
        - positions: A dictionary with node positions.
        - threshold: The distance threshold for edge creation/removal.
        
        Returns:
        - G: The updated graph.
        """

        # Create a list of all possible node pairs without duplication
        all_node_pairs = [(node1, node2) for node1, node2 in product(self.G.nodes(), repeat=2) if node1 != node2]

        # NOTE: Added local perceptive field (range, theta) to determine connectivity. 
        #       Ensured that the stubborn agent doesn't receive connection from others. 
        # TODO: For now, all agents have a fixed theta of zero. Correct this in ROS. 

        # Go through all node pairs and update edges based on distance
        for node1, node2 in all_node_pairs:
            # Remove the connection from other nodes to the stubborn agent. A stubborn agent doesn't receive from others. 
            if node1 in self.stubborn_nodes:
                if self.G.has_edge(node2, node1): self.G.remove_edge(node2, node1)
                # NOTE: If the stubborn agent connection is defined, enforce it.
                if self.connection == [-1]: 
                    continue
                elif self.connection != [-1] and node2 not in self.connection:
                    if self.G.has_edge(node1, node2): self.G.remove_edge(node1, node2)
                    continue
                # continue
                continue
            dx = self.positions[node1][0] - self.positions[node2][0]
            dy = self.positions[node1][1] - self.positions[node2][1]
            alpha= math.atan2(dy,dx)
            angle_diff = abs(alpha - 0) # Assume that all boids start with an angle of 0. 
            angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
            if abs(angle_diff) < self.comm_theta / 2:
                distance = np.linalg.norm(np.array(self.positions[node2]) - np.array(self.positions[node1]))
                if distance <= self.comm_range:
                    self.G.add_edge(node2, node1)
                else:
                    if self.G.has_edge(node2, node1): self.G.remove_edge(node2, node1)
            else:
                if self.G.has_edge(node2, node1): self.G.remove_edge(node2, node1)

    def create_stubborn_nodes(self, k, pos=None, connection=[-1]):
        '''
        creates stubborn nodes at specified locs
        
        k: number of stubborn nodes
        pos: positions of these stubborn nodes
        '''
        pos = self.bounds/2 if pos is None else pos

        # Add new stubborn nodes
        for i in range(self.n, self.n + k):
            self.G.add_node(i, pos=pos)
            self.positions[i] = pos
            # self.stubborn_nodes.append(i)

            # add connection from all nodes not to
            for node in self.G.nodes():
                if len(connection) == 1 and connection[0] == -1 and node != self.n:
                    self.G.add_edge(self.n, node) # NOTE: The stubborn agent doesn't receive from other agents!
                elif node in connection and node != self.n:
                    self.G.add_edge(self.n, node) # NOTE: The stubborn agent doesn't receive from other agents!
        self.n += k

    def make_graph_undirected(self):

        all_edges = list(self.G.edges())

        # Iterate over the extracted edges and modify the graph
        for (u, v) in all_edges:
            print(u,v)
            self.G.add_edge(v, u)

        all_edges = list(self.G.edges())

    def distance(self, node1, node2):
        pos1 = self.positions[node1]
        pos2 = self.positions[node2]
        return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5

    #________________________  Getters, Setters  ____________________
    def get_positions(self):
        return self.positions
    
    def set_position(self,node,pos):
        '''
        node: (robot id)
        pos: (x,y) position of the robot
        '''
        self.positions[node] = pos
        # Handle the case where self.positions.values() contains None. This check should also work if the values are 
        # NumPy arrays (i.e., avoid checking ambiguous truth values).
        self.graph_init = all(element is not None for element in self.positions.values())

    def get_stubborn_nodes(self):
        return self.stubborn_nodes
    
    def show_nodes(self):
        print(self.G.nodes())

    def get_adj(self):
      return nx.to_numpy_array(self.G)
    
    def is_graph_init(self):
      return self.graph_init
    
    def is_stubborn(self,node):
        return True if node in self.stubborn_nodes else False

    def get_num_nodes(self):
        return self.n
    
    def get_velocities(self):
        return self.velocities
    
    def get_edges(self):
        return list(self.G.edges())
    
    def set_stubborn_pos(self, pos):
        self.stubborn_pos = pos

    def get_stubborn_pos(self):
        return self.stubborn_pos