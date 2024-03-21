import numpy as np
import random
from random import randint

np.random.seed(42)
random.seed(15)
# random.seed(35) # Test zero stubborn nodes


class Formation:
    def __init__(self, n_agents, k_agents):
        self.n_agents = n_agents
        self.k_agents = k_agents

    def distribute_stubborn(self, positions, center):
        angles = np.linspace(0, 2 * np.pi, self.k_agents + 1, endpoint=False)
        radius = 0.5  # distance from the center to the stubborn agents
        for angle in angles[:-1]:
            positions.append((center[0] + np.cos(angle) * radius, center[1] + np.sin(angle) * radius))
        return positions

    def line_formation(self, start=(0.50, 0.50), spacing=0.10, stubborn_agents=True):
        positions = [(start[0] + i * spacing, start[1]) for i in range(self.n_agents + self.k_agents)]
        return positions
        # center = (start[0] + (self.n_agents + self.k_agents - 1) * spacing / 2, start[1])
        # if not stubborn_agents:
        #     return positions
        # return self.distribute_stubborn(positions, center)

    def square_formation(self, start=(0.50, 0.50), spacing=0.10, stubborn_agents=True):
        side_length = np.ceil(np.sqrt(self.n_agents))
        positions = [(start[0] + (i % side_length) * spacing, start[1] + (i // side_length) * spacing) for i in range(self.n_agents)]
        center = (start[0] + side_length * spacing / 2, start[1] + side_length * spacing / 2)
        if not stubborn_agents:
            return positions
        return self.distribute_stubborn(positions, center)

    def circle_formation(self, center=(0.100, 0.100), radius=0.50, stubborn_agents=True):
        angles = np.linspace(0, 2 * np.pi, self.n_agents, endpoint=False)
        positions = [(center[0] + np.cos(angle) * radius, center[1] + np.sin(angle) * radius) for angle in angles]
        if not stubborn_agents:
            return positions
        if self.k_agents == 1:
            positions.extend([center])
        else:
            positions = self.distribute_stubborn(positions, center)
        return positions

    def polygon_formation(self, center=(0.100, 0.100), radius=0.50, sides=3, stubborn_agents=True):
        angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)
        positions = [(center[0] + np.cos(angle) * radius, center[1] + np.sin(angle) * radius) for angle in angles]
        if not stubborn_agents:
            return positions
        if self.k_agents == 1:
            positions.extend([center])
        else:
            positions = self.distribute_stubborn(positions, center)
        return positions

    def triangle_formation(self, start=(0.50, 0.50), spacing=0.10, stubborn_agents=True):
        rows = int((-1 + np.sqrt(1 + 8 * self.n_agents)) / 2)
        remaining = self.n_agents - rows * (rows + 1) // 2
        positions = []
        for i in range(rows):
            for j in range(i + 1):
                positions.append((start[0] + j * spacing, start[1] + i * spacing))
        for i in range(remaining):
            positions.append((start[0] + i * spacing, start[1] + rows * spacing))
        if not stubborn_agents:
            return positions
        center = (start[0] + rows * spacing / 2, start[1] + rows * spacing / 2)
        if self.k_agents == 1:
            positions.extend([center])
        else:
            positions = self.distribute_stubborn(positions, center)
        return positions
    

def calculate_area_polygon(n_sides, radius):
    """This function calculates the area of an n-sided polygon given the radius of the """
    side_length = 2 * radius * np.sin(np.pi/n_sides)
    area = n_sides/2 * side_length * radius
    return area


def calculate_centroid(size, vertices):
    """This function can calculate the centroid of a polygon"""

    
# NOTE: Zero indexing is used for all the nodes
def t1(G):
    positions = {}

    G.add_node(0, pos=(150, 150))
    positions[0] = (150,150)

    G.add_node(1, pos=(450, 150))
    positions[1] = (400,150)

    G.add_node(2, pos=(400, 400))
    positions[2] = (400,400)

    G.add_node(3, pos=(150, 400))
    positions[3] = (150,400)

    G.add_edge(0, 1)
    G.add_edge(1, 0)
    G.add_edge(1, 2)
    G.add_edge(2, 1)
    G.add_edge(2, 3)
    G.add_edge(3, 2)
    G.add_edge(3, 1)
    G.add_edge(1, 3)
    
    return G, positions


def t2(G):
    positions = {}

    G.add_node(0, pos=(150, 150))
    positions[0] = (150,150)

    G.add_node(1, pos=(450, 150))
    positions[1] = (400,150)

    G.add_node(2, pos=(400, 400))
    positions[2] = (400,400)

    G.add_node(3, pos=(150, 400))
    positions[3] = (150,400)

    # G.add_edge(1, 4)
    # G.add_edge(4, 3)
    # G.add_edge(3, 2)

    # G.add_edge(1, 2)
    G.add_edge(1, 0)
    # G.add_edge(2, 3)
    # G.add_edge(3, 2)
    G.add_edge(2, 3)
    # G.add_edge(4, 3)
    
    return G, positions

def t3(G):
    positions = {}

    G.add_node(0, pos=(150, 150))
    positions[0] = (150,150)

    G.add_node(1, pos=(450, 150))
    positions[1] = (400,150)

    G.add_node(2, pos=(400, 400))
    positions[2] = (400,400)

    G.add_node(3, pos=(150, 400))
    positions[3] = (150,400)

    G.add_edge(0, 1)
    G.add_edge(2, 3)
    G.add_edge(1, 0)
    G.add_edge(3, 2)
    
    return G, positions

def t4(G):
    positions = {}

    G.add_node(0, pos=(150, 150))
    positions[0] = (150,150)

    G.add_node(1, pos=(450, 150))
    positions[1] = (400,150)

    G.add_node(2, pos=(400, 400))
    positions[2] = (400,400)

    G.add_node(3, pos=(150, 400))
    positions[3] = (150,400)

    G.add_edge(0, 1)
    G.add_edge(0, 3)
    G.add_edge(2, 1)
    G.add_edge(2, 3)
    
    return G, positions

def t5(G):
    positions = {}

    G.add_node(0, pos=(150, 150))
    positions[0] = (150,150)

    G.add_node(1, pos=(450, 150))
    positions[1] = (400,150)

    G.add_node(2, pos=(400, 400))
    positions[2] = (400,400)

    G.add_node(3, pos=(150, 400))
    positions[3] = (150,400)

    G.add_edge(0, 1)
    G.add_edge(1, 2)
    G.add_edge(2, 3)
    G.add_edge(3, 0)
    
    return G, positions

def t6(G):
    positions = {}

    G.add_node(0, pos=(150, 150))
    positions[0] = (150,150)

    G.add_node(1, pos=(450, 150))
    positions[1] = (400,150)

    G.add_node(2, pos=(400, 400))
    positions[2] = (400,400)

    G.add_node(3, pos=(150, 400))
    positions[3] = (150,400)

    G.add_edge(0, 2)
    G.add_edge(2, 1)
    G.add_edge(1, 0)
    G.add_edge(2, 3)
    
    return G, positions


def create_random(G, n, bounds):
    """
    Initializes a graph with n points randomly placed within the specified bounds.

    Args:
    - G: The graph representing the network.
    - n: The number of points (nodes).
    - width: The width of the area within which the points are placed.
    - height: The height of the area within which the points are placed.

    Returns:
    - G: The updated graph with nodes and positions.
    - positions: A dictionary with node positions.
    """

    width, height = bounds

    positions = {}

    # Create n random points within the given bounds
    for i in range(n):
        x = randint(0, width)
        y = randint(0, height)

        # Add the point as a node to the graph
        G.add_node(i, pos=(x, y))
        positions[i] = (x, y)

        # Optionally, create edges between nodes if needed
        # This is where you might add logic to connect nodes based on certain criteria

    return G, positions

def create_circle(G, n, r,origin=[0,0]):
    '''
    Creates n points at the circum of a circle of radius r at origin 
    '''
    # Calculate the angle between each point
    angle_step = 2 * np.pi / n

    positions = {}

    # Create n points on the circle and add them to the graph
    for i in range(n):
        # Calculate the angle for this point
        angle = i * angle_step

        # Calculate the x and y coordinates using trigonometry
        x = r * np.cos(angle)
        y = r * np.sin(angle)

        # Add the point as a node to the graph
        G.add_node(i, pos=(x, y))
        positions[i] = (x+origin[0],y+origin[1])

        # Create edges between adjacent points
        if i < n - 1:
            G.add_edge(i, i + 1)
            G.add_edge(i+1, i)
        else:
            # Connect the last point to the first point
            G.add_edge(i, 0)

    return G, positions