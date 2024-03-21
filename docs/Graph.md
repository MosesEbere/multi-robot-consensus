## Graph Management: `Graph.py`

The `Graph.py` script introduces the Graph class, essential for modeling and manipulating the interactions and communications among robots in a swarm. It provides the backbone for implementing consensus algorithms, managing dynamic topologies, and incorporating stubborn agents, thereby offering a versatile tool for simulating distributed control strategies in multi-robot systems.

### Features of the Graph Class

- **Dynamic Topology Management**: Supports various topologies such as random, circle, and predefined test graphs, adaptable to the presence of stubborn agents.
- **Consensus Protocols Implementation**: Facilitates key consensus tasks including rendezvous, discrete consensus, and formation consensus, with customizable parameters for speed, tolerance, and desired formations.
- **Stubborn Agent Integration**: Allows the inclusion of stubborn agents with specific positions and connections, enhancing the simulation's complexity and realism.
- **Graph Utilities**: Offers functionalities for connecting nodes based on communication range and field of view, adding/removing edges dynamically, and managing stubborn nodes.
- **Visualization and Analysis**: Aids in visualizing graph structures, positions, and velocities for analysis and debugging purposes.

### Utilization and Integration

- **Initialization**: The class is initialized with parameters specifying the topology, communication range, stubborn agent details, and velocity scaling factors, among others.
- **Graph Construction**: Based on the initialization parameters, a graph is constructed, which can be static or dynamically updated to reflect changes in the robot swarm or environment.
- **Position and Velocity Management**: Robot positions and velocities are managed within the class, providing the necessary data for executing movement and consensus algorithms.
- **Stubborn Agents**: Special handling for stubborn agents, including their initialization, position updates, and their impact on the rest of the swarm's behavior.

### Example Use-Case

Below is a simplified example of initializing the Graph class for a swarm with a random topology and integrating a stubborn agent:

```python
from Graph import Graph

# Initialize the Graph with a random topology for 10 agents
graph = Graph(topo='random', params=[10, 4], comm_range=100, directed=False, stubborn_nodes=[10], stubborn_pos=[5, 5])

# Perform a consensus task such as rendezvous with specified parameters
graph.rendezvous(speed=0.1, tolerance=0.01)

# Update the graph to reflect movements or changes in topology
graph.update_connectivity()

# Access positions and velocities for analysis or further processing
positions = graph.get_positions()
velocities = graph.get_velocities()
```

This example illustrates the creation of a graph for a robot swarm, executing a consensus task, and then updating the graph based on the swarm's dynamics.