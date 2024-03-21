## Stubborn Agent Management: `stubborn_agent.py`

The `stubborn_agent.py` script defines the `StubbornAgent` class, designed to manage stubborn agents in the multi-robot consensus framework. These agents can follow unique motion patterns, such as circular, lawnmower, or random walks, independent of the consensus protocol. This capability allows for testing the robustness of consensus algorithms against non-cooperative elements within the swarm.

### Key Functionalities

- **Motion Patterns**: Supports various motion patterns for stubborn agents, including circular, lawnmower, zigzag, and random walks, allowing for diverse simulation scenarios.
- **Goal Update**: The `update_goal` method dynamically updates the goal positions of stubborn agents based on the specified motion type.
- **Position Normalization**: Incorporates a method to normalize agent positions within the simulation environment, ensuring agents remain within designated bounds.
- **Independent Management**: Designed to operate somewhat independently from the main Graph class, allowing for easier integration and management of stubborn agents in simulations.

### Utilization in Simulations

- **Initialization**: Instantiate the StubbornAgent class with the initial center position, bounds of the simulation area, and the desired number of stubborn agents.
- **Motion Configuration**: Choose the motion pattern for stubborn agents by specifying it in the `update_goal` method call, allowing for dynamic behavior during simulations.
- **Integration**: While the StubbornAgent class operates independently, it's designed to be easily integrated with the Graph class, enabling the simulation of mixed cooperative and non-cooperative agent behaviors.

Example initialization and update call:

```python
from stubborn_agent import StubbornAgent

# Initialize the StubbornAgent class for a single stubborn agent in a 100x100 area
stubborn_agent = StubbornAgent(stubborn_center=(50, 50), bounds=(100, 100), n=1, positions={}, k=1)

# Update the stubborn agent's goal with a circular motion pattern
stubborn_agent.update_goal(motion_type='circular')
```

This snippet demonstrates setting up a stubborn agent with a circular motion pattern in a defined area, showcasing the flexibility and ease of integrating complex agent behaviors into your simulations.

### Advanced Motion Configurations

Experiment with various motion patterns and parameters to simulate different scenarios and challenges within your multi-robot systems. Adjusting parameters such as motion type, step size, and pattern complexity can yield insights into the resilience and adaptability of consensus protocols against dynamic and unpredictable elements.