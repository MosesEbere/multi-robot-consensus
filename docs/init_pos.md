## Formation Initialization and Helper Functions: `init_pos.py`

The `init_pos.py` script is integral to the Multi-Robot Consensus Package, facilitating the initialization of robot formations, including line, square (rectangle), circle, and polygon formations. It also provides functionalities for distributing stubborn agents within these formations and includes helper functions for creating various test scenarios and formations based on graph structures.

### Key Features

- **Flexible Formation Definitions**: Supports multiple formation types, such as line, square, circle, polygon, and triangle, with configurable parameters for size, spacing, and center.
- **Stubborn Agent Integration**: Ability to include and distribute stubborn agents within formations, enhancing scenario complexity and testing robustness.
- **Test Scenarios**: Includes functions for creating test graphs (`t1` to `t6`) and random or circle-based formations, aiding in the development and debugging of consensus algorithms.

### Formation Class

The `Formation` class allows for the generation of predefined formations with an emphasis on flexibility and configuration:

- **Methods**: Includes methods for generating line, square, circle, polygon, and triangle formations. Each method has parameters for customization and an option to include stubborn agents.
- **Usage**: Instantiated with the number of agents and stubborn agents, the class methods return positions that define the desired formation geometry.

### Helper Functions

- **Test Graph Functions (`t1` to `t6`)**: Provide predefined graph structures for testing different network configurations and behaviors.
- **`create_random` and `create_circle`**: Generate random or circle-based formations within specified bounds or with a given radius, aiding in the creation of diverse simulation environments.
- **Utility Functions**: Include `calculate_area_polygon` for calculating the area of a polygonal formation and `calculate_centroid` for finding the centroid of a polygon, useful for analytical purposes and formation control.

### Example Usage

Below is an example of how to generate a circle formation using the `Formation` class:

```python
from init_pos import Formation

# Initialize the Formation class for 10 agents with 1 stubborn agent
formation_generator = Formation(n_agents=10, k_agents=1)

# Generate a circle formation with a specified center and radius
circle_positions = formation_generator.circle_formation(center=(0.100, 0.100), radius=0.50, stubborn_agents=True)

# Output: List of positions for each agent in the circle formation
print(circle_positions)
```

This example demonstrates the simplicity of generating agent formations, which can then be utilized within simulation environments or for testing consensus algorithms.