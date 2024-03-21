## Potential Field Obstacle Avoidance: `potential_field.py`

The `potential_field.py` script implements the `PotentialField` class, leveraging the concept of potential fields for obstacle avoidance in multi-robot systems. By creating a virtual force field around obstacles based on their proximity, this method guides robots around obstacles smoothly, ensuring safe navigation without direct collisions.

### Key Features and Methods

- **Brushfire Algorithm**: Utilizes the brushfire algorithm to calculate the distance from obstacles, generating a gradient used to direct robots away from obstacles.
- **Force Field Creation**: Generates a force field from the brushfire map, where each point in the space exerts a force on the robot, steering it towards free space.
- **Dynamic Adaptation**: Adjusts the force field in real-time based on the robot's position and the configuration of obstacles, allowing for flexible navigation in dynamic environments.
- **Map Integration**: Supports setting and updating the occupancy map, resolution, and origin, ensuring that the obstacle avoidance mechanism is accurately aligned with the environment.

### Implementation Details

- **Initialization**: The class is initialized with parameters such as the radius around the robot used to check for obstacle occupancy, providing a customizable safety margin.
- **Map Setting**: The `set` method allows for the introduction of the occupancy map and its properties, which are essential for calculating the potential fields accurately.
- **Potential Field Calculation**: The `_potential_field` method calculates the potential field vector for the robot's current position, guiding its movement within the environment.

### Example Usage

Below is an example showing how to initialize the `PotentialField` class and use it to navigate around obstacles:

```python
from potential_field import PotentialField

# Initialize the PotentialField class with an obstacle radius parameter
potential_field = PotentialField(obstacle_radius=1.0)

# Set the occupancy map, resolution, and origin for the potential field calculation
potential_field.set(map_data, map_resolution, map_origin)

# Calculate the potential field vector for the current robot position
force_vector = potential_field._potential_field(robot_position)

# Apply the calculated force vector to the robot's control system
robot.apply_force(force_vector)
```

This snippet demonstrates setting up the PotentialField class with a given map and using it to compute and apply a force vector for obstacle avoidance.