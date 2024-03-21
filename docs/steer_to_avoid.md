## Obstacle Avoidance: `SteerToAvoid` Class

The `steer_to_avoid.py` script encapsulates the `SteerToAvoid` class, a crucial component designed to enable robots to dynamically navigate around obstacles during their operation within the consensus framework. By calculating steering adjustments based on the robots' positions, velocities, and the surrounding environment, this class significantly enhances the autonomy and safety of the multi-robot system.

### Key Features and Methods

- **Dynamic Steering Adjustments**: Calculates real-time steering adjustments to navigate around obstacles based on the robot's current trajectory and sensory inputs.
- **Path Validation**: Utilizes a lookahead mechanism to predict potential collisions with obstacles and validates the path before making adjustments.
- **Obstacle Dilation**: Pre-processes the map data by dilating obstacles, providing a safer margin for robot navigation.
- **Steering Direction Calculation**: Determines the optimal steering direction to avoid obstacles while minimizing deviation from the intended path.
- **Flexible Obstacle Avoidance**: Supports multiple steering strategies, including immediate direction changes and planned path adjustments.

### Implementation Details

- **Initialization**: The class is initialized with parameters specifying obstacle radius, step angle for steering adjustments, and the maximum steering angle, tailoring the obstacle avoidance behavior to specific robot capabilities and environmental characteristics.
- **Map Setting**: Incorporates map data, including obstacle locations, resolution, and origin, allowing the class to make informed steering decisions.
- **Steering Force Calculation**: Upon detecting an imminent collision, calculates a steering force that guides the robot away from the obstacle, ensuring a smooth and safe navigation.

### Example Usage

Below is an example demonstrating how to set up the `SteerToAvoid` class for a robot and process sensory data to navigate around obstacles:

```python
from steer_to_avoid import SteerToAvoid

# Initialize the SteerToAvoid class with obstacle avoidance parameters
steer_to_avoid = SteerToAvoid(obstacle_radius=1.0, step_angle=np.radians(10), max_steering_angle=np.radians(90))

# Set the map data, resolution, and origin for obstacle avoidance
steer_to_avoid.set(map_data, map_resolution, map_origin)

# Calculate the steering force for the current robot position and velocity
steering_force = steer_to_avoid._steer_to_avoid(robot_position, robot_velocity)

# Apply the steering force to the robot's control system
robot.apply_force(steering_force)
```

This example showcases the basic setup and application of the SteerToAvoid class to implement obstacle avoidance in a robotic navigation system.