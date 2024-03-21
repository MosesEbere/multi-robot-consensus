## Main ROS Node: `formation_ctl.py`

The `formation_ctl.py` script is the core of the Multi-Robot Consensus Package, coordinating the multi-robot system's consensus tasks. It manages the dynamic formation of robots, obstacle avoidance, and the integration of stubborn agents within the swarm. This node utilizes a range of utilities and custom classes to achieve precise formation control and robust navigation.

### Features and Functionalities

- **Dynamic Formation Control**: Configurable initial and final formations leveraging graph theory for precise coordination.
- **Stubborn Agent Management**: Incorporation of stubborn agents with custom positions, motion types, and goals, enhancing scenario complexity.
- **Obstacle Avoidance**: Utilizes potential fields and steering behaviors to navigate around obstacles.
- **Velocity Scaling**: Adjusts agent velocities for smooth and controlled movements.
- **Graph-based Coordination**: Employs a graph structure to manage agent interactions, communication ranges, and topology switching.
- **Visualization**: Supports real-time visualization of agent positions, trajectories, and formations in ROS RViz.

### Implementation Highlights

- **Formation and Task Management**: The node dynamically selects formation and task functions based on configuration parameters, allowing for flexible adaptation to various scenarios.
- **Normalize Positions**: Ensures that agent positions are normalized within the map dimensions, facilitating consistent behavior in constrained environments.
- **Velocity Calculation for Stubborn Agents**: Calculates velocities for stubborn agents based on their goal updates, ensuring they move according to the specified motion patterns.
- **Prioritized Acceleration**: Allows for the prioritization of different behaviors (e.g., formation maintenance, separation, and obstacle avoidance) to achieve balanced and effective agent control.


### ROS Topics and Parameters

- **Subscribers**: Listens to `/robot_{}/odom` for odometry information of each agent.
- **Publishers**: Sends commands via `/robot_{}/cmd_vel` to control the velocity of each agent. Also publishes visualization markers for RViz.
- **Parameters**: Utilizes a wide range of ROS parameters to configure the behavior of the multi-robot system, including formation shapes, agent counts, communication ranges, and more.

### Running the Node

To run the `formation_ctl.py` node in your ROS environment, follow these steps:

1. Ensure the Multi-Robot Consensus Package is built and sourced in your ROS workspace.
2. Launch the node using a ROS launch file or directly via `rosrun`:

   ```bash
   roslaunch multi-robot-consensus formation_ctl.launch
   ```

   or

   ```bash
   rosrun multi-robot-consensus formation_ctl.py
   ```

3. Adjust the ROS parameters as needed to customize the behavior of the multi-robot system.
