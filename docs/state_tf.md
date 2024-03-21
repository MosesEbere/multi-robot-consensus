## Pose Message Transformation: `state_tf.py`

The `state_tf.py` script includes the `pose_msg_to_state` function, designed to convert ROS odometry messages (`nav_msgs/Odometry`) into a 2D state vector. This utility function simplifies the process of extracting position, orientation, and velocity information from ROS messages, facilitating its use in navigation, control, and consensus algorithms within the multi-robot system.

### Function Details

- **Function**: `pose_msg_to_state(msg)`
- **Input**: `msg` - A ROS message of type `nav_msgs/Odometry`, containing the pose and twist information of a robot.
- **Output**: A list representing the 2D state vector `[x, y, theta, v_x, v_y, w]`, where `x` and `y` are the position coordinates, `theta` is the orientation angle, `v_x` and `v_y` are linear velocities, and `w` is the angular velocity.

This function is crucial for extracting and converting the odometry data from ROS messages into a format that is readily usable for calculations and algorithms related to robot movement and behavior.

### Example Usage

Below is an example showing how to use the `pose_msg_to_state` function to convert an odometry message into a state vector:

```python
from nav_msgs.msg import Odometry
from state_tf import pose_msg_to_state

# Assume `odom_msg` is an Odometry message received from a ROS topic
odom_msg = Odometry()

# Convert the Odometry message to a state vector
state_vector = pose_msg_to_state(odom_msg)

# The state vector now contains the robot's position, orientation, and velocities
print("State Vector:", state_vector)
```

This example demonstrates the straightforward conversion of odometry data into a state vector, enabling its direct application in control and navigation algorithms.