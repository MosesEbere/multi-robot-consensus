## Utility Functions: `utils.py`

The `utils.py` script includes a set of utility functions and classes that support the operation of the multi-robot consensus package. Among these utilities, the `TextColors` class enables colored console outputs for better log visibility, and the `combine_behaviors` function is crucial for calculating the net acceleration of agents based on multiple behavior inputs.

### TextColors Class

The `TextColors` class defines ANSI escape codes for various console text colors, facilitating more readable and distinguishable log messages. This class includes colors like red, green, yellow, blue, magenta, cyan, and white, along with a reset code to return the console color to its default.

Example usage for console output:

```python
print(f"{TextColors.RED}Error: Connection lost{TextColors.RESET}")
```

This example demonstrates how to use the TextColors class to print an error message in red, enhancing its visibility in the console.

### combine_behaviors Function

The `combine_behaviors` function calculates the net weighted acceleration of an agent by combining different behaviors (e.g., formation keeping, separation, and obstacle avoidance). This function is essential for determining the agent's movement based on a prioritized list of behaviors and constraints like maximum acceleration.

Parameters:
- `behavior_list`: A list of behavior names to be considered.
- `force_dict`: A dictionary mapping behavior names to their respective force vectors.
- `use_prioritized_acc`: A boolean indicating whether to use prioritized acceleration.
- Additional keyword arguments like `max_acc` and `all_behaviors`.

Example calculation:

```python
net_acc = combine_behaviors(behavior_list=my_behaviors, force_dict=my_force_dict, use_prioritized_acc=True, max_acc=1.0, all_behaviors=my_all_behaviors)
```

This function plays a critical role in the decision-making process for agent movements, allowing for dynamic adjustments based on the situation and predefined priorities.

### Utilization in the Package

Incorporate the `TextColors` class into your logging strategy to improve log readability and diagnostics. Use the `combine_behaviors` function to manage agent movements effectively, taking into account multiple simultaneous behavioral inputs and constraints. These utilities are designed to enhance both the development experience and the performance of your multi-robot systems.