import copy
import numpy as np


class StubbornAgent():
    """
    A class representing a stubborn agent in a graph.

    Attributes:
        stubborn_goal (tuple): The current goal position of the agent.
        time (int): The current time step.

    Methods:
        update_goal(motion_type): Updates the stubborn goal based on the given motion type.
        circular_motion(): Updates the stubborn goal using circular motion.
        lawnmower_motion(): Updates the stubborn goal using lawnmower motion.
        zigzag_motion(): Updates the stubborn goal using zigzag motion.
        random_walk(): Updates the stubborn goal using random walk motion.
    """

    def __init__(self, stubborn_center, bounds, n, positions, k=4, map_dimensions=None):
        self.stubborn_goal = [(0, 0) for _ in range(k)]
        self.stubborn_center = stubborn_center
        self.k = k
        self.time = [0 for _ in range(k)]
        # self.time_delta = [max(i+0.23, 2) for i in range(k)]  # different time delta for each stubborn agent
        self.time_delta = [i for i in np.arange(0.1, k+1, 0.01)]
        self.bounds = bounds
        self.positions = positions
        self.n = n
        self.map_dimensions = map_dimensions
        # raise ValueError

    def lawnmower_motion(self):
        """
        Updates the stubborn goal using lawnmower motion.
        """
        # TODO: Fix the lawnmower motion. It's not working as expected.
        map_x, map_y = self.bounds  # dimensions of the map
        line_spacing = 50  # distance between the lines
        step_size = 1  # distance moved in each update
        time_displacement = 50  # time displacement for the stubborn agents

        # Update the positions of the stubborn agents
        for i in range(self.n - len(self.stubborn_goal), self.n):
            # Calculate the total distance moved
            total_distance = (self.time[i - (self.n - len(self.stubborn_goal))] + i * time_displacement) * step_size

            # Calculate the current line and position within the line
            line = int(total_distance // (map_x + line_spacing))
            pos_within_line = total_distance % (map_x + line_spacing)

            # Determine the direction and position based on the line number and position within the line
            if pos_within_line <= map_x:  # moving horizontally
                if line % 2 == 0:  # even line number
                    new_position = (pos_within_line, line * line_spacing)
                else:  # odd line number
                    new_position = (map_x - pos_within_line, line * line_spacing)
            else:  # moving vertically to the next line
                new_position = (self.stubborn_goal[i - (self.n - len(self.stubborn_goal))][0], line * line_spacing + pos_within_line - map_x)

            # Adjust to the correct coordinate system
            new_position = (new_position[0], map_y - new_position[1])

            # Update the position and goal of the stubborn agent
            self.positions[i] = new_position
            self.stubborn_goal[i - (self.n - len(self.stubborn_goal))] = new_position

            self.time[i - (self.n - len(self.stubborn_goal))] += self.time_delta[i - (self.n - len(self.stubborn_goal))]  # increment the time with different time delta

    def circular_motion(self):
        """
        Updates the stubborn goal using circular motion.
        """
        center = self.stubborn_center
        radius = 0.000
        step_size = 2 * np.pi / 100  # complete a circle every 100 time steps

        positions = copy.deepcopy(self.positions)
        
        # Update the goal and position of each stubborn agent
        for i in range(len(self.stubborn_goal)):
            angle = self.time[i] * step_size
            self.stubborn_goal[i] = (center[0] + np.cos(angle) * radius, center[1] + np.sin(angle) * radius)
            # self.positions[self.n - 1 + i] = self.normalize_positions((center[0] + np.cos(angle) * radius, center[1] + np.sin(angle) * radius))
            positions[self.n - 1 + i] = (center[0] + np.cos(angle) * radius, center[1] + np.sin(angle) * radius)
            self.time[i] += self.time_delta[i]

        # Return only the positions of the stubborn agents in the self.positions dictionary
        return [positions[i] for i in range(self.n - len(self.stubborn_goal), self.n)]

    def normalize_positions(self, positions) -> list:
        """
        Normalize positions to fall within the map dimensions specified during initialization.

        Args:
            positions (tuple): A tuple of (x, y) tuples representing positions.

        Returns:
            list: A list of normalized (x, y) tuples.
        """
        # TODO: Correct this normalization. It's not working as expected.
        normalized_positions = [positions[0] / self.bounds[1] * (self.map_dimensions['max_x'] - self.map_dimensions['min_x']) + self.map_dimensions['min_x'],
                                 (self.bounds[0] - positions[1]) / self.bounds[0] * (self.map_dimensions['max_y'] - self.map_dimensions['min_y']) + self.map_dimensions['min_y']]
        return normalized_positions

    def update_goal(self, motion_type):
        """
        Updates the stubborn goal based on the given motion type.

        Args:
            motion_type (str): The type of motion to be applied.

        Raises:
            ValueError: If the motion type is unknown.
        """
        # NOTE: Implememt the POSITION return for each of the motion types below. 
        if motion_type == 'circular':
            return self.circular_motion()
        elif motion_type == 'lawnmower':
            return self.lawnmower_motion()
        elif motion_type == 'zigzag':
            return self.zigzag_motion()
        elif motion_type == 'randomwalk':
            return self.random_walk()
        elif motion_type == 'spiral':
            return self.spiral_motion()
        else:
            raise ValueError(f"Unknown motion type: {motion_type}")
        
    def set_stubborn_center(self, center):   
        self.stubborn_center = center


    # def zigzag_motion(self):
    #     """
    #     Updates the stubborn goal using zigzag motion.
    #     """
    #     map_x, map_y = self.bounds  # dimensions of the map
    #     line_spacing = 80  # distance between the lines
    #     step_size = 1  # distance moved in each update

    #     # Calculate the total distance moved
    #     total_distance = self.time * step_size

    #     # Calculate the current line and position within the line
    #     line = int(total_distance // (map_x + line_spacing))
    #     pos_within_line = total_distance % (map_x + line_spacing)

    #     # Determine the direction and position based on the line number and position within the line
    #     if pos_within_line <= map_x:  # moving horizontally
    #         if line % 2 == 0:  # even line number
    #             self.stubborn_goal = (pos_within_line, line * line_spacing)
    #         else:  # odd line number
    #             self.stubborn_goal = (map_x - pos_within_line, line * line_spacing)
    #     else:  # moving vertically to the next line
    #         self.stubborn_goal = (self.stubborn_goal[0], line * line_spacing + pos_within_line - map_x)

    #     self.positions[self.n-1] = self.stubborn_goal
    #     self.time += 3  # increment the time

    # def random_walk(self):
    #     """
    #     Updates the stubborn goal using random walk motion.
    #     """
    #     map_x, map_y = self.bounds  # dimensions of the map
    #     dx, dy = random.choice([(0, 1), (0, -1), (1, 0), (-1, 0)])  # random direction
    #     self.stubborn_goal = (self.stubborn_goal[0] + dx, self.stubborn_goal[1] + dy)
    #     self.stubborn_goal = (max(0, min(map_x, self.stubborn_goal[0])), max(0, min(map_y, self.stubborn_goal[1])))

    # def spiral_motion(self):
    #     """
    #     Updates the stubborn goal using spiral motion.
    #     """
    #     center = self.stubborn_center
    #     a, b = 0.5, 0.2  # these control the size and tightness of the spiral
    #     step_size = 2 * np.pi / 100  # complete a full spiral every 100 time steps
    #     angle = self.time * step_size
    #     r = a * np.exp(b * angle)
    #     x = center[0] + np.cos(angle) * r
    #     y = center[1] + np.sin(angle) * r

    #     # Ensure the stubborn_goal is within the bounds
    #     map_x, map_y = self.bounds
    #     x = max(0, min(map_x, x))
    #     y = max(0, min(map_y, y))

    #     self.stubborn_goal = (x, y)
    #     self.positions[self.n-1] = self.stubborn_goal
    #     self.time += self.time_delta