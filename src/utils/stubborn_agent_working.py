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

    def __init__(self, stubborn_center, bounds, n, positions):
        self.stubborn_goal = None
        self.stubborn_center = stubborn_center
        self.time = 0
        self.time_delta = 1
        self.bounds = bounds
        self.n = n
        self.positions = positions

    def update_goal(self, motion_type):
        """
        Updates the stubborn goal based on the given motion type.

        Args:
            motion_type (str): The type of motion to be applied.

        Raises:
            ValueError: If the motion type is unknown.
        """
        if motion_type == 'circular':
            self.circular_motion()
        elif motion_type == 'lawnmower':
            self.lawnmower_motion()
        elif motion_type == 'zigzag':
            self.zigzag_motion()
        elif motion_type == 'randomwalk':
            self.random_walk()
        elif motion_type == 'spiral':
            self.spiral_motion()
        else:
            raise ValueError(f"Unknown motion type: {motion_type}")
        
        return self.positions   

    def circular_motion(self):
        """
        Updates the stubborn goal using circular motion.
        """
        center = self.stubborn_center
        radius = 10
        step_size = 2 * np.pi / 100  # complete a circle every 100 time steps
        angle = self.time * step_size
        self.stubborn_goal = (center[0] + np.cos(angle) * radius, center[1] + np.sin(angle) * radius)
        self.positions[self.n-1] = self.stubborn_goal
        self.time += self.time_delta

    def lawnmower_motion(self):
        map_x, map_y = self.bounds  # dimensions of the map
        line_spacing = 50  # distance between the lines
        step_size = 1  # distance moved in each update

        # Calculate the total distance moved
        total_distance = self.time * step_size

        # Calculate the current line and position within the line
        line = int(total_distance // (map_x + line_spacing))
        pos_within_line = total_distance % (map_x + line_spacing)

        # Determine the direction and position based on the line number and position within the line
        if pos_within_line <= map_x:  # moving horizontally
            if line % 2 == 0:  # even line number
                self.stubborn_goal = (pos_within_line, line * line_spacing)
            else:  # odd line number
                self.stubborn_goal = (map_x - pos_within_line, line * line_spacing)
        else:  # moving vertically to the next line
            self.stubborn_goal = (self.stubborn_goal[0], line * line_spacing + pos_within_line - map_x)

        # Adjust for Pygame's coordinate system
        self.stubborn_goal = (self.stubborn_goal[0], map_y - self.stubborn_goal[1])

        self.positions[self.n-1] = self.stubborn_goal
        self.time += self.time_delta  # increment the time


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