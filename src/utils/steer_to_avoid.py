import rospy
import numpy as np
import math
import copy
from scipy.ndimage import morphology

class SteerToAvoid:

    # Constructor
    def __init__(self,obstacle_radius,step_angle,max_steering_angle):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        self.map_dim = None
        self.step_angle = step_angle
        self.max_steering_angle = max_steering_angle
        self.obstacle_radius=obstacle_radius
        self.resolution = None        # map sampling resolution (size of a cell))                            
        self.origin = None            # world position of cell (0, 0) in self.map                      
        self.there_is_map = False          # set method has been called                          
        self.steering_force = None
        self.path_valid = True
    
    def set(self, data, resolution, origin):     
        # Set occupancy map, its resolution and origin. 
        self.map = self.dilate_obstacles(data, 4)   # dilate the obstacles slightly 
        # self.map = data   # dilate the obstacles slightly 
        self.map_dim = self.map.shape
        self.resolution = resolution
        self.origin = np.array(origin)
        rospy.loginfo('map dimensions: %s', self.map_dim)

    def _position_to_map(self, p):
        # print("p[0]",p[0],"p[1]",p[1],"resolution",self.resolution)
        mx = (p[0]-self.origin[0])/self.resolution 
        my = (p[1]-self.origin[1])/self.resolution
        # if self._in_map([mx,my]):
        return [int(round(mx)),int(round(my))]
        # return [] 
    
    def _in_map(self, loc):
        '''
        loc: list of index [x,y]
        returns True if location is in map and false otherwise
        '''
        [mx,my] = loc
        if mx >= self.map.shape[0]-1 or my >= self.map.shape[1]-1 or mx < 0 or my < 0:
            return False 
        return True
    
    def dilate_obstacles(self,grid, dilation_length):
        obstacle_mask = (grid == 100)
        structuring_element = np.array([[1, 1, 1],
                                        [1, 1, 1],
                                        [1, 1, 1]], dtype=bool)
        dilated_mask = morphology.binary_dilation(obstacle_mask, iterations=dilation_length, structure=structuring_element)
        dilated_grid = np.zeros_like(grid)
        dilated_grid[dilated_mask] = 100  # Set occupied cells to 100 in the dilated grid
        return dilated_grid
    
    def dilate_obstacles(self,grid, dilation_length):
        obstacle_mask = (grid == 100)
        structuring_element = np.array([[1, 1, 1],
                                        [1, 1, 1],
                                        [1, 1, 1]], dtype=bool)
        dilated_mask = morphology.binary_dilation(obstacle_mask, iterations=dilation_length, structure=structuring_element)
        dilated_grid = np.zeros_like(grid)
        dilated_grid[dilated_mask] = 100  # Set occupied cells to 100 in the dilated grid
        return dilated_grid
    
    #----------------------------------------------------------------behavior core functions----------------------------#

    def is_valid(self, point):
    # Assume grid_map is a 2D list representing the grid map
        x, y = self._position_to_map(point)
        # Check if the point is within the grid boundaries
        if 0 <= x and x < self.map_dim[0] and 0 <= y and y < self.map_dim[0]:
            # Check if the cell is free (0) or occupied (100)
            if self.map[x,y] == 0:
                return True  # Free cell
            else:
                # print('the cell that is false!:', point)
                return False  # Occupied cell
        else:
            return False  # Point is outside the grid boundaries

    def calc_lookahead (self,boid_x,boid_y,boid_theta):
        new_x = boid_x + self.obstacle_radius * math.cos(boid_theta)
        new_y = boid_y + self.obstacle_radius * math.sin(boid_theta)
        return self._position_to_map([new_x, new_y]), [new_x,new_y]

    def check_path(self, p1,p2, step_size=0.25):
        waypoints=[]
        dist = np.linalg.norm(np.array(p1) - np.array(p2))
        num_steps = dist / step_size
        num_steps= int(num_steps)
        for j in range(num_steps):
            interpolation = float(j) / num_steps  #the interpolation value for each step to find the pt we are checking right now
            x = p1[0] * (1-interpolation) + p2[0] * interpolation
            y = p1[1] * (1-interpolation) + p2[1] * interpolation
            waypoints.append((x,y))

        waypoints = waypoints[1:]
        
        for w in waypoints:
            if self.is_valid(w) == False:
                return False
        return True
         
    def turn_back(self,boid_theta):
        return boid_theta + 3.14  # Return the opposite angle if outside the map
    
    def _get_steering_direction(self, boid_pos, boid_vel):
        steering_adjustment = 0  #no adjustment done yet
        boid_x, boid_y = boid_pos
        boid_theta = math.atan2(boid_vel[1],boid_vel[0]) #we calc the theta using the direction of velocity
        map_coords, cart_coords = self.calc_lookahead(boid_x,boid_y,boid_theta)
        grid_x, grid_y = int(map_coords[0]), int(map_coords[1])
        self.path_valid = self.check_path(boid_pos,cart_coords)
        
        while not(self.path_valid): #and abs(steering_adjustment) <= self.max_steering_angle:
            steering_adjustment += self.step_angle

            # check right
            new_theta = boid_theta + steering_adjustment
            map_coords, cart_coords = self.calc_lookahead(boid_x,boid_y,new_theta)
            self.path_valid = self.check_path(boid_pos,cart_coords)
            if self.path_valid:
                boid_theta = new_theta
                break
            new_grid_x, new_grid_y = map_coords
            if not(self._in_map([new_grid_x,new_grid_y])):
                new_grid_x, new_grid_y = new_grid_x/2, new_grid_y/2

            # check left
            new_theta = boid_theta - steering_adjustment
            map_coords, cart_coords = self.calc_lookahead(boid_x,boid_y,new_theta)
            self.path_valid = self.check_path(boid_pos,cart_coords)
            if self.path_valid:
                boid_theta = new_theta
                break
            new_grid_x, new_grid_y = map_coords
            if not(self._in_map([new_grid_x,new_grid_y])):
                new_grid_x, new_grid_y = new_grid_x/2, new_grid_y/2

            if steering_adjustment == 0.0:
                return None 
        
        return boid_theta
    
    def _create_steering_force(self,steering_angle):
        # print('Steering angle (NEW): ', steering_angle)
        fx= math.cos(steering_angle)
        fy= math.sin(steering_angle)
        return np.array([fx,fy])
    
    def _steer_to_avoid(self, boid_pose, boid_vel, boid_goal=None): 
        vel_abs = np.linalg.norm(boid_vel)
        if np.all(boid_vel == 0):
            return np.array([0.0,0.0])
        steering_angle = self._get_steering_direction(boid_pose[:2], boid_vel)
        if steering_angle is None:
            return np.array([0.0, 0.0])
        self.steering_force = self._create_steering_force(steering_angle)
        return self.steering_force

