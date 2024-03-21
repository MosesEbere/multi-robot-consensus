import numpy as np
import math
import rospy
import matplotlib.pyplot as plt
import copy
import matplotlib.patches as patches

def wrap_angle(angle):
    """Wrap an angle to the range of -pi to pi."""
    return math.atan2(math.sin(angle), math.cos(angle))

class PotentialField:

    def __init__(self, obstacle_radius):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        self.map_dim = None

        # map sampling resolution (size of a cell))                            
        self.resolution = None
        
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        
        # set method has been called                          
        self.there_is_map = False
        
        # radius arround the robot used to check occupancy of a given position                 
        self.obstacle_radius = obstacle_radius                    

        # Force field for potential function
        self.force_field = None
    
    # Set occupancy map, and create initial force field. 
    def set(self, data, resolution, origin):
    
        print(' In set Map')

        # save map params
        self.map = data
        self.map_dim = self.map.shape
        self.resolution = resolution
        self.origin = np.array(origin)
    
        # initialize force field
        brushfire_map = self._brushfire(map = self.map/100.0, 
                                        max_radius=self.obstacle_radius/self.resolution)
        self.force_field = self._create_force_field(brushfire_map)
    
        # self.visualize_force_field()
        # np.save('forcefield.npy',self.map)
        np.savez('FF.npz', array_with_arrays=self.force_field)
        # print(self.force_field)

        print('-------  Force Field Params  ----------')
        rospy.loginfo('grid radius: %s', self.obstacle_radius/self.resolution)
        rospy.loginfo('size: %s', self.force_field.shape)
        rospy.loginfo('index 10,10: %s', self.force_field[50,50])
    
    #_____________________________  main call function  ___________________________________    

    def _potential_field(self,pose):
        if self.force_field == None:
            return [0,0]
        grid_pose = self._position_to_map(pose)
        grid_pose = (int(round(grid_pose[0],0)),int(round(grid_pose[1],0)))
        return self.force_field[grid_pose] 
    
    #_____________________________  algorithms  ___________________________________

    def _brushfire(self,map,max_radius):
        '''
        Compute the potential field created by obstacles via brushfire algo
        '''
        map_shape = np.shape(map)

        motions = [[-1,0], [1,0], [0,-1], [0,1],\
                    [-1,-1], [-1,1], [1,-1], [1,1]]
        
        value = 1 # value of the obstacle position
        map = np.array(map)
        result = np.where(map == 1)
        [x_s,y_s] = result[0:2]
        ones_loc = []
        for i in range(len(x_s)):
            ones_loc.append([x_s[i], y_s[i]])
        queue = ones_loc
        
        while queue != []:
            value += 1
            if value > max_radius:
                break
            new_queue = []
            for p in queue:
                for m in motions:
                    new_idx = [p[0]+m[0],p[1]+m[1]]    # p+m
                    if new_idx[0]>-1 and new_idx[1]>-1 and new_idx[0]<map_shape[0] and new_idx[1]<map_shape[1]:
                        if map[new_idx[0],new_idx[1]] == 0:
                            map[new_idx[0],new_idx[1]] = value
                            new_queue.append(new_idx) 
            queue = new_queue
        return map

    def _create_force_field(self, brushfire_map):
        '''
        Create force field using gradient of the brushfire and magnitude
        '''
        map_shape = brushfire_map.shape

        gradient_direction = np.arctan2(np.gradient(brushfire_map, axis=0), 
                                        np.gradient(brushfire_map, axis=1))

        brushfire_norm = 1 - brushfire_map / brushfire_map.max()

        force_field = np.empty(map_shape, dtype=object)
        for r in range(map_shape[0]):
            for c in range(map_shape[1]):
                angle_radians = gradient_direction[r,c]
                # scale the force field vector by the distance (from brushfire algo)
                dx = brushfire_norm[r,c] * np.cos(angle_radians)
                dy = brushfire_norm[r,c] * np.sin(angle_radians)
                force_field[r,c] = np.array([dx,dy])
        
        return force_field

    #_____________________________  helper funcs  ___________________________________

    def _position_to_map(self, p):
        mx = (p[0]-self.origin[0])/self.resolution 
        my = (p[1]-self.origin[1])/self.resolution
        if self._in_map([mx,my]):
            return [mx,my]
        return [] 
    
    def _in_map(self, loc):
        '''
        loc: list of index [x,y]
        returns True if location is in map and false otherwise
        '''
        [mx,my] = loc
        if mx >= self.map.shape[0]-1 or my >= self.map.shape[1]-1 or mx < 0 or my < 0:
            return False 
        return True

    
