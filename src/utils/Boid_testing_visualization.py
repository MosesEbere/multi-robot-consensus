import numpy as np
from p5 import stroke, circle

class Boid:

    def __init__(self,id,weights,percep_field,state):
        
        self.id = id
        self.w_a, self.w_c, self.w_s = weights
        self.local_r, self.local_theta = percep_field

        # [x,y,theta,v_x,v_y,w]
        self.state = state

        # Visulaization radius
        self.radius = 5 

        # list of neigbors as boid objects
        self.neighbors = None

    def show(self):
        stroke(255)
        circle((self.state[0], self.state[1]), self.radius)

    def edges(self):
        if self.state[0] > self.get_canvas()[0]:
            self.state[0] = 0
        elif self.state[0] < 0:
            self.state[0] = self.get_canvas()[0]

        if self.state[1] > self.get_canvas()[1]:
            self.state[1] = 0
        elif self.state[1] < 0:
            self.state[1] = self.get_canvas()[1]

    @staticmethod
    def get_canvas():
        canvas = [1000, 1000]
        return canvas

    def get_pos(self):
        '''
        Returns position as [x,y]
        '''
        return self.state[:2]
    
    def get_theta(self):
        '''
        Returns heading theta
        '''
        return self.state[2]

    def get_pose(self):
        '''
        Returns pose as [x,y,theta]
        '''
        return self.state[:3]
    
    def get_linvel(self):
        '''
        Returns linear velocity as [v_x,v_y]
        '''
        return self.state[3:5]
    
    def get_angvel(self):
        '''
        Returns angular velocity w (theta_dot)
        '''
        return self.state[5]

    def get_vel(self):
        '''
        Returns lin and angular velocity as [v_x,v_y,w]
        '''
        return self.state[3:]
    
    def set_state(self,state):
        '''
        Updates state of boid
        '''
        self.state = state

    def set_neighbors(self, neighbor):
        '''
        Updates the neighbors of the boid
        '''
        self.neighbors = neighbor
            
    def update(self,boids=None):
        '''
        Updates the robot pose and neighborhood.

        Parameters:
        boids: list of all boids as Boid instances

        Returns:
        - cmd_vel: desired velocities of the boid as [v_x,v_y]
        '''
        
        # TODO: update neighborhood 

        # Compute acceleration
        acc = self._combine_behaviours()

        # TODO: Compute desired velocities from desired accel
        cmd_vel = self.get_linvel() + acc

        # Motion Model --------------------------------------
        self.state[0] += cmd_vel[0]
        self.state[1] += cmd_vel[1]

        return cmd_vel


    def _alignment(self):
        ''' Computes required acceleration due to alignment. '''
        # TODO
        acc = np.zeros((2,))
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                acc += np.array(self.get_linvel()) - np.array(boid.get_linvel()) 
            acc = acc/len(self.neighbors) 
        return acc
    

    def _cohesion(self):
        ''' Computes required acceleration due to cohesion. '''
        # TODO
        acc = np.zeros((2,))
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                acc += np.array(self.get_pos()) - np.array(boid.get_pos()) 
            acc = acc/len(self.neighbors) 
        return acc
    

    def _seperation(self):
        ''' Computes required acceleration due to seperation. '''
        # TODO
        acc = np.zeros((2,))
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                distance = np.array(self.get_pos()) - np.array(boid.get_pos()) 
                square_distance = (np.linalg.norm(np.array(self.get_pos()) - np.array(boid.get_pos())))**2
                acc += (distance/square_distance)
            acc = acc/len(self.neighbors) 
        return acc


    def _combine_behaviours(self):
        ''' Calls behaviours, and computes the net weighted acceleration. '''
        acc = self.w_a*self._alignment() + self.w_c*self._cohesion() - self.w_s*self._seperation()
        return acc
    

    # TODO: def _obstacle_avoidance(self):
    # TODO: def _pursuits(self)