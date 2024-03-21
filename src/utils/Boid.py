import numpy as np
import math

class Boid:

    def __init__(self, id, weights, percep_field, state, **kwargs):
        
        self.id = id
        self.w_a, self.w_c, self.w_s = weights
        self.local_r, self.local_theta = percep_field

        # save poses for plotting later
        self.poses_list = np.empty((0, 3), dtype=float)

        # [x,y,theta,v_x,v_y,w]
        self.state = state

        # list of neigbors as boid objects
        self.neighbors = None

        # Desired goal - [x, y, radius, tolerance]
        self.goal = None

        # Max speed
        self.max_speed = kwargs['max_speed']

        # Max acceleration
        self.max_acc = kwargs['max_acc']

        # Use prioritized acceleration
        self.use_prioritized_acc = kwargs['use_prioritized_acc']

        # List of behaviors in order of priority*
        self.behavior_list = kwargs['behavior_list']

        # Hard Arrival (Hard Stop)
        self.hard_arrival = kwargs['hard_arrival']

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
    
    def get_goal(self):
        '''
        Returns goal as [x,y,radius,tolerance]
        '''
        return self.goal
    
    def get_pos_n_velocity_direction(self):
        '''
        Returns pos and direction of movement as [x,y,theta_dir]
        '''
        boid_theta = math.atan2(self.state[4],self.state[3]) #we calc the theta using the direction of velocity
        return np.array([self.state[0], self.state[1], boid_theta])
    
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

    def set_goal(self, goal):
        '''
        Updates the goal of the boid
        '''
        self.goal = goal

    def behavior_function(self, behavior):
        '''
        Dynamically calls the behavior function
        '''
        name = list(behavior.keys())[0]
        weight = list(behavior.values())[0][0]
        constructor = list(behavior.values())[0][1]
        if constructor == 'self':
            return weight * getattr(self, name)()
        return weight * getattr(constructor, name)(self.get_pose(), self.get_linvel(), self.get_goal())
            
    def update(self, boids=None):
        '''
        Calls and combines behaviors.

        Parameters:
        boids: list of all boids as Boid instances

        Returns:
        - cmd_vel: desired velocities of the boid as [v_x,v_y]
        '''

        # Compute acceleration
        # Check if the boid is a leader
        leader = False if self.get_goal() == None else True
        acc = self._combine_behaviors(self.behavior_list, leader, self.use_prioritized_acc)

        # # Compute velocity
        if leader:
            cmd_vel = np.array([0., 0.]) if acc[0] == 0.0 and acc[1] == 0.0 else np.tanh(acc) # **
        else:
            cmd_vel = acc # NOTE: This is does/n't use the appropriate motion model because of the obstacle avoidance.
            # cmd_vel = acc + self.get_linvel() # NOTE: This is doesn't use the appropriate motion model because of the obstacle avoidance.
            # cmd_vel = 0.1 * np.tanh(np.array(self.get_linvel())) + acc
            # vel_mag = np.linalg.norm(self.get_linvel())
            # acc_mag = np.linalg.norm(acc)
            # vel_acc = vel_mag * acc/acc_mag if acc_mag != 0 else np.array([0., 0.])
            # cmd_vel = vel_acc + acc

        # print("cmd_vel: ", cmd_vel, acc)

        # Clip the velocity before returning it. 
        cmd_vel = np.clip(cmd_vel, -self.max_speed, self.max_speed)

        # Save trajectory
        # self.poses_list = np.vstack((self.poses_list, self.get_pos_n_velocity_direction()))
        # np.savetxt('/home/alamdar11/Desktop/path_simple_maze_10x10_SteerToAvoid_wall.npy', self.poses_list)

        return cmd_vel

    def _alignment(self):
        ''' Computes required acceleration due to alignment. '''
        acc = np.array([0., 0.])
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                acc +=  np.array(boid.get_linvel()) - np.array(self.get_linvel()) 
            acc = acc/len(self.neighbors) 
        return acc

    def _cohesion(self):
        ''' Computes required acceleration due to cohesion. '''
        acc = np.array([0., 0.]) 
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                acc +=  np.array(boid.get_pos()) - np.array(self.get_pos())
            acc = acc/len(self.neighbors) 
        return acc

    def _separation(self):
        ''' Computes required acceleration due to seperation. '''
        acc = np.array([0., 0.])
        if self.neighbors != None and len(self.neighbors) != 0:
            for boid in self.neighbors:
                distance = np.array(boid.get_pos()) - np.array(self.get_pos())
                square_distance = (np.linalg.norm(np.array(boid.get_pos()) - np.array(self.get_pos())))**2
                acc += (distance/square_distance)
            acc = acc/len(self.neighbors) 
        return - acc # Negative because we want to move away from the neighbors.

    def _combine_behaviors(self, behavior_list=[], leader=False, use_prioritized_acc=False):
        ''' Calls behaviours, and computes the net weighted acceleration. '''
        acc = np.array([0., 0.])
        for behavior in behavior_list:
            # print("Behavior: ", behavior)
            acc_request = self.behavior_function(behavior)
            if list(behavior.keys())[0] == '_steer_to_avoid' and not (acc_request[0] != 0.0 and acc_request[1] != 0.0):
                # print("steer to avoid active, acc request: ", acc_request)
                continue
            # If there is no goal (current boid is a follower), and this behavior is arrival, then set the acceleration to 0.
            if not leader and list(behavior.keys())[0] == '_arrival':
                acc_request = np.array([0., 0.])

            # ------------------------------------------------------------------------
            # If there is a goal (current boid is a leader), and this behavior is arrival, then check if the goal has been reached.
            elif leader and list(behavior.keys())[0] == '_arrival':
                if self.hard_arrival:
                    if acc_request[0] == 0.0 and acc_request[1] == 0.0:
                        return acc_request + acc
                        # return np.array([0.02, 0.02])
            # ------------------------------------------------------------------------

            if use_prioritized_acc:
                acc_mag = np.linalg.norm(acc_request)
                if np.linalg.norm(acc) + acc_mag > self.max_acc:
                    # Clip the acceleration
                    return np.clip(acc + acc_request, -self.max_acc, self.max_acc)
            acc += acc_request
        return acc

    def test_obstacle_avoidance(self,ob=None):
        # print("Boid ID: ", self.id)

        vel = np.array([0.8, 0.0])
        obs_vel = 0.8*np.array(ob._steer_to_avoid(self.get_pose(), self.get_linvel())) 
        # print("obs_vel: ", obs_vel)

        if obs_vel[0] != 0.0 and obs_vel[1] != 0.0:
            vel = obs_vel
        
        # print('Velocity: ', vel)
        # print('Velocity Frome: ', np.array(ob.compute(self.get_pos(),self.get_linvel())) )
        return vel