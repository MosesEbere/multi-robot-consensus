import numpy as np
import random


class Navigate:
    def __init__(self, max_acc, max_speed):
        # map: 2D array of integers which categorizes world occupancy
        self.max_acc = max_acc
        self.max_speed = max_speed

    def _seek(self, boid_pose, boid_vel, boid_goal):
        ''' Computes required acceleration for the boid to seek the goal.'''
        acc = np.array([0., 0.])
        if boid_goal != None:
            distance = np.array(np.array(boid_pose[:2]) - np.array(boid_goal[:2]))
            square_distance = (np.linalg.norm(np.array(boid_pose[:2]) - np.array(boid_goal[:2])))**2
            # Mitigate zero-division
            square_distance = 1 if square_distance == 0 else square_distance
            acc = (distance/square_distance)
        return acc

    def _arrival(self, boid_pose, boid_vel, boid_goal):
        ''' Computes required acceleration for the boid to arrive at the goal.'''
        desired_velocity = np.array([0., 0.])
        if boid_goal != None:
            target_offset = np.array(boid_goal[:2]) - np.array(boid_pose[:2])   # goal[x, y], not r or tolerance. 
            distance = np.linalg.norm(target_offset)
            if distance < boid_goal[3]:
                # print("Distance: ", distance)
                #TODO: send a flag that boid_goal has been reached and generate a new boid_goal
                
                return np.array([0., 0.]) # Set the distance to 0 when it falls below the tolerance. 

            ramped_speed = self.max_speed * (distance / boid_goal[2])
            clipped_speed = np.minimum(ramped_speed, self.max_speed)
            desired_velocity = (clipped_speed / distance) * target_offset
            # Sample the desired velocity from the velocity space using probability
            # desired_velocity = np.array([random.uniform(-self.max_speed, self.max_speed), random.uniform(-self.max_speed, self.max_speed)]) if random.uniform(0, 1) < 0.5 else desired_velocity
        return desired_velocity
