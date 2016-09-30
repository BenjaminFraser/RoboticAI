# Now we want to create particles,
# p[i] = robot(). In this assignment, write
# code that will assign 1000 such particles
# to a list.
#
# Your program should print out the length
# of your list (don't cheat by making an
# arbitrary list of 1000 elements!)
#
# Don't modify the code below. Please enter
# your code at the bottom.

from math import *
import random


landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0


class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError, 'X coordinate out of bound'
        if new_y < 0 or new_y >= world_size:
            raise ValueError, 'Y coordinate out of bound'
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    
    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);
    
    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z
    
    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError, 'Robot cant move backwards'         
        
        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi
        
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size    # cyclic truncate
        y %= world_size
        
        # set particle
        res = robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res
    
    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
    
    def measurement_prob(self, measurement):
        """Takes a robot sense measurement of the form robot.sense() and determines how
            likely this measurement is to be accurate. In essence, this provides us with
            an imortance weight to assign individual particles based on their location."""
        prob = 1.0;
        # determine how far the predicted measurements (measurement arg) lies from actual belief
        # note: there MUST be measurement noise for this, otherwise 0/0 indeterminate will raise an exception.
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob
    
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))


#myrobot = robot()
#myrobot.set_noise(5.0, 0.1, 5.0)
#myrobot.set(30.0, 50.0, pi/2)
#myrobot = myrobot.move(-pi/2, 15.0)
#print myrobot.sense()
#myrobot = myrobot.move(-pi/2, 10.0)
#print myrobot.sense()

def particle_list(size, fwd_noise=0, turn_noise=0, sense_noise=0):
    p_list = []
    for particle in range(size):
        inst = robot()
        inst.set_noise(fwd_noise, turn_noise, sense_noise)
        p_list.append(inst)
    return p_list

def particle_move(particle_list, move_angle, move_distance):
    p_list = []
    for i in range(len(particle_list)):
        p_list.append(particle_list[i].move(move_angle, move_distance))
    return p_list

def assign_particle_weights(robot, particle_list):
    weights = []
    Z = myrobot.sense()
    for i in range(len(particle_list)):
        weights.append(particle_list[i].measurement_prob(Z))
    return weights

def select_particle(particle_list, particle_weights):
    """Selects a single particle within a list of many particles (particle list), based on the probabilistic 
        weight assigned to each particle, as in the particle weights list. Each index within particle list
        alligns with the probability weighting within the same index in the particle weights list.
    Args: 
        particle_list (list): A list of particle arrays, of the form [x_position, y_position, heading_direction]
        particle_weights (list): A list of probabilistic particle weights, with indexes corresponding to those
                                 within particle_list.
    Returns:
        The chosen particle in the form [x_position, y_position, heading_direction]
    """
    maximum = sum(particle_weights)
    choice = random.uniform(0, maximum) 
    current = 0
    for n, particle in enumerate(particle_list):
        current += particle_weights[n]
        if current > choice:
            return particle

def resampling(particle_list, particle_weights):
    """Takes a given list of particle x and y positions and heading direction, and a list of 
        equivalent particle weights, and randomly resamples the list according to weighted 
        probabilities. Particles with higher importance weight are more likely to be resampled
    Args: 
        particle_list (list): A list of particle arrays, of the form [x_position, y_position, heading_direction]
        particle_weights (list): A list of probabilistic particle weights, with indexes corresponding to those
                                 within particle_list.
    Returns: 
        Updated particle list and particle weights after resampling, in the form updated_list, updated_weights.
    """
    if len(particle_list) == len(particle_weights):
        updated_list = []
        for i in range(len(particle_list)):
            updated_list.append(select_particle(particle_list, particle_weights))
        updated_weights = assign_particle_weights(myrobot, updated_list)
        return updated_list, updated_weights
    else:
        raise ValueError("Particle list and particle weights are not equal length!")

def distance_eval(robot, p_list):
    """Takes a robot instance, and an associated particle list, and determines the average error of all 
        the particles relative to the robots actual position. The function determines the euclidean distance
        between the robots x,y positions and the average particles x,y positions
    Args:
        robot: An instance of the robot class.
        p_list (list): A list of particle positions and orientations of size N.
    Returns:
        The average euclidean distance between the particles and actual robot position of the filter, as a float.
    """
    sum = 0.0;
    for i in range(len(p_list)): # calculate mean error by iterating through p
        dx = (p_list[i].x - robot.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p_list[i].y - robot.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p_list))

N = 1000
myrobot = robot()
p = particle_list(N, 0.05, 0.05, 5.0)
p_weights = assign_particle_weights(myrobot, p)

for i in range(10):
    myrobot = myrobot.move(0.1, 5.0)
    p = particle_move(p, 0.1, 5.0)
    p, p_weights = resampling(p, p_weights)
    print distance_eval(myrobot, p)

# test resampling algorithm functions
print len(p), len(p_weights)
print p
