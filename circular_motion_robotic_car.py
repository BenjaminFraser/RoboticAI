# -----------------
# USER INSTRUCTIONS
#
# Write a function in the class robot called move()
#
# that takes self and a motion vector (this
# motion vector contains a steering* angle and a
# distance) as input and returns an instance of the class
# robot with the appropriate x, y, and orientation
# for the given motion.
#
# *steering is defined in the video
# which accompanies this problem.
#
# For now, please do NOT add noise to your move function.
#
# Please do not modify anything except where indicated
# below.
#
# There are test cases which you are free to use at the
# bottom. If you uncomment them for testing, make sure you
# re-comment them before you submit.

from math import *
import random
# --------
# 
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!

landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"
max_steering_angle = pi/4 # You don't need to use this value, but it is good to keep in mind the limitations of a real car.

# ------------------------------------------------
# 
# this is the robot class
#

class robot:

    # --------

    # init: 
    #   creates robot and initializes location/orientation 
    #

    def __init__(self, length = 10.0):
        self.x = random.random() * world_size # initial x position
        self.y = random.random() * world_size # initial y position
        self.orientation = random.random() * 2.0 * pi # initial orientation
        self.length = length # length of robot
        self.bearing_noise  = 0.0 # initialize bearing noise to zero
        self.steering_noise = 0.0 # initialize steering noise to zero
        self.distance_noise = 0.0 # initialize distance noise to zero
    
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))

    def set(self, new_x, new_y, new_orientation):
        """Allows the setting of new parameters for the given robot instance. The new x,
            y and heading location details must be given as arguments."""
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        """Sets the noise parameters of the robot: bearing noise, steering noise and
            distance noise. This is useful and an important aspect of implementing
            a particle filter."""
        self.bearing_noise  = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
    
    def move(self, motion):
        """Takes a given motion input, of an array of the form [heading angle, distance] 
            and outputs the new robot location data, as a robot instance. This move command
            moves along a section of a circular path according to motion."""
        # obtain the input steer angle and distance, and apply the noise uncertainties to each
        # through using the original value as the mean, and the noise as the standard deviation.
        steer_angle = random.gauss(motion[0], self.steering_noise) 
        s = random.gauss(motion[1], self.distance_noise)
        # ensure steering input is not larger than the maximum possible steering angle
        if abs(steer_angle) > max_steering_angle:
            raise ValueError("Steering angle {} exceeds the maximum "
                             "of {1}".format(steer_angle, max_steering_angle))
        if s < 0.0:
            raise ValueError("The car distance must be positive, not negative.")

        turn_angle = (s / self.length) * tan(steer_angle)
        # set Wheel turn radius (R) as dist(s)/turn_angle if greater than 0.001
        if turn_angle > 0.001:
            R = s / turn_angle
            print "Turning angle is {}".format(R)
            # determine point of rotation co-ords (cx and cy)
            cx = self.x - (sin(self.orientation) * R)
            cy = self.y + (cos(self.orientation) * R)
            print "The co-ords of cx and cy are: {0}, {1}".format(cx, cy)
        else: 
            R = 0
            cx = self.x + (s * cos(self.orientation))
            cy = self.y + (s * sin(self.orientation))
        # determine new x, y and heading positions
        new_x = cx + (R * sin(self.orientation + turn_angle))
        new_y = cy - (R * cos(self.orientation + turn_angle))
        new_heading = (self.orientation + turn_angle) % (2.0 * pi)
        # make a copy of the robot instance and correct localisation data
        updated_rob = self
        updated_rob.set(new_x, new_y, new_heading)
        return updated_rob

## --------
## TEST CASE - only uncomment for testing purposes:
## 
## 1) The following code should print:
##       Robot:     [x=0.0 y=0.0 orient=0.0]
##       Robot:     [x=10.0 y=0.0 orient=0.0]
##       Robot:     [x=19.861 y=1.4333 orient=0.2886]
##       Robot:     [x=39.034 y=7.1270 orient=0.2886]
##
##
##length = 20.
##bearing_noise  = 0.0
##steering_noise = 0.0
##distance_noise = 0.0
##
##myrobot = robot(length)
##myrobot.set(0.0, 0.0, 0.0)
##myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
##
##motions = [[0.0, 10.0], [pi / 6.0, 10], [0.0, 20.0]]
##
##T = len(motions)
##
##print 'Robot:    ', myrobot
##for t in range(T):
##    myrobot = myrobot.move(motions[t])
##    print 'Robot:    ', myrobot
##
##

## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## move function with randomized motion data.

    
## 2) The following code should print:
##      Robot:     [x=0.0 y=0.0 orient=0.0]
##      Robot:     [x=9.9828 y=0.5063 orient=0.1013]
##      Robot:     [x=19.863 y=2.0201 orient=0.2027]
##      Robot:     [x=29.539 y=4.5259 orient=0.3040]
##      Robot:     [x=38.913 y=7.9979 orient=0.4054]
##      Robot:     [x=47.887 y=12.400 orient=0.5067]
##      Robot:     [x=56.369 y=17.688 orient=0.6081]
##      Robot:     [x=64.273 y=23.807 orient=0.7094]
##      Robot:     [x=71.517 y=30.695 orient=0.8108]
##      Robot:     [x=78.027 y=38.280 orient=0.9121]
##      Robot:     [x=83.736 y=46.485 orient=1.0135]
##
##
##length = 20.
##bearing_noise  = 0.0
##steering_noise = 0.0
##distance_noise = 0.0
##
##myrobot = robot(length)
##myrobot.set(0.0, 0.0, 0.0)
##myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
##
##motions = [[0.2, 10.] for row in range(10)]
##
##T = len(motions)
##
##print 'Robot:    ', myrobot
##for t in range(T):
##    myrobot = myrobot.move(motions[t])
##    print 'Robot:    ', myrobot

## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## move function with randomized motion data.


