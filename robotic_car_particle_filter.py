# --------------
# USER INSTRUCTIONS
#
# Now you will put everything together.
#
# First make sure that your sense and move functions
# work as expected for the test cases provided at the
# bottom of the previous two programming assignments.
# Once you are satisfied, copy your sense and move
# definitions into the robot class on this page, BUT
# now include noise.
#
# A good way to include noise in the sense step is to
# add Gaussian noise, centered at zero with variance
# of self.bearing_noise to each bearing. You can do this
# with the command random.gauss(0, self.bearing_noise)
#
# In the move step, you should make sure that your
# actual steering angle is chosen from a Gaussian
# distribution of steering angles. This distribution
# should be centered at the intended steering angle
# with variance of self.steering_noise.
#
# Feel free to use the included set_noise function.
#
# Please do not modify anything except where indicated
# below.

from math import *
import random

# --------
# 
# some top level parameters
#

max_steering_angle = pi / 4.0 # You do not need to use this value, but keep in mind the limitations of a real car.
bearing_noise = 0.1 # Noise parameter: should be included in sense function.
steering_noise = 0.1 # Noise parameter: should be included in move function.
distance_noise = 5.0 # Noise parameter: should be included in move function.

tolerance_xy = 15.0 # Tolerance for localization in the x and y directions.
tolerance_orientation = 0.25 # Tolerance for orientation.


# --------
# 
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!

landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks in (y, x) format.
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"


class robot:
    """Robot class that possesses an x co-ordinate, y co-ordinate and bearing orientation. The robot also
        has a length attribute, along with bearing, steering and distance noises, which are initialised
        at 0.0 by default."""

    def __init__(self, length = 20.0):
        """Creates a robot instance and initialises the location and orientation randomly, along with 
            setting default bearing, steering and distance noises of 0.0."""
        self.x = random.random() * world_size # initial x position
        self.y = random.random() * world_size # initial y position
        self.orientation = random.random() * 2.0 * pi # initial orientation
        self.length = length # length of robot
        self.bearing_noise  = 0.0 # initialize bearing noise to zero
        self.steering_noise = 0.0 # initialize steering noise to zero
        self.distance_noise = 0.0 # initialize distance noise to zero

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

    def measurement_prob(self, measurements):
        """Takes a given array of measurements of landmarks and calculates the probability, using a guassian
            distribution, that the measurement is accurate, through working out the bearing error of the 
            measurement compared to one with no sensing noise. The returned value is a probability between
            0 and 1, which is used within the particle filter as a means of assigning weights to particles."""

        # calculate the correct measurement using no sensing noise
        predicted_measurements = self.sense(0) # Our sense function took 0 as an argument to switch off noise.


        # compute errors
        error = 1.0
        for i in range(len(measurements)):
            error_bearing = abs(measurements[i] - predicted_measurements[i])
            error_bearing = (error_bearing + pi) % (2.0 * pi) - pi # truncate
            

            # update Gaussian
            error *= (exp(- (error_bearing ** 2) / (self.bearing_noise ** 2) / 2.0) /  
                      sqrt(2.0 * pi * (self.bearing_noise ** 2)))

        return error
    
    def __repr__(self): #allows us to print robot attributes.
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), 
                                                str(self.orientation))
    
    def move(self, motion):
        """Takes a given motion input, of an array of the form [heading angle, distance] 
            and outputs the new robot location data, as a robot instance. This move command
            moves along a section of a circular path according to motion."""
        # obtain the input steer angle and distance 
        steer_angle, s = motion[0], motion[1]
        # make a new robot inst to be updated as the moved robot.
        updated_rob = robot()
        updated_rob.bearing_noise = self.bearing_noise
        updated_rob.steering_noise = self.steering_noise
        updated_rob.distance_noise = self.distance_noise

        # ensure steering input is not larger than the maximum possible steering angle
        if abs(steer_angle) > max_steering_angle:
            raise ValueError("Steering angle {0} exceeds the maximum "
                             "of {1}".format(steer_angle, max_steering_angle))
        if s < 0.0:
            raise ValueError("The car distance must be positive, not negative.")

        # apply the noise uncertainties to each
        # through using the original value as the mean, and the noise as the standard deviation.
        steer_angle = random.gauss(steer_angle, self.steering_noise) 
        s = random.gauss(s, self.distance_noise)

        turn_angle = (s / self.length) * tan(steer_angle)
        # set Wheel turn radius (R) as dist(s)/turn_angle if greater than 0.001
        if abs(turn_angle) > 0.001:
            R = s / turn_angle
            # determine point of rotation co-ords (cx and cy)
            cx = self.x - (sin(self.orientation) * R)
            cy = self.y + (cos(self.orientation) * R)
            updated_rob.x = cx + (R * sin(self.orientation))
            updated_rob.y = cy - (R * cos(self.orientation))
        else: 
            R = 0
            updated_rob.x = self.x + (s * cos(self.orientation))
            updated_rob.y = self.y + (s * sin(self.orientation))
        updated_rob.orientation = (self.orientation + turn_angle) % (2.0 * pi)
        return updated_rob

    def sense(self, noise=1):
        """Performs a measurement check of the bearing from the robots current orientation to provided
            landmarks within the environment. Returns an array Z that contains the bearing, in radians, to
            each landmark provided."""
        # initialise Z as an array with the elements equal to 0.0
        Z = [0.0 for x in range(4)]
        # iterate through each of the landmarks and determine angle using atan2:
        for i in range(len(landmarks)):
            # sub robots current y,x from landmarks, and subtract orientation for bearing
            angle = atan2(landmarks[i][0] - self.y, 
                    landmarks[i][1] - self.x) - self.orientation
            # if bearing noise is selected, calculate the uncertainty according to a gaussian dist.
            if noise == 1:
                angle += random.gauss(0.0, self.bearing_noise)
            # round angle to 2 * pi.
            angle %= 2.0 * pi
            Z[i] = angle
            
        return Z

def get_position(p):
    """Takes a given set of N particles, and determines the average co-ordinates of x and y, along with the
        average orientation of the particles, to provide an overall position from the particle set. Returns
        the three values as a list, like follows: [x, y, orientation]."""
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi)) 
                        + p[0].orientation - pi)
    return [x / len(p), y / len(p), orientation / len(p)]

def generate_ground_truth(motions):
    """Determines the actual position and carries out the motions of the robot instance. Returns the
        robot instance, and list of measurements, Z, in the form [robot_inst, Z]. This is known as the
        measurements vector."""

    myrobot = robot()
    myrobot.set_noise(bearing_noise, steering_noise, distance_noise)

    Z = []
    T = len(motions)

    for t in range(T):
        myrobot = myrobot.move(motions[t])
        Z.append(myrobot.sense())
    #print 'Robot:    ', myrobot
    return [myrobot, Z]

def print_measurements(Z):
    """Prints the measurements associated with the generate_ground_truth returned values."""

    T = len(Z)

    print 'measurements = [[%.8s, %.8s, %.8s, %.8s],' % \
        (str(Z[0][0]), str(Z[0][1]), str(Z[0][2]), str(Z[0][3]))
    for t in range(1,T-1):
        print '                [%.8s, %.8s, %.8s, %.8s],' % \
            (str(Z[t][0]), str(Z[t][1]), str(Z[t][2]), str(Z[t][3]))
    print '                [%.8s, %.8s, %.8s, %.8s]]' % \
        (str(Z[T-1][0]), str(Z[T-1][1]), str(Z[T-1][2]), str(Z[T-1][3]))

# --------
#
# The following code checks to see if your particle filter
# localizes the robot to within the desired tolerances
# of the true position. The tolerances are defined at the top.
#

def check_output(final_robot, estimated_position):

    error_x = abs(final_robot.x - estimated_position[0])
    error_y = abs(final_robot.y - estimated_position[1])
    error_orientation = abs(final_robot.orientation - estimated_position[2])
    error_orientation = (error_orientation + pi) % (2.0 * pi) - pi
    correct = error_x < tolerance_xy and error_y < tolerance_xy \
              and error_orientation < tolerance_orientation
    return correct



def particle_filter(motions, measurements, N=500): # I know it's tempting, but don't change N!
    """Initialises the N particles required for the particle filter, and carries out the motion (prediction)
        and measurement (correction) updates that are provided as arguments to the function. The function
        resamples the list of particles after a prediction and correction cycle is carried out, and it does
        this through assigning weights to the particle measurements using measurement_prob() function.

    Args: 
        motions (list): A list of motions to be carried out. Each motion is another sequence containing
                        the motion instructions in the form [turn_magnitude (radians), distance]
        measurements (list): A list of measurements to be applied to the particle filter. Each measurement
                             is a four element list of bearings to each environment landmark.
    Returns:
        The function get_position(p), where p is the set of particles processed by the particle filter.
    """
    # Make particles
    p = []
    for i in range(N):
        r = robot()
        r.set_noise(bearing_noise, steering_noise, distance_noise)
        p.append(r)

    # Update particles with a prediction and correction cycle   
    for t in range(len(motions)):
    
        # motion update (prediction)
        p2 = []
        for i in range(N):
            p2.append(p[i].move(motions[t]))
        p = p2

        # measurement update
        w = []
        for i in range(N):
            w.append(p[i].measurement_prob(measurements[t]))

        # resampling
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p3.append(p[index])
        p = p3
    
    return get_position(p)

## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out.
##
## You can test whether your particle filter works using the
## function check_output (see test case 2). We will be using a similar
## function. Note: Even for a well-implemented particle filter this
## function occasionally returns False. This is because a particle
## filter is a randomized algorithm. We will be testing your code
## multiple times. Make sure check_output returns True at least 80%
## of the time.


 
## --------
## TEST CASES:
## 
##1) Calling the particle_filter function with the following
##    motions and measurements should return a [x,y,orientation]
##    vector near [x=93.476 y=75.186 orient=5.2664], that is, the
##    robot's true location.
##
##motions = [[2. * pi / 10, 20.] for row in range(8)]
##measurements = [[4.746936, 3.859782, 3.045217, 2.045506],
##                [3.510067, 2.916300, 2.146394, 1.598332],
##                [2.972469, 2.407489, 1.588474, 1.611094],
##                [1.906178, 1.193329, 0.619356, 0.807930],
##                [1.352825, 0.662233, 0.144927, 0.799090],
##                [0.856150, 0.214590, 5.651497, 1.062401],
##                [0.194460, 5.660382, 4.761072, 2.471682],
##                [5.717342, 4.736780, 3.909599, 2.342536]]
##
##print particle_filter(motions, measurements)

## 2) You can generate your own test cases by generating
##    measurements using the generate_ground_truth function.
##    It will print the robot's last location when calling it.
##
##
##number_of_iterations = 6
##motions = [[2. * pi / 20, 12.] for row in range(number_of_iterations)]
##
##x = generate_ground_truth(motions)
##final_robot = x[0]
##measurements = x[1]
##estimated_position = particle_filter(motions, measurements)
##print_measurements(measurements)
##print 'Ground truth:    ', final_robot
##print 'Particle filter: ', estimated_position
##print 'Code check:      ', check_output(final_robot, estimated_position)



