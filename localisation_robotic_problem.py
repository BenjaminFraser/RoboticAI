# The function localize takes the following arguments:
#
# colors:
#        2D list, each entry either 'R' (for red cell) or 'G' (for green cell)
#
# measurements:
#        list of measurements taken by the robot, each entry either 'R' or 'G'
#
# motions:
#        list of actions taken by the robot, each entry of the form [dy,dx],
#        where dx refers to the change in the x-direction (positive meaning
#        movement to the right) and dy refers to the change in the y-direction
#        (positive meaning movement downward)
#        NOTE: the *first* coordinate is change in y; the *second* coordinate is
#              change in x
#
# sensor_right:
#        float between 0 and 1, giving the probability that any given
#        measurement is correct; the probability that the measurement is
#        incorrect is 1-sensor_right
#
# p_move:
#        float between 0 and 1, giving the probability that any given movement
#        command takes place; the probability that the movement command fails
#        (and the robot remains still) is 1-p_move; the robot will NOT overshoot
#        its destination in this exercise
#
# The function should RETURN (not just show or print) a 2D list (of the same
# dimensions as colors) that gives the probabilities that the robot occupies
# each cell in the world.
#
# Compute the probabilities by assuming the robot initially has a uniform
# probability of being in any cell.
#
# Also assume that at each step, the robot:
# 1) first makes a movement,
# 2) then takes a measurement.
#
# Motion:
#  [0,0] - stay
#  [0,1] - right
#  [0,-1] - left
#  [1,0] - down
#  [-1,0] - up

def localize(colors,measurements,motions,sensor_right,p_move):
    # initializes p to a uniform distribution over a grid of the same dimensions as colors
    pinit = 1.0 / float(len(colors)) / float(len(colors[0]))
    p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]
    # iterate in parallel through both motions and measurements.
    for motion, measure in zip(motions, measurements):
        # carry out the relevant move and measure actions on the distribution p.
        print p, motion, p_move
        p = move(p, motion, p_move)
        print p, colors, measure, sensor_right
        p = sense(p, colors, measure, sensor_right)
    return p

def move(p, motion, p_move):
    not_move = 1 - p_move
    if motion == [0,0]:
        # since [0,0] means stay, return p without modification.
        return p    
    elif motion == [0,1]:
        # process a move to the right
        q = []
        for row in p:
            correct_move = [x*p_move for x in row[-1:] + row[:-1]]
            non_move = [x*not_move for x in row]
            new_row = [x + y for x, y in zip(correct_move, non_move)]
            q.append(new_row)
        return q
    elif motion == [0,-1]:
        # process a move to the left
        q = []
        for row in p:
            correct_move = [x*p_move for x in row[1:] + row[:1]]
            non_move = [x*not_move for x in row]
            new_row = [x + y for x, y in zip(correct_move, non_move)]
            q.append(new_row)
        return q
    elif motion == [1,0]:
        # process a move downwards
        q = p[:]
        # round off the length of the matrix, to support longer moves in future.
        n = -1 % len(p)
        # move the row downwards in the matrix.
        shifted = p[:n]
        del q[:n]
        q.extend(shifted)
        return q
    elif motion == [-1,0]:
        # process a move upwards
        q = p[:]
        n = 1 % len(p)
        # move the row upwards in the matrix.
        shifted = p[:n]
        del q[:n]
        q.extend(shifted)
        return q
    else:
        raise KeyError("Invalid movement entered!")

def sense(p, colors, measure, sense_right):
    q = p
    sense_wrong = 1 - sense_right
    # check if the sensed measurement is within the environment.
    for i, row in enumerate(colors):
        if measure in row:
            for j, x in enumerate(row):
                # calculate new values of probability dependent on sense probability.
                q[i][j] = p[i][j]*sense_right if row[j] == measure else p[i][j]*sense_wrong
    # determine the sum of elements of the 2d array
    q_sum = 0
    for row in q:
        q_sum += sum(row)
    # divide each element by the sum total in order to normalise each probability.
    for i, row in enumerate(q):
        for x in range(len(row)):
            q[i][x] = q[i][x]/q_sum
    return q

def show(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
    print '[' + ',\n '.join(rows) + ']'
    
#############################################################
# For the following test case, your output should be 
# [[0.01105, 0.02464, 0.06799, 0.04472, 0.02465],
#  [0.00715, 0.01017, 0.08696, 0.07988, 0.00935],
#  [0.00739, 0.00894, 0.11272, 0.35350, 0.04065],
#  [0.00910, 0.00715, 0.01434, 0.04313, 0.03642]]
# (within a tolerance of +/- 0.001 for each entry)

colors = [['R','G','G','R','R'],
          ['R','R','G','R','R'],
          ['R','R','G','G','R'],
          ['R','R','R','R','R']]
measurements = ['G','G','G','G','G']
motions = [[0,0],[0,1],[1,0],[1,0],[-1,0]]

# a few random test cases I used for fault finding.
#p = [[0.05, 0.05, 0.05, 0.05, 0.05], [0.05, 0.05, 0.05, 0.05, 0.05], [0.05, 0.05, 0.05, 0.05, 0.05], [0.05, 0.05, 0.05, 0.05, 0.05]]
#p = sense(p, colors, 'G', sense_right=0.7)
#print p
#p = move(p, [1,0], p_move=0.8)
#print p

p = localize(colors,measurements,motions,sensor_right = 0.7, p_move = 0.8)
show(p) # displays your answer
