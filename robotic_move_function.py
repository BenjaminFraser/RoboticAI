#Program a function that returns a new distribution 
#q, shifted to the right by U units. If U=0, q should 
#be the same as p.

p=[0, 1, 0, 0, 0]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
motions = [1, 1]
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def sense(p, Z):
    q = p
    # check if the sensed measurement is within the environment.
    if Z in world:
        for i, x in enumerate(p):
            # calculate new values of probability dependent on hit/miss probability.
            q[i] = p[i]*pHit if world[i] == Z else p[i]*pMiss
    q_sum = sum(q)
    # divide each value of q by the sum of q cells in order to normalise the probability distribution.
    for x in range(len(q)):
        q[x] = q[x]/q_sum
    return q

def move(p, U):
    # perform a shift of indexes by an integer 'U' using splice notation:
    # p[-U:] + p[:-U]
    # NOTE: Another longer winded version of carrying this out, as provided by Udacity is:
    # for i in range(len(p)):
        # q.append(p[(i-U) % len(p)])
    # calculate probabilities of making an exact movement of U, along with a low and high miss,
    # and add them to the distribution as a set of probabilities of robotic location.
    u_exact = [x*0.8 for x in p[-U:] + p[:-U]]
    u_low = [x*0.1 for x in p[-U-1:] + p[:-U-1]]
    u_high = [x*0.1 for x in p[-U+1:] + p[:-U+1]]
    q = [x + y + z for x, y, z in zip(u_exact, u_low, u_high)]

    return q

# simulate a move of the robot x times, at a movement range of u for an array of locations p.
def move_robot(p, x, u):
    s = p 
    for movement in range(x):
        s = move(s, u)
    return s

# simulate 100 moves of the robot for 1 step during each move.
robot_move_result = move_robot(p, 2, 1)
print robot_move_result

#Given the list motions=[1,1] which means the robot 
#moves right and then right again, compute the posterior 
#distribution if the robot first senses red, then moves 
#right one, then senses green, then moves right again, 
#starting with a uniform prior distribution.
for i in range(len(motions)):
    p = sense(p, measurements[i])
    p = move(p, motions[i])