# Enhanced version of the initial sense function, that provides multiple measurements
# and addresses robotic inexact motion.


p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
pHit = 0.6
pMiss = 0.2

def sense(p, Z):
    q = p
    if Z in world:
        for i, x in enumerate(p):
            q[i] = p[i]*pHit if world[i] == Z else p[i]*pMiss
    q_sum = sum(q)
    for x in range(len(q)):
        q[x] = q[x]/q_sum
    return q

# in order to handle multiple measurements, rather than altering the original function
# simply carry out a loop that performs multiple measurements.
for measure in measurements:
    p = sense(p, measure)

print p