#Modify your code so that it normalizes the output for 
#the function sense. This means that the entries in q 
#should sum to one.

#Modify the code below so that the function sense, which 
#takes p and Z as inputs, will output the NON-normalized 
#probability distribution, q, after multiplying the entries 
#in p by pHit or pMiss according to the color in the 
#corresponding cell in world.


p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
Z = 'red'
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

print sense(p,Z)