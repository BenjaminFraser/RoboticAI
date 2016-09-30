# an alternative roulette wheel particle filter resampling algorithm
# beta is initialised as a random number between (0-1) * 2.0 * max_importance_weight
# note: the value of 2.0 is completely random, as stipulated by Sebastian T. (Udacity)
# index is also randomly initialised between 0-1 * N
# Whilst beta is larger than w[index], beta is made smaller by the current w[index], and index is
# incremented by 1, until beta becomes smaller than the current w[index], at which point, we choose
# the current particle to append to our new particle list.

index = int(random.random() * N)
beta = 0.0
mw = max(w)
for i in range(N):
   beta += random.random() * 2.0 * mw
   while beta > w[index]:
       beta -= w[index]
       index = (index + 1) % N
   p3.append(p[index])