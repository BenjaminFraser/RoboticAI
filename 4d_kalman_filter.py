# Fill in the matrices P, F, H, R and I at the bottom
#
# This question requires NO CODING, just fill in the 
# matrices where indicated. Please do not delete or modify
# any provided code OR comments. Good luck!

from math import *

class matrix:
    """implements basic operations of a matrix class"""
    
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]
    
    def identity(self, dim):
        """Return a valid identity matrix for the given matrix dimensions"""
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
    
    def show(self):
        for i in range(self.dimx):
            print self.value[i]
        print ' '
    
    def __add__(self, other):
        """Modifies the default action given by '+' operations with a matrix object"""
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
    
    def __sub__(self, other):
        """Modified the default action given by a '-' operation with matrix objects"""
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self, other):
        """Modifies the default action given by multiplication 'x' actions on matrices"""
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
    
    def transpose(self):
        """Returns a transpose of the given matrix object instance"""
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions
    
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res
    
    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res
    
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)


########################################

def filter(x, P):
    for n in range(len(measurements)):
        
        # prediction
        x = (F * x) + u
        P = F * P * F.transpose()
        
        # measurement update
        Z = matrix([measurements[n]])
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P
    
    print 'x= '
    x.show()
    print 'P= '
    P.show()

########################################

print "### 4-dimensional example ###"

measurements = [[5., 10.], [6., 8.], [7., 6.], [8., 4.], [9., 2.], [10., 0.]]
initial_xy = [4., 12.]

# measurements = [[1., 4.], [6., 0.], [11., -4.], [16., -8.]]
# initial_xy = [-4., 8.]

# measurements = [[1., 17.], [1., 15.], [1., 13.], [1., 11.]]
# initial_xy = [1., 19.]

#set dt - time interval between measurements to be 0.1.
dt = 0.1

# set initial x, y positions to initial positions, and x, y velocities to zero
x = matrix([[initial_xy[0]], [initial_xy[1]], [0.], [0.]]) # initial state (location and velocity)
u = matrix([[0.], [0.], [0.], [0.]]) # external motion - this is external change to motion, not velocity of the robot.
                                     # we would use this if we had control over the motion of the robot and wanted to make a change.

#### DO NOT MODIFY ANYTHING ABOVE HERE ####
#### fill this in, remember to use the matrix() function!: ####

P =  matrix([[0, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 0, 1000, 0],
             [0, 0, 0, 1000]]) # initial uncertainty: 0 for positions x and y, 1000 for the two velocities

F =  matrix([[1., 0, dt, 0],
             [0, 1., 0, dt],
             [0, 0, 1., 0],
             [0, 0, 0, 1]]) # generalized to 4d

H =  matrix([[1, 0, 0, 0],
             [0, 1, 0, 0]]) # measurement function: reflect the fact that we observe x and y but not the two velocities

R =  matrix([[dt, 0], [0, dt]]) # measurement noise uncertainty: use 2x2 matrix with 0.1 as main diagonal
                                # this assumes each measurement has an uncertainty covariance of 0.1

I =  matrix([[1., 0, 0, 0],
             [0, 1., 0, 0],
             [0, 0, 1., 0],
             [0, 0, 0, 1.]]) # 4d identity matrix - all elements of the principle diagonal are all ones.
                             # when a compatible matrix is multiplied by an identity 

# Notes on the above:
# H is a projection matrix, which is designed to create a 1 column and 2 row matrix that only
# contains the positions x and y, and not the velocities of each, since we can't directly measure them.
# F is a state transition matrix, which must determine the new states of positions based on the current positional
# and velocity data within x. New data for x is determined using this matrix, which must be 4 x 4 since
# we have 4 dimensions within x: x_position, y_position, x_velocity, y_velocity.

###### DO NOT MODIFY ANYTHING HERE #######

filter(x, P)
