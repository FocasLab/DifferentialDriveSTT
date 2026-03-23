#2-D problem with 2nd degree of tube polynomial
#One single Obstacle at {(1,2.5),(2.5,3)}

import numpy as np
import matplotlib.pyplot as plt
import z3
from z3 import *
import progressbar
import pandas as pd
import time
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation

# To run loaded model, change Fine tune to 1
Fine_tune = 0

start = time.time()

################## Define the hyperparameters (Start, Target, Tube degree, Obstacles)  #####################
dim = 2         #no of dimension
N = 180          #no of timestep
t0 = 1e-5       #start time
tf = 90          #final time
deg = 3         #degree of tube polynomial (same for x_c,y_c,r)
xdim = (deg+1)*(dim+1)  #length of unknown vector x

### Start and Target regions (X gives x coord, Y gives y coord, r gives radius)
S_X = 0.35
S_Y = 0.35
S_r = 0.2
T_X = 2.65
T_Y = 2.65
T_r = 0.2

### State space bounds
X_L = 0         # lower limit of x dimension
Y_L = 0         # lower limit of y dimension
X_U = 3         # upper limit of x dimension
Y_U = 3         # upper limit of y dimension

## Obstacle region (center_x, center_y, radius)
obstacles = [
    (1.0, 1.0, 0.25),  # Obs 1
    (2.5, 1.5, 0.25),  # Obs 2
    (0.75, 2.25, 0.25),   # Obs 3
    (1.5, 1.75, 0.25)    # Obs 4
]

m = deg+1

###############################Solve the constraints##################################
if Fine_tune == 0:
    # Define variables
    x = [Real('x{}'.format(i)) for i in range(xdim)]

    #Initialize Solver
    s = Solver()

    ################## Reduce search space (some kind of supervised) ######################
    # for x_c
    s.add(x[0]>=0.3,x[0]<=0.4)
    s.add(x[1]>=0.04,x[1]<=0.07)
    s.add(x[2]>=-0.001,x[2]<=-0.0007)
    s.add(x[3]>=3e-6,x[3]<=5e-6)
    # for y_c
    s.add(x[4]>=0.3,x[4]<=0.4)
    s.add(x[5]>=-0.005,x[5]<=-0.003)
    s.add(x[6]>=0.0006,x[6]<=0.0008)
    s.add(x[7]>=-5e-6,x[7]<=-4e-6)
    # for r
    s.add(x[8]==0.2)
    s.add(x[9]==0)
    s.add(x[10]==0)
    s.add(x[11]==0)

    ####################### Condition 1: Equality constraint corresponding to start and target #################################
    Aeq = np.zeros((2*(dim+1), xdim))
    for j in range(xdim):
        if j<m:
            Aeq[0,j] = t0**j
            Aeq[3,j] = tf**j
        elif j>=m and j<2*m:
            Aeq[1,j] = t0**(j-m)
            Aeq[4,j] = tf**(j-m)
        elif j>=2*m and j<3*m:
            Aeq[2,j] = t0**(j-2*m)
            Aeq[5,j] = tf**(j-2*m)
    Beq = np.transpose(np.array([S_X,S_Y,S_r,T_X,T_Y,T_r]))

    constraints_eq = [sum([Aeq[i][j] * x[j] for j in range(len(Aeq[0]))]) == Beq[i] for i in range(len(Beq))]
    s.add(constraints_eq)

    ###################### Condition 2: State-space constraint and radius constraint #########################
    t = np.linspace(t0,tf,N)
    for k in range(N):
        gam_x = 0
        gam_y = 0
        gam_r = 0
        for j in range(m):
            gam_x = gam_x + x[j]*t[k]**j
            gam_y = gam_y + x[j+m]*t[k]**j
            gam_r = gam_r + x[j+2*m]*t[k]**j
        
        state_constraint = z3.And(gam_x-X_L-gam_r>0, gam_y-Y_L-gam_r>0, X_U-gam_x-gam_r>0, Y_U-gam_y-gam_r>0, gam_r>0)
        s.add(state_constraint)

    ###################### Condition 3: The avoid constraints ###########################
    for k in range(N):  # time instaces
        gam_x = 0
        gam_y = 0
        gam_r = 0
        for j in range(m):
            gam_x = gam_x + x[j]*t[k]**j
            gam_y = gam_y + x[j+m]*t[k]**j
            gam_r = gam_r + x[j+2*m]*t[k]**j
        
        for (ox, oy, R) in obstacles:
            s.add( (gam_x - ox)**2 + (gam_y - oy)**2 - (gam_r + R)**2 > 0 )

    ###################### Solve and satisfy the constraints ###########################
    xi = np.zeros(xdim)
    print("Constraints are ready")
    if s.check() == sat:
        print("Model Checking start")
        model = s.model()
        # Print the solution
        print("Solution:")
        for i in range(len(x)):
            # x[i] = model[x[i]]
            xi[i] = float(model[x[i]].numerator().as_long())/float(model[x[i]].denominator().as_long())
            print("{} = {}".format(x[i], xi[i]))

            # save the solution
            df = pd.DataFrame(xi)
            df.to_excel('test4.xlsx', index=False)

    else:
        print("Unsatisfiable")

    end = time.time()
    print("Reqd time:")
    print(end - start)

else:
    xi = pd.read_excel('test4.xlsx', header=None)
    xi = xi.iloc[:,0].to_numpy()
    xi = xi[1:]

# Plot the results
t = np.linspace(t0,tf,N)
theta = np.linspace(0,2*np.pi,N)
T,Theta = np.meshgrid(t, theta)

# degree 1
# r = xi[2*m] + xi[2*m+1]*t
# X = (xi[0] + xi[1]*T) + r*np.cos(Theta)
# Y = (xi[m] + xi[m+1]*T) + r*np.sin(Theta)
# degree 2
# r = xi[2*m] + xi[2*m+1]*t + xi[2*m+2]*t**2
# X = (xi[0] + xi[1]*T + xi[2]*T**2) + r*np.cos(Theta)
# Y = (xi[m] + xi[m+1]*T + xi[m+2]*T**2) + r*np.sin(Theta)
# degree 3
r = xi[2*m] + xi[2*m+1]*t + xi[2*m+2]*t**2 + xi[2*m+3]*t**3
X = (xi[0] + xi[1]*T + xi[2]*T**2 + xi[3]*T**3)+r*np.cos(Theta)
Y = (xi[m] + xi[m+1]*T + xi[m+2]*T**2 + xi[m+3]*T**3)+r*np.sin(Theta)
Z = T

# Plot the 3D tube
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, Z, color='g', alpha=0.4, edgecolor='k')
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Time")
ax.set_xlim(0, 3)
ax.set_ylim(0, 3)
ax.set_zlim(0, tf)
plt.show()