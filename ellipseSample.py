import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 22

#load the grid, set start and goal <x, y> positions, number of iterations, step size
grid = np.load('cspace.npy')
start = np.array([300.0, 300.0])
goal = np.array([1400.0, 775.0])
numIterations = 700
stepSize = 75
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

fig = plt.figure("Sampling in An Ellipse")
plt.imshow(grid, cmap='binary')
plt.plot(start[0],start[1],'ro')
plt.plot(goal[0],goal[1],'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

#plot an ellipse, properly rotated
x_center = 0.5*(start[0]+goal[0])
y_center = 0.5*(start[1]+goal[1])
#only costs and radii change
c_min = 1198.0
c_best = 1541.0
rad_x = c_best/2
rad_y = np.sqrt(c_best**2-c_min**2)/2
t = np.linspace(0, 2*np.pi, 100)

#rotation angle (fixed)
tht = np.arctan2(goal[1]-start[1], goal[0]-start[0])

plt.plot(rad_x*np.cos(t)*np.cos(tht) - rad_y*np.sin(t)*np.sin(tht) +\
         x_center, rad_x*np.cos(t)*np.sin(tht) - rad_y*np.sin(t)*np.cos(tht) + y_center)

#sample points inside this ellipse

for i in range(300):
    x = random.randint(1, grid.shape[1])
    y = random.randint(1, grid.shape[0])
    
    if (((x-x_center)*np.cos(-tht)+(y-y_center)*np.sin(-tht))**2/rad_x**2 +\
        ((x-x_center)*np.sin(-tht)+(y-y_center)*np.cos(-tht))**2/rad_y**2 ) < 1:
        plt.plot(x,y,'go')
        