#Assignment 0: Motion Planning using Tree Algorithms

#import what we need
import numpy as np
import matplotlib.pyplot as plt
import random

#load the grid 
grid = np.load('cspace.npy')
print('Grid dimensions')
print(grid.shape)
#start and goal positions as numpy arrays
start = np.array([100.0, 100.0])
goal = np.array([1600.0, 750.0])
#the goal region
goalCircle = plt.Circle((goal[0], goal[1]), 25, color='b', fill = False)
#a new figure
fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
#plot the start and goal points
plt.plot(start[0],start[1],'ro')
plt.plot(goal[0],goal[1],'bo')
#plot the goal region circle
ax = fig.gca()
ax.add_patch(goalCircle)
#label the X and Y axes
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

#tree Node class
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX                #X Location
        self.locationY = locationY                #Y Location  
        self.children = []                        #children list   
        self.parent = None                        #parent node reference 
        
#function to sample a point within grid limits (note the [y,x])
def sampleAPoint():
    x = random.randint(1, grid.shape[1])
    y = random.randint(1, grid.shape[0])
    point = np.array([x, y])
    return point   
     
#set the root of the tree
root = treeNode(start[0],start[1])

#sample 2 points
point = sampleAPoint()
point = treeNode(point[0],point[1])
point2 = sampleAPoint()
point2 = treeNode(point2[0],point2[1])

#add this to the children of root
root.children.append(point)
root.children[0].children.append(point2)

#TODO: set the parent of point (what will it be?)-----------------


#TODO: set the parent of point2 (what will it be?)----------------


#traverse the entire tree (recursion)
def traverseTree(root):
    if not root:
        return
    print(root.locationX, root.locationY)
    for child in root.children:
        traverseTree(child)

#plot on figure
plt.plot([root.locationX, point.locationX, point2.locationX], [root.locationY, point.locationY, point2.locationY],'go', linestyle="--")  

#traverse tree
print('\nTree nodes')
traverseTree(root)

#Arrays (lists) in Python
WaypointArray = []
p2 = np.array([100,100])
p1 = np.array([70,70])

print('\n')
#inserting from the beginning
WaypointArray.insert(0,p2)
print(WaypointArray)
WaypointArray.insert(0,p1)
print(WaypointArray)
#inserting from the end
p3 = np.array([60,60])
WaypointArray.append(p3)
print(WaypointArray)



    

    