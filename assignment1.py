#RRT algorithm
import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 22

#tree Node class (DONE)
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX                #X Location
        self.locationY = locationY                #Y Location  
        self.children = []                        #children list   
        self.parent = None                        #parent node reference 

#RRT Algorithm class 
class RRTAlgorithm():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])         #The RRT (root position)
        self.goal = treeNode(goal[0], goal[1])                 #goal position
        self.nearestNode = None                                #nearest node            
        self.iterations = min(numIterations, 250)              #number of iterations to run
        self.grid = grid                                       #the map
        self.rho = stepSize                                    #length of each branch 
        self.path_distance = 0                                 #total path distance  
        self.nearestDist = 10000                               #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                  #number of waypoints
        self.Waypoints = []                                    #the waypoints
        
    #add the node to the nearest node, and add goal if necessary (TODO--------)    
    def addChild(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            #append goal to nearestNode's children
            #and set goal's parent to nearestNode
            pass #delete this when you're done
        else:
            #create a tree node from locationX, locationY
            #append this node to nearestNode's children
            #set the parent to nearestNode
            pass #delete this when you're done
        
    #sample random point within grid limits (DONE)
    def sampleAPoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        return point
    
    #steer a distance stepSize from start location to end location (keep in mind the grid limits) (DONE)
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1]-1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0]-1
        return point
    
    #check if obstacle lies between the start and end point of the edge (DONE)
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho):
            testPoint[0] = min(grid.shape[1]-1, locationStart.locationX + i*u_hat[0])
            testPoint[1] = min(grid.shape[0]-1,locationStart.locationY + i*u_hat[1])
            if self.grid[round(testPoint[1]).astype(np.int64),round(testPoint[0]).astype(np.int64)] == 1:
                return True
        return False

    #find the unit vector between 2 locations (DONE)
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    #find the nearest node from a given (unconnected) point (Euclidean distance) (TODO--------)
    def findNearest(self, root, point):
        if not root:
            return
        #find distance between root and point use distance method,
        #if it's lower than or equal to nearestDist then
        #update nearestNode to root
        #update nearestDist to the distance from line 84
        for child in root.children:
            self.findNearest(child, point)

    #find euclidean distance between a node object and an XY point (TODO--------)
    def distance(self, node1, point):
        #dist = complete this and uncomment after
        return dist
    
    #check if the goal is within stepsize (rho) distance from point, return true if so otherwise false (TODO--------)
    def goalFound(self,point):
        pass #delete this when you're done
    
    #reset: set nearestNode to None and nearestDistance to 10000 (TODO--------)
    def resetNearestValues(self):
        pass #delete this when you're done
        
    #trace the path from goal to start (TODO--------)
    def retraceRRTPath(self,goal):
        
        if goal.locationX == self.randomTree.locationX:
            return
        
        #add 1 to numWaypoints
        #extract the X Y location of goal in a numpy array 
        #insert this array to waypoints (from the beginning)
        #add rho to path_distance
        
        self.retraceRRTPath(goal.parent)   

        
#end of class definitions
#------------------------------------------------------------------------------------------------------------------------#
        
#load the grid, set start and goal <x, y> positions, number of iterations, step size
grid = np.load('cspace.npy')
start = np.array([100.0, 100.0])
goal = np.array([1600.0, 750.0])
numIterations = 200
stepSize = 100
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0],start[1],'ro')
plt.plot(goal[0],goal[1],'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')
    
#Begin
rrt = RRTAlgorithm(start, goal, numIterations, grid, stepSize)
plt.pause(2)

#RRT algorithm (TODO-------)
#iterate
for i in range(rrt.iterations):
    #Reset nearest values, call the resetNearestValues method
    print("Iteration: ",i)
    #algorithm begins here-------------------------------------------------------------------------------
    
    #sample a point (use the appropriate method, store the point in variable)- call this variable 'point' to match Line 151

    #find the nearest node w.r.t to the point (just call the method do not return anything)
    new = rrt.steerToPoint(rrt.nearestNode, point) #steer to a point, return as 'new'
    #if not in obstacle
    if not rrt.isInObstacle(rrt.nearestNode, new):
        #add new to the nearestnode (addChild), again no need to return just call the method
        plt.pause(0.10)
        plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]],'go', linestyle="--")  
        #if goal found (new is within goal region)
        if (rrt.goalFound(new)):
            #append goal to path
            #retrace
            break

#-----------------------------------------------------------------------------------------------------------------------------#

#Add start to waypoints (DONE)
rrt.Waypoints.insert(0,start)
print("Number of waypoints: ", rrt.numWaypoints)
print("Path Distance (m): ", rrt.path_distance)    
print("Waypoints: ", rrt.Waypoints)

#plot the waypoints in red (DONE)
for i in range(len(rrt.Waypoints)-1):
    plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]],'ro', linestyle="--")
    plt.pause(0.10)
