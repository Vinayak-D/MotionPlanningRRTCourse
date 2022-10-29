#Informed RRT Star algorithm
import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 15

#tree Node class
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX                #X Location
        self.locationY = locationY                #Y Location  
        self.children = []                        #children list   
        self.parent = None                        #parent node reference 
        
#Informed RRT Star Algorithm class
class InformedRRTStarAlgorithm():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])          #The RRT (root position) (has 0 cost)
        self.goal = treeNode(goal[0], goal[1])                  #goal position (initialize to a high cost)
        self.nearestNode = None                                 #nearest node            
        self.iterations = min(numIterations, 1400)              #number of iterations to run
        self.grid = grid                                        #the map
        self.rho = stepSize                                     #length of each branch   
        self.nearestDist = 10000                                #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                   #number of waypoints
        self.Waypoints = []                                     #the waypoints
        self.serializedTree = []                                #the serialized tree as list of lists
        self.searchRadius = self.rho*2                          #the radius to search for finding neighbouring vertices 
        self.neighbouringNodes = []                             #neighbouring nodes  
        self.goalArray = np.array([goal[0],goal[1]])            #goal as an array
        self.goalCosts = [10000]                                #the costs to the goal (ignore first value)
        self.initialPathFound = False                           #trigger when initial path obtained
        self.ellipseAngle = np.arctan2(goal[1]-start[1], goal[0]-start[0])
        self.xCenterEllipse = 0.5*(start[0]+goal[0])            #x-center of ellipse
        self.yCenterEllipse = 0.5*(start[1]+goal[1])            #y-center of ellipse
        self.c_min = np.sqrt((goal[1]-start[1])**2 + (goal[0]-start[0])**2) 
        self.a = np.linspace(0, 2*np.pi, 100)                   #angle for parametric ellipse plots
            
    #add the node to the nearest node, and add goal if necessary (TODO--------)     
    def addChild(self, treeNode):
        if (treeNode.locationX == self.goal.locationX):
            #append goal to nearestNode's children
            #and set goal's parent to nearestNode
        else:    
            #append this node to nearestNode's children
            #set the parent to nearestNode
        
    #sample random point within grid limits (DONE)
    def sampleAPoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        return point
    
    #sample random point within ellipse limits (DONE)
    def checkIfInEllipse(self, pointX, pointY, c_best):
        rad_x = c_best/2
        rad_y = np.sqrt(c_best**2-self.c_min**2)/2
        if (((pointX - self.xCenterEllipse)*np.cos(-self.ellipseAngle) + (pointY - self.yCenterEllipse)*np.sin(-self.ellipseAngle))**2/rad_x**2 + \
            ((pointX - self.xCenterEllipse)*np.sin(-self.ellipseAngle) + (pointY - self.yCenterEllipse)*np.cos(-self.ellipseAngle))**2/rad_y**2 ) < 1:
            return True
        return False
    
    #plot an ellipse with parameters (DONE)
    def plotEllipse(self, c_best):
        rad_x = c_best/2
        rad_y = np.sqrt(c_best**2-self.c_min**2)/2
        plt.plot(rad_x*np.cos(self.a)*np.cos(self.ellipseAngle) - rad_y*np.sin(self.a)*np.sin(self.ellipseAngle) + self.xCenterEllipse,\
                 rad_x*np.cos(self.a)*np.sin(self.ellipseAngle) - rad_y*np.sin(self.a)*np.cos(self.ellipseAngle) + self.yCenterEllipse)
    
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
        distance = int(self.distance(locationStart, locationEnd))
        for i in range(distance):
            testPoint[0] = min(1799, locationStart.locationX + i*u_hat[0])
            testPoint[1] = min(1119,locationStart.locationY + i*u_hat[1])
            if self.grid[round(testPoint[1]).astype(np.int64),round(testPoint[0]).astype(np.int64)] == 1:
                return True
        return False

    #find the unit vector between 2 locations (DONE)
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        v_norm = np.linalg.norm(v)
        if v_norm < 1:
            v_norm = 1
        u_hat = v/v_norm
        return u_hat
    
    #find the nearest node from a given (unconnected) point (Euclidean distance) (TODO--------)
    def findNearest(self, root, point):
        if not root:
            return
        #find distance between root and point use distance method,
        #if it's lower than or equal to nearestDist then
        #update nearestNode to root
        #update nearestDist to the distance from line 110
        #recursive call
        for child in root.children:
            self.findNearest(child, point)
    
    #find neighbouring nodes (TODO--------)        
    def findNeighbouringNodes(self,root,point):
        if not root:
            return
        #find distance between root and point (dist)
        #add root to neighbouringNodes if dist is less than or equal to searchRadius
        #recursive call
        for child in root.children:
            self.findNeighbouringNodes(child, point)        

    #find euclidean distance between a node and an XY point (DONE)
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)         
        return dist
    
    #check if the goal is within stepsize (rho) distance from point, return true if so otherwise false (TODO--------)
    def goalFound(self,point):
        pass #delete this when you're done
    
    #reset: set nearestNode to None, nearestDistance to 10K and neighbouringNodes to empty array (TODO--------)
    def resetNearestValues(self):
        pass #delete this when you're done
    
    #trace the path from goal to start, since have to reset if called many times, do this iteratively (TODO--------)
    def retracePath(self):
        self.numWaypoints = 0
        self.Waypoints = []
        goalCost = 0
        goal = self.goal
        while goal.locationX != self.randomTree.locationX:
            #add 1 to numWaypoints
            #extract the X Y location of goal in a numpy array 
            #insert this array to waypoints (from the beginning)
            #add distance between the node and it's parent to goalCost (goalCost keeps increasing)     
            goal = goal.parent   
        self.goalCosts.append(goalCost)    
        
    #find unique path length from root of a node (cost) (DONE)
    def findPathDistance(self, node):
        costFromRoot = 0
        currentNode = node
        while currentNode.locationX != self.randomTree.locationX:
            costFromRoot += self.distance(currentNode, np.array([currentNode.parent.locationX, currentNode.parent.locationY])) 
            currentNode = currentNode.parent   
        return costFromRoot    
        
        
#end of class definitions
#------------------------------------------------------------------------------#
        
#load the grid, set start and goal <x, y> positions, number of iterations, step size
grid = np.load('cspace.npy')
start = np.array([575.0, 510.0])
goal = np.array([1300.0, 710.0])
numIterations = 1400
stepSize = 75
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

fig = plt.figure("Informed RRT Star Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0],start[1],'ro')
plt.plot(goal[0],goal[1],'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')
    
#Begin
iRRTStar = InformedRRTStarAlgorithm(start, goal, numIterations, grid, stepSize)
plt.pause(4)

#Informed RRT Star algorithm 
#iterate
for i in range(iRRTStar.iterations):
    
    #Reset nearest values
    iRRTStar.resetNearestValues()
    print("Iteration: ",i)
    
    #algorithm begins here
    point = iRRTStar.sampleAPoint()
    
    #if an initial path has been found
        #get 'c_best', the last element of goalCosts
        #if the point is not in ellipse (pass appropriate arguments to 'checkIfInEllipse')
            continue #go to next iteration in for loop
            
    #find the nearest node w.r.t to the point (just call the method do not return anything)
    #steer to point, set the returned variable to ('new')
    #if not in obstacle
    if not iRRTStar.isInObstacle(iRRTStar.nearestNode, new):
        iRRTStar.findNeighbouringNodes(iRRTStar.randomTree, new)
        min_cost_node = iRRTStar.nearestNode
        min_cost = iRRTStar.findPathDistance(min_cost_node)
        min_cost = min_cost + iRRTStar.distance(iRRTStar.nearestNode, new)
        
        #connect along minimum cost path (TODO-------)
        
        #for each node in neighbouringNodes
            #find the cost from the root (findPathDistance)
            #add the distance between the node and the new point ('new') to the above cost (use the relevant method)
        #if node and new are obstacle free AND the cost is lower than min_cost (use the relevant method)
            #set the min_cost_node to this node
            #set the min_cost to this cost
        #update nearest node to min_cost_node, create a treeNode object from the new point - call this newNode ('new[0],new[1]')
        #SIDE NOTE: if neighbouringNodes is empty, it'll add to the original nearest node (obstacle free)  
  
        #plot for display
        plt.pause(0.01)
        plt.plot([iRRTStar.nearestNode.locationX, new[0]], [iRRTStar.nearestNode.locationY, new[1]],'go', linestyle="--")  
        
        #rewire tree (TODO-------)    
        #for each node in neighbouringNodes
            #set a variable: 'cost' to min_cost
            #add the distance between 'new' and node to cost
            #if node and new are obstacle free AND the cost is lower than the distance from root to vertex
            #set the parent of node to 'newNode' (see line 222)
                
        #if goal found, and the projected cost is lower, then append to path, trigger flag let it sample more (TODO-------)
        point = np.array([newNode.locationX, newNode.locationY])
        if iRRTStar.goalFound(point):
            #calculate projectedCost

            if #projectedCost is lower than the last goalCosts array element:
                #trigger initialPathFound flag to true
                #add goal to nearest Node (addChild)
                plt.plot([iRRTStar.nearestNode.locationX, iRRTStar.goalArray[0]], [iRRTStar.nearestNode.locationY, iRRTStar.goalArray[1]],'go', linestyle="--") 
                #retrace path (call the method)
                
                #this part is done (lines 248-257)
                print("Goal Cost: ", iRRTStar.goalCosts)
                plt.pause(0.25)
                iRRTStar.Waypoints.insert(0,start)
                #plot the waypoints
                for i in range(len(iRRTStar.Waypoints)-1):
                    plt.plot([iRRTStar.Waypoints[i][0], iRRTStar.Waypoints[i+1][0]], [iRRTStar.Waypoints[i][1], iRRTStar.Waypoints[i+1][1]],'ro', linestyle="--")
                    plt.pause(0.01)
                #plot ellipse
                c_best = iRRTStar.goalCosts[-1]
                iRRTStar.plotEllipse(c_best)
 
print(iRRTStar.goalCosts[1:-1])
