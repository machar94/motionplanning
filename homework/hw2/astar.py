import heapq
import numpy as np
import IPython
import math
from copy import deepcopy


class aStarSearch():
    """ Implements the a star search algorithm """

    def __init__(self, start, goal, hueristic, connectedness, robot, env, handles, steps):
        # Assumption that the start position is exactly center of a node
        # Assumption that the end position is exactly center of a node
        self.hueristic = hueristic
        self.start = self.Node(coord=start, g=0, h=0)
        self.goal = self.Node(coord=goal, g=0, h=0)
        self.start.h = self.costToGoal(self.start)
 
        self.connectedness = connectedness
        self.heap = []
        self.robot = robot
        self.env = env
        self.unitLength = steps
        self.handles = handles
        self.moveCost = min(steps)
        self.metaPath   = []

    class Node():
        def __init__(self, coord, g, h):
            self.coord = coord
            self.g = g
            self.h = h
            self.parent = self

        def getF(self):
            return self.g + self.h

    def search(self):
        path = []
        closedSet = set()

        # Add the start to the beginning of the priority queue
        heapq.heappush(self.heap, (self.start.getF(), self.start))

        # While loop, while the priority queue is not empty
        while len(self.heap) > 0:
            # Get the first node on the top of the queue
            currHeapObj = heapq.heappop(self.heap)
            closedSet.add(tuple(currHeapObj[1].coord))

            # Check if current node is goal - if so we are done
            if (self.isNodeGoal(currHeapObj[1])):
                path = self.traceBackToGoal(currHeapObj[1])
                print "Path Cost: %f" % currHeapObj[1].g
                self.printPath()
                break

            # Retrieve all available nodes through possible control
            nextNodes = self.getConnectedSpaces(currHeapObj[1])

            for n in nextNodes:

                tupN = tuple(n.coord)
                if tupN in closedSet:
                    continue

                # Add a new node if not in open list already
                heapObj = self.checkIfInHeap(n)
                if heapObj is None:
                    heapq.heappush(self.heap, (n.getF(), n))

                else:
                    if n.g < heapObj[1].g:
                        heapObj[1].g = n.g
                        heapObj[1].parent = n.parent

                        heapq.heappush(self.heap, (heapObj[1].getF(), heapObj[1]))
                        heapq.heapify(self.heap)
                    else:
                        heapq.heappush(self.heap, heapObj)
                        heapq.heapify(self.heap)

        return path

    def costToGoal(self, currNode):
        return self.costToNode(currNode, self.goal)

    def costToNode(self, node1, node2):
        n1 = node1.coord
        n2 = node2.coord

        if self.hueristic == "euclidian":
            dist = math.sqrt((n2[0]-n1[0])**2 + (n2[1]-n1[1])**2)
            diff = self.angleDiff(n1[2], n2[2])
            dist = dist + diff
        elif self.hueristic == "manhattan":
            dist = abs(n2[0]-n1[0]) + abs(n2[1]-n1[1])
            diff = self.angleDiff(n1[2], n2[2])
            dist = dist + diff
        # Dijkstra's Algorithm
        else:
            dist = 0
        return dist
    
    def angleDiff(self, a1, a2):
        """ Wraps angle diff to [0, PI] """
        diff = a1 - a2
        if (abs(diff) > math.pi):
            if (diff > 0):
                diff = diff - 2*math.pi
            else:
                diff = diff + 2*math.pi
        return abs(diff)


    def traceBackToGoal(self, node):
        path = []
        self.metaPath = []

        while node.parent != node:
            path.append(deepcopy(node.coord.tolist()))
            self.metaPath.append(tuple(deepcopy(node.coord.tolist())) + tuple([node.g]))
            node = node.parent
        
        # Add start node as well
        path.append(deepcopy(node.coord.tolist()))
        self.metaPath.append(tuple(deepcopy(node.coord.tolist())) + tuple([node.g]))

        path.reverse()
        self.metaPath.reverse()
        return path


    def getConnectedSpaces(self, node):
        spaces = []

        if (self.connectedness == 4):
            moves = [[0, 1, 0,-1, 0, 0],[1, 0, -1, 0, 0, 0],[0, 0, 0, 0, 1, -1]]
            
            for i in range(0,len(moves[0])):
                location = np.array([node.coord[0]+self.unitLength[0]*moves[0][i], \
                    node.coord[1]+self.unitLength[1]*moves[1][i], \
                    (node.coord[2]+self.unitLength[2]*moves[2][i])%(2*np.pi)])
                self.robot.SetActiveDOFValues(location)
                if (self.env.CheckCollision(self.robot) == False):
                    neighbor = self.Node(coord=location, g=0, h=0)
                    neighbor.parent = node
                    neighbor.h = self.costToGoal(neighbor)
                    neighbor.g = node.g + self.costToNode(neighbor, node)
                    spaces.append(neighbor)

                    blue = np.array([[0,0,1]])
                    self.handles.append(self.env.plot3(points=np.array((location[0],location[1],0.05)),pointsize=3.0,colors=blue))

                else:
                    red = np.array([[1,0,0]])
                    self.handles.append(self.env.plot3(points=np.array((location[0],location[1],0.05)),pointsize=3.0,colors=red))


        if (self.connectedness == 8):
            for i in range(-1,2):
                for j in range (-1,2):
                    for k in range (-1,2):
                        if i!=0 or j!=0 or k!=0:
                            location = np.array([node.coord[0]+i*self.unitLength[0], \
                                node.coord[1]+j*self.unitLength[1], \
                                (node.coord[2]+k*self.unitLength[2])%(2*np.pi)])
                            self.robot.SetActiveDOFValues(location)
                            if (self.env.CheckCollision(self.robot) == False):
                                neighbor = self.Node(coord=location, g=0, h=0)
                                neighbor.parent = node
                                neighbor.h = self.costToGoal(neighbor)
                                neighbor.g = node.g + self.costToNode(neighbor, node)
                                spaces.append(neighbor)

                                blue = np.array([[0,0,1]])
                                self.handles.append(self.env.plot3(points=np.array((location[0],location[1],0.05)),pointsize=3.0,colors=blue))

                            else:
                                red = np.array([[1,0,0]])
                                self.handles.append(self.env.plot3(points=np.array((location[0],location[1],0.05)),pointsize=3.0,colors=red))

        return spaces

    def checkIfNodesEqual(self, node1, node2):
        epsilon = 0.000001
        if (self.almostEqual(node1.coord[0], node2.coord[0], epsilon) and \
            self.almostEqual(node1.coord[1], node2.coord[1], epsilon) and \
            self.almostEqual(node1.coord[2], node2.coord[2], epsilon)):
            return True

        else:
            return False

    def almostEqual(self, val1, val2, tolerance):
        return abs(val1 - val2) < tolerance

    def checkIfInHeap(self, node):
        for i in range(0, len(self.heap)):
            if self.checkIfNodesEqual(self.heap[i][1],node):
                return self.heap.pop(i)
        return None

    def isNodeGoal(self, node):
        closeEnough = math.sqrt(self.unitLength[0]**2 + self.unitLength[1]**2)
        if math.sqrt((node.coord[0]-self.goal.coord[0])**2 + (node.coord[1]-self.goal.coord[1])**2) <= closeEnough:
            return True
        else:
            return False

    def printPath(self):
        f = open("path.txt","w")
        f.write("==== Printing Path ====\n")
        for i in xrange(0,len(self.metaPath)):
            f.write("%.2f "*len(self.metaPath[i]) % self.metaPath[i] + "\n")
                
        f.write("==== Printing Path ====\n")
        f.close()