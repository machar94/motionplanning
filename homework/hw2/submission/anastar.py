#!/usr/bin/env python
# -*- coding: utf-8 -*-
import heapq
import numpy as np
import IPython
import math
from copy import deepcopy
import time

G_INIT_VAL = 100000.0
E_INIT_VAL = 100000.0
EPSILON    = 0.000001



class anaStarSearch():
    """ Implements the ANA* search algorithm
            - hueristic: euclidian
            - connectedness: 8 """
    
    class Node():
        def __init__(self, coord, g, h):
            self.coord = coord
            self.g = g
            self.h = h
            self.parent = self

        def getE(self, G):
            """ Return -E since heapq sorts via smallest value """
            return -((G - self.g) / self.h)

        def getF(self):
            return self.g + self.h
        
        def __eq__(self, node):
            if isinstance(node, self.__class__):
                if (almostEqual(node.coord[0], self.coord[0], EPSILON) and \
                    almostEqual(node.coord[1], self.coord[1], EPSILON) and \
                    almostEqual(node.coord[2], self.coord[2], EPSILON)):
                    return True
            return False


    def __init__(self, start, goal, timeout, steps, robot, env, handles, startTime):
        self.G          = G_INIT_VAL
        self.E          = E_INIT_VAL
        self.heap       = []
        self.robot      = robot
        self.env        = env
        self.steps      = steps
        self.handles    = handles
        self.path       = []
        self.visitedSet = []
        self.startTime  = startTime
        self.timeout    = timeout
        self.connectedness = 8
        self.metaPath   = []
        self.pathFound  = True
        self.solution   = []

        # Initialize start and end nodes
        self.start = self.Node(coord=start, g=0, h=0)
        self.goal = self.Node(coord=goal, g=G_INIT_VAL, h=0)
        self.start.h = self.costToGoal(self.start)

        # Debugging 
        self.logFile = open("log.txt", "w")
        np.set_printoptions(precision=3)
        np.set_printoptions(suppress=True)


    def costToGoal(self, node):
        """ Euclidean Distance """
        return self.costToNode(node, self.goal)

    def costToNode(self, node1, node2):
        """ Proper cost calculation """
        n1 = node1.coord
        n2 = node2.coord
        dist = math.sqrt((n2[0]-n1[0])**2 + (n2[1]-n1[1])**2)
        diff = self.angleDiff(n1[2], n2[2])
        dist = dist + diff
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

    def search(self):

        # Add the start to the beginning of the priority queue
        heapq.heappush(self.heap, (self.start.getE(self.G), self.start))

        while (len(self.heap)) > 0 and ((time.clock() - self.startTime) < self.timeout):
            self.logFile.write("Searching for a solution\n")
            path = self.improveSolution()
            
            # Solution saved if an actual path is returned
            if path:
                # for i in range(0, len(path)):
                #     green = np.array([0,1,0])
                #     self.handles.append(self.env.plot3(points=np.array((path[i][0],path[i][1],0.05)),pointsize=3.0,colors=green))

                self.solution = path
            
            self.updateKeys()
            self.printPath()

        return self.solution

    def improveSolution(self):

        while len(self.heap) > 0:
            
            # Retrieve node with maxE from open set
            currObj = heapq.heappop(self.heap)

            idx = self.inVisited(currObj[1].coord)
            if idx is None:
                self.visitedSet.append(currObj[1])

            # First part of tuple holds -E value because priority queue sorts via smallest
            # Determines the current suboptimality
            if (currObj[0] > -self.E):
                self.E = -currObj[0]
                print "E: %.2f" % self.E

            if self.isGoal(currObj[1]):
                self.goal.parent = currObj[1].parent
                self.pathFound = True
                self.G = currObj[1].g
                print "G: %.2f" % self.G
                print "Solution Time: %.2f" % (time.clock() - self.startTime)
                return self.constructPath(currObj[1])

            # For debugging
            #d_gs = self.inspectGValues()
            self.logFile.write("Expanding from node: " + np.array_str(currObj[1].coord) + "\n")

            successors = self.getSuccessors(currObj[1])
            #d_coords = self.inspectCoords() 
            for node in successors:

                # Check if current node can be updated with a shorter cost to get here
                moveCost = self.costToNode(currObj[1], node)
                if (currObj[1].g + moveCost < node.g):
                    node.g = currObj[1].g + moveCost
                    node.parent = currObj[1]

                    if node.getF() < self.G:
                        idx = self.inHeap(node)
                        
                        if idx is not None:
                            # Remove the node in the list to update E score
                            self.heap.pop(idx)
                            heapq.heapify(self.heap)

                        # Update / insert node into open priority queue
                        heapq.heappush(self.heap, (node.getE(self.G), node))

        if self.pathFound == True:
           return self.constructPath(self.goal)
        else:
            return []

    def getSuccessors(self, node):
        successors = []

        if (self.connectedness == 4):
            moves = [[0, 1, 0,-1, 0, 0],[1, 0, -1, 0, 0, 0],[0, 0, 0, 0, 1, -1]]
            
            for i in range(0,len(moves[0])):
                location = np.array([node.coord[0]+self.steps[0]*moves[0][i], \
                    node.coord[1]+self.steps[1]*moves[1][i], \
                    (node.coord[2]+self.steps[2]*moves[2][i])%(2*math.pi)])

                idx = self.inVisited(location)
                if idx is not None:
                    successors.append(self.visitedSet[idx])
                    continue

                self.robot.SetActiveDOFValues(location)
                if (self.env.CheckCollision(self.robot) == False):
                    s        = self.Node(coord=location, g=G_INIT_VAL, h=0)
                    s.parent = node
                    s.h      = self.costToGoal(s)
                    successors.append(s)

                    blue = np.array([[0,0,1]])
                    self.handles.append(self.env.plot3(points=np.array((location[0],location[1],0.05)),pointsize=3.0,colors=blue))

                else:
                    red = np.array([[1,0,0]])
                    self.handles.append(self.env.plot3(points=np.array((location[0],location[1],0.05)),pointsize=3.0,colors=red))


        if (self.connectedness == 8):
            for i in xrange(-1,2):
                for j in xrange(-1,2):
                    for k in xrange(-1,2):
                        if i != 0 or j != 0 or k!= 0:
                            location = np.array([node.coord[0]+i*self.steps[0], \
                                node.coord[1]+j*self.steps[1], \
                                (node.coord[2]+k*self.steps[2])%(2*math.pi)])

                            idx = self.inVisited(location)
                            if idx is not None:
                                successors.append(self.visitedSet[idx])
                                continue
                            
                            self.robot.SetActiveDOFValues(location)
                            if (self.env.CheckCollision(self.robot) == False):
                                s        = self.Node(coord=location, g=G_INIT_VAL, h=0)
                                s.parent = node
                                s.h      = self.costToGoal(s)
                                successors.append(s)

                                blue = np.array([[0,0,1]])
                                self.handles.append(self.env.plot3(points=np.array((location[0],location[1],0.05)),pointsize=3.0,colors=blue))

                            else:
                                red = np.array([[1,0,0]])
                                self.handles.append(self.env.plot3(points=np.array((location[0],location[1],0.05)),pointsize=3.0,colors=red))

        return successors

    def inHeap(self, node):
        for i in range(0, len(self.heap)):
            if self.heap[i][1] == node:
                return i
        return None

    def inVisited(self, coord):
        node = self.Node(coord, 0, 0)
        for i in range(0, len(self.visitedSet)):
            if self.visitedSet[i] == node:
                return i
        return None

    def constructPath(self, node):
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

    def isGoal(self, node):
        distBound = math.sqrt(self.steps[0]**2 + self.steps[1]**2)
        withinDist = math.sqrt((node.coord[0]-self.goal.coord[0])**2 + \
                        (node.coord[1]-self.goal.coord[1])**2) <= distBound
        withinAng = almostEqual(self.goal.coord[2], node.coord[2], EPSILON)
        return withinDist and withinAng

    def updateKeys(self):
        f = open("updateKeys.txt", "w")
        f.write("==== Key Values ====\n")
        for i in xrange(0, len(self.heap)):
            data = self.heap[i][1].coord.tolist()
            data.append(self.heap[i][1].g)
            data.append(self.heap[i][1].h)
            f.write("%.2f "*len(data) % tuple(data) + "\n")
        f.close()

        newQueue = []
        while len(self.heap) > 0:
            heapObj = self.heap.pop()

            # Path is alreay longer than current solution
            if (heapObj[1].getF() >= self.G):
                continue
            else:
                newHeapObj = (heapObj[1].getE(self.G), heapObj[1])
                heapq.heappush(newQueue, newHeapObj)

        self.heap = newQueue

    def inspectGValues(self):
        gs = []
        for i in xrange(0,len(self.heap)):
            gs.append(self.heap[i][1].g)

        return gs

    def inspectCoords(self):
        coords = []
        for i in xrange(0,len(self.heap)):
            coords.append(self.heap[i][1].coord.tolist())

        return coords

    def printPath(self):
        f = open("path.txt","w")
        f.write("==== Printing Path ====\n")
        for i in xrange(0,len(self.metaPath)):
            f.write("%.2f "*len(self.metaPath[i]) % self.metaPath[i] + "\n")
                
        f.write("==== Printing Path ====\n")
        f.close()


def almostEqual(val1, val2, tolerance):
    return abs(val1 - val2) < tolerance
        
