#!/usr/bin/env python

import rospy
import numpy as np
import scipy as sci
from scipy import optimize
import math
import csv
#from anytree import Node, RenderTree
import Queue
import sys
from tf2_msgs.msg import TFMessage

#Class for Queue and Tree in BFS
class pathNode:
    def __init__(self, priority, x, y, dist, parent, pushDir):
        self.priority = priority
        self.x = x
        self.y = y
        self.dist = dist
        self.parent = parent
        self.pushDir = pushDir
    def printNode(self):
        print str(self.dist) + "   " + str(self.x) + " " \
            + str(self.y) + "   " + str(self.pushDir)

def getModel():
    #Load the data from csv file
    train_data_X = np.loadtxt('../data/legoData.csv', delimiter=',', usecols=(1,2,5));
    train_data_Y = np.loadtxt('../data/legoData.csv', delimiter=',', usecols=(3,4));
    
    #Add Bias term and basis functions
    train_data_X = np.c_[np.ones((train_data_X.shape[0], 1)), train_data_X] 
    train_data_X = np.c_[train_data_X, np.cos(train_data_X[:, 3])]
    train_data_X = np.c_[train_data_X, np.sin(train_data_X[:, 3])]

    #Split into training and testing data
    test_data_X = train_data_X[::5]
    test_data_Y = train_data_Y[::5]
    train_data_X = np.delete(train_data_X, slice(None, None, 5), axis = 0)
    train_data_Y = np.delete(train_data_Y, slice(None, None, 5), axis = 0)

    return np.linalg.lstsq(train_data_X, train_data_Y)

def func(x, C, Y, D):
    out = [C[0][0] + C[1][0] * x[0] + C[2][0] * x[1] 
         + C[3][0] * x[2] + C[4][0] * x[3] + C[5][0] * x[4] - Y[0]]
    out.append(C[0][1] + C[1][1] * x[0] + C[2][1] * x[1] 
         + C[3][1] * x[2] + C[4][1] * x[3] + C[5][1] * x[4] - Y[1])
    out.append(x[3] - math.cos(x[2]))
    out.append(x[4] - math.sin(x[2]))
    out.append(x[2] - D)
    return out

def distance(x0, x1, y0, y1):
    return math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))

def atGoal(curr, start):
    return distance(curr.x, start[0], curr.y, start[1]) < 0.03

def validPos(x, y, curr, start):
    d1 = distance(curr.x, start[0], curr.y, start[1])
    d2 = distance(x, start[0], y, start[1]) 
    return x < 0.6 and x > -0.6 and y < 0.6 and y > -0.6 and d2 < d1

def printPath(curr):
    while curr != None:
        curr.printNode()
        curr = curr.parent


def callback(data):
    print data.transforms[0].header.frame_id

def makeMove(model, goal):
    timeout = 0
    #start = wait for message /yellow_lego
    #Queue for BFS
    q = Queue.PriorityQueue()
    q.put(pathNode(distance(goal[0], start[0], goal[1], start[1]), goal[0], goal[1], 0, None, None))
    while timeout == 0:
        curr = q.get()
        if curr.dist > 20:
            timeout = 1
        if atGoal(curr, start):
            printPath(curr)
            # push(startx, starty, 0.6, curr.pushDir, 0.10)
            break;
        for i in range(0, 32):
            pushDir = i * math.pi / 16.0
            n = sci.optimize.fsolve(func, [0,0,0,0,0], args=(model[0], [curr.x, curr.y], pushDir))
            if validPos(n[0], n[1], curr, start):
                q.put(pathNode(distance(n[0], start[0], n[1], start[1]) + curr.dist + 1, 
                                n[0], n[1], curr.dist + 1, curr, pushDir))
    #newPose = wait for message /yellow_lego
    d = distance(newPosex, goalx, newPosey, goaly)
    return d < 0.02

model = getModel()
moves = 0
goal = [float(sys.argv[1]), float(sys.argv[2])]
goalReached = False
while goalReached == False:
    goalReached = makeMove(model, goal)
    moves = moves + 1
print "Reached goal in " + str(moves) + " pushes"

