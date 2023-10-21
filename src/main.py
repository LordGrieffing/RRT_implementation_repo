#!/usr/bin/python3

# -- implement RRT algorithm on a image

import cv2
import numpy as np
import networkx as nx
import random as rd
import math
from scipy.optimize import fsolve
import time

# -- import an image and convert it to a binary image
img = cv2.imread('maze0.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)


# -- initialize the start and end points
start = [10, 10]
end = [300, 300]

# -- initialize the tree
tree = nx.Graph()

# -- Define step size
stepSize = 5

# -- This method generates a random sample in the space
def generateSample():
    sample_x = rd.randrange(0, 400)
    sample_y = rd.randrange(0, 400)

    #print([sample_x, sample_y])
    return [sample_x, sample_y]

# -- This method finds which existing node is the closest node to the generated sample
def findClosest(graph, sample):
    
    shortestDist = 1000
    closestNode = 0

    for node in graph.nodes:
        node_x = graph.nodes[node]['x']
        node_y = graph.nodes[node]['y']

        tempNode = [node_x, node_y]

        dist = math.dist(sample, tempNode)

        if dist < shortestDist:
            shortestDist = dist
            closestNode = node

    return closestNode

# -- This method generates a new node that is along the line from the closests node and the sample
def propogate(graph, closestNode, sample):

    # I don't feel like calling the node a bunch of times so make an array to work with
    prevNode = [graph.nodes[closestNode]['x'], graph.nodes[closestNode]['y']]

    # Check to see if the sample is close enough to just be added as a node
    if math.dist(prevNode, sample) <= stepSize:
        return sample
    
    # Find information needed for the line equation
    slope = (prevNode[0] - sample[0]) / (prevNode[1] - sample[1])
    y_intercept = sample[1] - (slope * sample[0])

    # Use formula to find two possible points
    if prevNode[0] < sample[0]:
        x_true = prevNode[0] + (stepSize / math.sqrt(1 + slope**2))
        y_true = prevNode[1] + slope * (x_true - prevNode[0]) 

    elif prevNode[0] > sample[0]:
        x_true = prevNode[0] - (stepSize / math.sqrt(1 + slope**2))
        y_true = prevNode[1] + slope * (x_true - prevNode[0]) 

    # Return the point
    return [x_true, y_true]
    
# -- Checks whether a point is on a line or not
def lineCheck(point, slope, y_intercept):
    
    test_y = (slope * point[0]) + y_intercept

    if int(test_y) == int(point[1]):
        return True
    else:
        return False

# -- Checks if a number is between a range of numbers
def between(range, num):

    range_x = int(range[0])
    range_y = int(range[1])
    
    if range_x <= num[0]:
        if range_y >= num[1]:
            return True
        
    else:
        return False

# -- This method uses Bresenham's line Algorithm to draw an edge
def drawTheEdge(start, end):
    
    # Initialize variables we will be using
    x_next = start[0]
    y_next = start[1]
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    pk = (2 * delta_y) - delta_x
    line = []
    endInt = [int(end[0]), int(end[1])]

    #print(x_next)
    #print(y_next)
    #print(endInt)
    # Generate points in the line
    while (x_next != endInt[0]) or (y_next != endInt[1]):

        if x_next == endInt[0]:
        
            if y_next < endInt[1]:

                y_next = y_next + 1

            else:
                y_next = y_next - 1

        elif y_next == endInt[1]:
        
            if x_next < endInt[0]:

                x_next = x_next + 1

            else:
                x_next = x_next - 1

        else:
            if pk > 0:
            
                pkNext = pk + (2 * delta_y) - (2 * delta_x)

                if x_next > endInt[0]:
                    x_next = x_next - 1
                else:
                    x_next = x_next + 1

                if y_next > endInt[1]:
                    y_next = y_next - 1
                else:
                    y_next = y_next + 1

            
            if pk < 0:
                pkNext = pk + (2 * delta_y)

                if x_next > end[0]:
                    x_next = x_next - 1
                else:
                    x_next = x_next + 1
                
        


        line.append([x_next, y_next])

    return line






def test_line_function(start, end, img):
    
    img[start[0], start[1]] = [0, 0, 255]
    img[end[0], end[1]] = [0, 0, 255]
    
    cv2.imshow('My Image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    cv2.line(img, (start[1], start[0]), (end[1], end[0]), (255,0,0), 1)

    cv2.imshow('My Image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    


    




# -- This is the algroithm, use it to search for a motion plan
def rrt_algorithm(img, start, end, tree):

    # Initialize start
    tree.add_node(1)
    tree.nodes[1]['x'] = start[0]
    tree.nodes[1]['y'] = start[1]
    tree.nodes[1]['parent'] = 0
    img[start[0], start[1]] = [0,0,0]

    # Inflate goal zone
    goalzone_x = [(end[0] - 10), (end[0] + 10)]
    goalzone_y = [(end[1] - 10), (end[1] + 10)]

    # Initizalize some variables we will be using
    notAtGoal = True
    goalNode = 0
    notAtRoot = True
    path = []
    #line = []

    # Generate new nodes until one is in the goalzone
    while notAtGoal:
        
        # Generate a sample
        sample = generateSample()

        # Find the closest node to the sample
        closestNode = findClosest(tree, sample)

        # Generate a new node
        newNodeID = len(tree) + 1
        validNode = False
        
        while not validNode:

            newNodeCoords = propogate(tree, closestNode, sample)
            color = img[int(newNodeCoords[0]), int(newNodeCoords[1])]
            time.sleep(1)
            if not np.all(color == 0):
                validNode = True
            
                '''
                line = drawTheEdge([tree.nodes[closestNode]['x'], tree.nodes[closestNode]['y']] , newNodeCoords)
                validLinePoint = True
                i = 0
                while validLinePoint and i == len(line) - 1:
                    color = img[line[i]]
                    if np.any(color == 0):
                        validNode = False
                        validLinePoint = False

                    i = i + 1
                '''

            else:
                sample = generateSample()



        # Add new node to graph
        tree.add_node(newNodeID)
        tree.nodes[newNodeID]['x'] = int(newNodeCoords[0])
        tree.nodes[newNodeID]['y'] = int(newNodeCoords[1])
        tree.nodes[newNodeID]['parent'] = closestNode
        tree.add_edge(closestNode, newNodeID)

        

        # Draw the node and the new edge
        img[int(newNodeCoords[0]), int(newNodeCoords[1])] = [0, 0, 255]
        
        cv2.line(img, (int(newNodeCoords[1]), int(newNodeCoords[0])), (tree.nodes[closestNode]['y'], tree.nodes[closestNode]['x']), (255,0,0), 1)

        cv2.imshow('My Image',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


        # Check if newest node is in goal zone
        if between(goalzone_x, [tree.nodes[newNodeID]['x'],tree.nodes[newNodeID]['y']] ) and between(goalzone_y, [tree.nodes[newNodeID]['x'],tree.nodes[newNodeID]['y']]):
            notAtGoal = False
            goalNode = newNodeID


    # display map
    cv2.imshow('My Image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Find path from start to goal
    currentNode = goalNode
    while notAtRoot:
        path.append(currentNode)

        parentNode = tree.nodes[currentNode]['parent']

        if parentNode == 0:
            notAtRoot = False

        currentNode = parentNode

    return path









































# -- run RRT algorithm
rrt_algorithm(img, start, end, tree)
#test_line_function(start, end, img)

# -- show the result
#cv2.imshow('image', thresh)
#cv2.waitKey(0)









































