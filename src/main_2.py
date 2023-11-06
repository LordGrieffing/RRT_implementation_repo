#!/usr/bin/python3

# -- implement RRT algorithm on a image

import cv2
import numpy as np
import networkx as nx
import random as rd
import math
from scipy.optimize import fsolve
import time

# -- set up colors
COLOR_RED = (0,0,255)
COLOR_BLACK = (0,0,0)
COLOR_WHITE = (255, 255, 255)
COLOR_BLUE = (255, 0, 0)

# -- This method generates a random sample in the space
def generateSample(height, width):
    sample_x = rd.randrange(0, height)
    sample_y = rd.randrange(0, width)

    #print([sample_x, sample_y])
    return [sample_x, sample_y]

# -- This method finds which existing node is the closest node to the generated sample
def findClosest(graph, sample):
    
    shortestDist = 1000000
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
    
    # Check if the X value of the previous node and the sample are the same to avoid divide by zero error
    if prevNode[0] == sample[0]:
        if prevNode[1] > sample[1]:
            x_true = prevNode[0]
            y_true = prevNode[1] - stepSize
        else:
            x_true = prevNode[0]
            y_true = prevNode[1] + stepSize

        return [x_true, y_true]
    
    # Find information needed for the line equation
    slope = (prevNode[1] - sample[1]) / (prevNode[0] - sample[0])
    y_intercept = sample[1] - (slope * sample[0])

    # Use formula to find desired point
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
def between(range_x, range_y, num):

    
    if num[1] >= range_x[0] and num[1] <= range_x[1]:
        if num[0] >= range_y[0] and num[0] <= range_y[1]:
            return True
        
    else:
        return False

# -- This method uses Bresenham's line Algorithm to draw an edge
# -- I got this from ChatGPT
def bresenham_line(x1, y1, x2, y2):
    # Setup initial conditions
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    x, y = x1, y1
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1

    # Decision variables for determining the next point
    if dx > dy:
        err = dx / 2
        while x != x2:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2
        while y != y2:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    points.append((x, y))  # Add the final point
    return points


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
    img[start[0], start[1]] = [0,0,255]

    # Inflate goal zone
    goalzone_x = [(end[0] - 10), (end[0] + 10)]
    goalzone_y = [(end[1] - 10), (end[1] + 10)]

    #Draw the goal zone
    cv2.line(img, ((end[0] - 10), (end[1] + 10)), ((end[0] + 10), (end[1] + 10)), COLOR_BLUE, 1)
    cv2.line(img, ((end[0] - 10), (end[1] - 10)), ((end[0] + 10), (end[1] - 10)), COLOR_BLUE, 1)
    cv2.line(img, ((end[0] - 10), (end[1] + 10)), ((end[0] - 10), (end[1] - 10)), COLOR_BLUE, 1)
    cv2.line(img, ((end[0] + 10), (end[1] - 10)), ((end[0] + 10), (end[1] + 10)), COLOR_BLUE, 1)

    # cv2.imshow('My Image',img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Initizalize some variables we will be using
    notAtGoal = True
    goalNode = 0
    notAtRoot = True
    path = []
    #line = []

    # Generate new nodes until one is in the goalzone
    while notAtGoal:
        
        # Generate a sample
        sample = generateSample(imgHeight, imgWidth)
        #img[sample[0], sample[1]] = [0, 0, 255]

        # Find the closest node to the sample
        closestNode = findClosest(tree, sample)
        if tree.nodes[closestNode]['x'] == sample[0] and tree.nodes[closestNode]['y'] == sample[1]:
            sample = generateSample(imgHeight, imgWidth)

        # Generate a new node
        newNodeID = len(tree) + 1
        validNode = False
        
        while not validNode: 

            newNodeCoords = propogate(tree, closestNode, sample)
            color = img[int(newNodeCoords[0]), int(newNodeCoords[1])]
            if not np.all(color == 0):
                line = []
                line = bresenham_line(int(newNodeCoords[0]), int(newNodeCoords[1]), tree.nodes[closestNode]['x'], tree.nodes[closestNode]['y'])
                lineValid = True

                for i in range(len(line)):
                    if np.all(img[line[i][0], line[i][1]] == 0):
                        lineValid = False
                        #print("Invalid line", line[i][0], " ", line[i][1])
                        break
                
                if lineValid:
                    validNode = True

                else: 
                    sample = generateSample(imgHeight, imgWidth)
                    if tree.nodes[closestNode]['x'] == sample[0] and tree.nodes[closestNode]['y'] == sample[1]:
                        sample = generateSample(imgHeight, imgWidth) 
            
             
             
            else:
                sample = generateSample(imgHeight, imgWidth)
                if tree.nodes[closestNode]['x'] == sample[0] and tree.nodes[closestNode]['y'] == sample[1]:
                    sample = generateSample(imgHeight, imgWidth)


        # Add new node to graph
        tree.add_node(newNodeID)
        tree.nodes[newNodeID]['x'] = int(newNodeCoords[0])
        tree.nodes[newNodeID]['y'] = int(newNodeCoords[1])
        tree.nodes[newNodeID]['parent'] = closestNode
        tree.add_edge(closestNode, newNodeID)


        # Draw the node and the new edge
        img[int(newNodeCoords[0]), int(newNodeCoords[1])] = [0, 0, 255]        
        cv2.line(img, (int(newNodeCoords[1]), int(newNodeCoords[0])), 
                 (tree.nodes[closestNode]['y'], tree.nodes[closestNode]['x']), 
                 COLOR_BLUE, 1)


        # Check if newest node is in goal zone
        if between(goalzone_x, goalzone_y,  [tree.nodes[newNodeID]['x'],tree.nodes[newNodeID]['y']] ):
            notAtGoal = False
            goalNode = newNodeID
            print("========================")
            print("goal node ID: ", goalNode)
            print("goal node coords: ", [tree.nodes[newNodeID]['x'],tree.nodes[newNodeID]['y']])
            print("========================")


    # display map
    cv2.imshow('My Image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return goalNode


if __name__ == '__main__':
    # -- set input parameters
    filename = 'maze5.png'
    stepSize = 15 # Define step size

    # -- import an image and convert it to a binary image
    img = cv2.imread('BentSpear_maze.png')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    imgHeight, imgWidth, channels = img.shape

    # -- initialize the start and end points
    start = [60, 10]
    #end = [200, 350]
    end = [485, 485]

    # -- initialize the tree
    tree = nx.Graph()

    # -- run RRT algorithm
    goalNode = rrt_algorithm(img, start, end, tree)

    # attempt to display best path
    notRoot = True
    bestpath = []
    bestpath.append(goalNode)
    current_node = goalNode

    while notRoot:
        img[tree.nodes[current_node]['x'], tree.nodes[current_node]['y']] = [0, 0, 255]

        parent = tree.nodes[current_node]['parent']

        if parent != 0:
            # draw a straight line from parent to current node
            cv2.line(img, (tree.nodes[parent]['y'], tree.nodes[parent]['x']),
                    (tree.nodes[current_node]['y'], tree.nodes[current_node]['x']),
                    COLOR_RED, 3)
            current_node = parent
            bestpath.append(parent)
        else:
            notRoot = False

    # Display the image
    cv2.imshow('My Image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
