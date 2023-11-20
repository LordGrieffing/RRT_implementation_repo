#!/usr/bin/python3

# -- implement RRT algorithm on a image

import cv2
import numpy as np
import networkx as nx
import random as rd
import math
from scipy.optimize import fsolve
import time


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

# -- shortest neighbor check
def shortestNeighbor(currentNode, neighborhood, tree, img):
    
    validPathPossible = False
    bestNeighbor = 0
    shortestTrip = 10000

    for i in range(len(neighborhood)):

        line = []
        line = bresenham_line(int(currentNode[0]), int(currentNode[1]), tree.nodes[neighborhood[i]]['x'], tree.nodes[neighborhood[i]]['y'])
        lineValid = True

        for j in range(len(line)):
            if np.all(img[line[j][0], line[j][1]] == 0):
                lineValid = False
                break

        if lineValid:
            currentDist = math.dist([tree.nodes[neighborhood[i]]['x'], tree.nodes[neighborhood[i]]['y']], [int(currentNode[0]), int(currentNode[1])])
            currentShortPath = currentDist + tree.nodes[neighborhood[i]]['path_to_start']
        
            if currentShortPath <= shortestTrip:
                shortestTrip = currentShortPath
                bestNeighbor = neighborhood[i]
                validPathPossible = True 

    return bestNeighbor, validPathPossible

def neighborhoodAdjustment(tree, currentNode, neighborhood):
    
    for i in range(len(neighborhood)):
        tempdist = math.dist([tree.nodes[currentNode]['x'], tree.nodes[currentNode]['y']], [tree.nodes[neighborhood[i]]['x'], tree.nodes[neighborhood[i]]['y']])
        temp_path_to_start = tempdist + tree.nodes[currentNode]['path_to_start']

        if temp_path_to_start < tree.nodes[neighborhood[i]]['path_to_start']:
            line = []
            line = bresenham_line(tree.nodes[currentNode]['x'], tree.nodes[currentNode]['y'], tree.nodes[neighborhood[i]]['x'], tree.nodes[neighborhood[i]]['y'])
            lineValid = True

            for j in range(len(line)):
                if np.all(img[line[j][0], line[j][1]] == 0):
                    #print("bad line")
                    lineValid = False
                    break

            if lineValid:
                #print("current parent: ", tree.nodes[neighborhood[i]]['parent'])
                tree.nodes[neighborhood[i]]['parent'] = currentNode
                tree.nodes[neighborhood[i]]['path_to_start'] = temp_path_to_start


# -- This is the algroithm, use it to search for a motion plan
def rrt_algorithm(img, start, end, tree):

    # Initialize start
    tree.add_node(1)
    tree.nodes[1]['x'] = start[0]
    tree.nodes[1]['y'] = start[1]
    tree.nodes[1]['parent'] = 0
    tree.nodes[1]['path_to_start'] = 0
    img[start[0], start[1]] = [0,0,255]

    # Inflate goal zone
    goalzone_x = [(end[0] - 10), (end[0] + 10)]
    goalzone_y = [(end[1] - 10), (end[1] + 10)]

    #Draw the goal zone
    #cv2.line(img, ((end[0] - 10), (end[1] + 10)), ((end[0] + 10), (end[1] + 10)), (255, 0, 0), 1)
    #cv2.line(img, ((end[0] - 10), (end[1] - 10)), ((end[0] + 10), (end[1] - 10)), (255, 0, 0), 1)
    #cv2.line(img, ((end[0] - 10), (end[1] + 10)), ((end[0] - 10), (end[1] - 10)), (255, 0, 0), 1)
    #cv2.line(img, ((end[0] + 10), (end[1] - 10)), ((end[0] + 10), (end[1] + 10)), (255, 0, 0), 1)

    #cv2.imshow('My Image',img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    # Initizalize some variables we will be using
    notAtGoal = True
    goalNode = 0
    notAtRoot = True
    path = []
    bestNeighbor = 0
    neighborhood = []

    # Generate new nodes until one is in the goalzone
    while notAtGoal:
        
        # Generate a sample
        sample = generateSample(imgHeight, imgWidth)

        # Find the closest node to the sample
        closestNode = findClosest(tree, sample)
        # If the sample happens to be the same point as the node generate a new node.
        # Yes the possiblilty exist that the node generated might still over lap with the point, but I am willing to live with those odds
        #print(closestNode)
        if tree.nodes[closestNode]['x'] == sample[0] and tree.nodes[closestNode]['y'] == sample[1]:
            sample = generateSample(imgHeight, imgWidth)

        # Generate a new node
        newNodeID = len(tree) + 1
        validNode = False

        while not validNode:
            newNodeCoords = propogate(tree, closestNode, sample)
            color = img[int(newNodeCoords[0]), int(newNodeCoords[1])]

            if not np.all(color == 0):
                
                # Initialize neighborhood
                neighborhood = []

                # Build the node's neighborhood
                for i in range(len(tree.nodes)):
                    tempDistance = math.dist([tree.nodes[i + 1]['x'], tree.nodes[i + 1]['y']], [int(newNodeCoords[0]), int(newNodeCoords[1])])
                    if tempDistance <= neighbors:
                        neighborhood.append(i + 1)

                # Check which neighbor gives the shortest path to the start
                bestNeighbor, lineValid = shortestNeighbor(newNodeCoords, neighborhood, tree, img)
                
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

        #print("we made something valid")
        # Add new node to graph
        tree.add_node(newNodeID)
        tree.nodes[newNodeID]['x'] = int(newNodeCoords[0])
        tree.nodes[newNodeID]['y'] = int(newNodeCoords[1])
        tree.nodes[newNodeID]['parent'] = bestNeighbor
        tree.nodes[newNodeID]['path_to_start'] = tree.nodes[bestNeighbor]['path_to_start'] + math.dist([tree.nodes[bestNeighbor]['x'], tree.nodes[bestNeighbor]['y']], [int(newNodeCoords[0]), int(newNodeCoords[1])])
        #print("A new node was added")
        #tree.add_edge(closestNode, newNodeID)

        # Check if this new node makes paths shorter for it's neighbors
        neighborhoodAdjustment(tree, newNodeID, neighborhood)
        print("new node: ", newNodeID)
        print("coords: ", tree.nodes[newNodeID]['x'], " ", tree.nodes[newNodeID]['y'])

        # Check if newest node is in goal zone
        if between(goalzone_x, goalzone_y,  [tree.nodes[newNodeID]['x'],tree.nodes[newNodeID]['y']] ):
            notAtGoal = False
            goalNode = newNodeID
            print("========================")
            print("goal node ID: ", goalNode)
            print("goal node coords: ", [tree.nodes[newNodeID]['x'],tree.nodes[newNodeID]['y']])
            print("========================")

    return tree, goalNode




if __name__ == "__main__":
    # -- import an image and convert it to a binary image
    img = cv2.imread('map_sequence_1.png')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    imgHeight, imgWidth, channels = img.shape
                                                               

    # -- initialize the start and end points
    start = [300, 200]
    end = [240, 300]
    #end = [200, 350]

    # -- initialize the tree
    tree = nx.Graph()

    # -- Define step size
    stepSize = 15

    # -- Define neighborhood Length
    neighbors = 50

    # -- Run algorithm
    tree, goalNode = rrt_algorithm(img, start, end, tree)

    # -- Draw graph
    '''
    for i in range(len(tree.nodes)):
        node = i + 1
        # Draw node
        img[tree.nodes[node]['x'], tree.nodes[node]['y']] = [0, 0, 255]

        # Draw line
        if tree.nodes[node]['parent'] == 0:
            pass
        else:
            parent = tree.nodes[node]['parent']
            cv2.line(img, (tree.nodes[node]['y'], tree.nodes[node]['x']), (tree.nodes[parent]['y'], tree.nodes[parent]['x']), (255, 0, 0), 1)

    '''

    #cv2.imshow('My Image',img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    # Attempt to display best path
    notRoot = True
    bestpath = []
    bestpath.append(goalNode)
    current_node = goalNode

    while notRoot:
        img[tree.nodes[current_node]['x'], tree.nodes[current_node]['y']] = [0, 0, 255]
        #img[tree.nodes[current_node]['y'], tree.nodes[current_node]['x']] = [0, 0, 255]

        parent = tree.nodes[current_node]['parent']

        if parent != 0:
            cv2.line(img, (tree.nodes[parent]['y'], tree.nodes[parent]['x']), (tree.nodes[current_node]['y'], tree.nodes[current_node]['x']), (0, 0, 255), 5)
            current_node = parent
            bestpath.append(parent)
        else:
            notRoot = False

    cv2.imwrite('/home/jacob/rrt_implementation/RRT_implementation_repo/src/map_sequence_1_traced.png', img)
    cv2.imshow('My Image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()