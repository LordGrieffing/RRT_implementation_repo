#!/usr/bin/env python3


import sys
import numpy as np
import sys
import networkx as nx
from scipy import ndimage
from skimage.morphology import skeletonize
import cv2
import math
import time


def erode_image(map_array, filter_size = 9):
    """ Erodes the image to reduce the chances of robot colliding with the wall
    each pixel is 0.05 meter. The robot is 30 cm wide, that is 0.3 m. Half is
    0.15 m. If we increase the walls by 20 cm on either side, the kernel should
    be 40 cm wide. 0.4 / 0.05 = 8
    """
    kernel = np.ones((filter_size,filter_size), np.uint8)
    eroded_img = cv2.erode(map_array, kernel, iterations = 1)
    return eroded_img

def data_skeleton(data_binary):
    skeleton = skeletonize(data_binary)
    return skeleton

def run_skeleton_input(ske):
    # -- convert skeleton to binary skeleton
    if len(np.unique(ske)) > 2:
        # -- skeleton is not binary
        ske[np.where(ske > 0)] = 1
        ske[np.where(ske < 1)] = 0
    
    # -- generate the graph
    #node_img = mark_node(ske)
    graph = build_sknw(ske)
    
    # -- return the graph
    return graph

def build_sknw(ske, multi=False):
    buf = buffer(ske)
    nbs = neighbors(buf.shape)
    acc = np.cumprod((1,)+buf.shape[::-1][:-1])[::-1]
    mark(buf, nbs)
    pts = np.array(np.where(buf.ravel()==2))[0]
    nodes, edges = parse_struc(buf, pts, nbs, acc)
    return build_graph(nodes, edges, multi)

def buffer(ske):
    buf = np.zeros(tuple(np.array(ske.shape)+2), dtype=np.uint16)
    buf[tuple([slice(1,-1)]*buf.ndim)] = ske
    return buf

def neighbors(shape):
    dim = len(shape)
    block = np.ones([3]*dim)
    block[tuple([1]*dim)] = 0
    idx = np.where(block>0)
    idx = np.array(idx, dtype=np.uint8).T
    idx = np.array(idx-[1]*dim)
    acc = np.cumprod((1,)+shape[::-1][:-1])
    return np.dot(idx, acc[::-1])

def mark(img, nbs): # mark the array use (0, 1, 2)
    img = img.ravel()
    for p in range(len(img)):
        if img[p]==0:continue
        s = 0
        for dp in nbs:
            if img[p+dp]!=0:s+=1
        if s==2:img[p]=1
        else:img[p]=2

def parse_struc(img, pts, nbs, acc):
    img = img.ravel()
    buf = np.zeros(131072, dtype=np.int64)
    #sys.exit(0)
    num = 10
    nodes = []
    for p in pts:
        if img[p] == 2:
            nds = fill(img, p, num, nbs, acc, buf)
            num += 1
            nodes.append(nds)
    edges = []
    for p in pts:
        for dp in nbs:
            if img[p+dp]==1:
                edge = trace(img, p+dp, nbs, acc, buf)
                edges.append(edge)
    return nodes, edges

def build_graph(nodes, edges, multi=False):
    graph = nx.MultiGraph() if multi else nx.Graph()
    for i in range(len(nodes)):
        graph.add_node(i, pts=nodes[i], o=nodes[i].mean(axis=0))
    for s,e,pts in edges:
        l = np.linalg.norm(pts[1:]-pts[:-1], axis=1).sum()
        graph.add_edge(s,e, pts=pts, weight=l)
    return graph

def fill(img, p, num, nbs, acc, buf):
    back = img[p]
    img[p] = num
    buf[0] = p
    cur = 0; s = 1
    
    while True:
        p = buf[cur]
        for dp in nbs:
            cp = p+dp
            if img[cp]==back:
                img[cp] = num
                buf[s] = cp
                s+=1
        cur += 1
        if cur==s:break
    return idx2rc(buf[:s], acc)

def trace(img, p, nbs, acc, buf):
    c1 = 0; c2 = 0
    newp = 0
    cur = 1
    while True:
        buf[cur] = p
        img[p] = 0
        cur += 1
        for dp in nbs:
            cp = p + dp
            if img[cp] >= 10:
                if c1==0:
                    c1 = img[cp]
                    buf[0] = cp
                else:
                    c2 = img[cp]
                    buf[cur] = cp
            if img[cp] == 1:
                newp = cp
        p = newp
        if c2!=0:break
    return (c1-10, c2-10, idx2rc(buf[:cur+1], acc))

def idx2rc(idx, acc):
    rst = np.zeros((len(idx), len(acc)), dtype=np.int16)
    for i in range(len(idx)):
        for j in range(len(acc)):
            rst[i,j] = idx[i]//acc[j]
            idx[i] -= rst[i,j]*acc[j]
    rst -= 1
    return rst

def convert_to_binary(map_array):
    map_binary = np.zeros(np.shape(map_array), dtype=bool)
    map_binary[np.where(map_array == 255)] = True
    return map_binary

def profileEnd(start, path):
    end = time.time()
    total = end - start
    with open(path, 'a') as f:
        f.write('\n')
        f.write(str(total))








if __name__ == "__main__":
    img = cv2.imread('map_sequence_5.png')

    '''
    for i in range(100):
        #start = time.time()
        map_data = erode_image(img)
        map_binary = convert_to_binary(map_data)

        skeleton = data_skeleton(map_binary)
        #cv2.imshow('My Image', skeleton)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

        graph = run_skeleton_input(skeleton)
        #profileEnd(start, "Skeleton_profile_data/Skeleton_profile_sequence5_n100.txt")

    '''
    map_data = erode_image(img)
    map_binary = convert_to_binary(map_data)

    skeleton = data_skeleton(map_binary)
    #cv2.imshow('My Image', skeleton)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    graph = run_skeleton_input(skeleton)


    # -- print the graph
    # -- Display graph
    nodeList = list(graph.nodes)
    edgeList = list(graph.edges)

    # -- add nodes
    for j in range(len(nodeList)):
        nodeCoords = (graph.nodes[nodeList[j]]['pts'][0][1], graph.nodes[nodeList[j]]['pts'][0][0])
        cv2.circle(img, nodeCoords, 4, (255, 0, 0), -1)

    for j in range(len(edgeList)):
                
        currentEdge = edgeList[j]
        cv2.line(img, (graph.nodes[currentEdge[0]]['pts'][0][1], graph.nodes[currentEdge[0]]['pts'][0][0]), (graph.nodes[currentEdge[1]]['pts'][0][1], graph.nodes[currentEdge[1]]['pts'][0][0]), (0, 0, 255), 1)

    cv2.imshow('My Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #cv2.imwrite("skeleton_map_sequence__eroded_1.png", skeleton)




























































