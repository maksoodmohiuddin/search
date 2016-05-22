# Graph Search
# Implementation variety of graph search algorithms

from __future__ import division
import random
import matplotlib.pyplot as plt
import pickle
import sys
sys.path.insert(1, 'lib')
import networkx
import math
import heapq
from collections import OrderedDict

# Romania map data from Russell and Norvig AI book, Chapter 3."""
romania = pickle.load(open('romania_graph.pickle', 'rb'))

# PQ (Using heapq as base)
class PriorityQueue():
    # Implementation of a priority queue to store nodes during search."""

    def __init__(self):
        self.queue = []
        self.current = 0    

    def next(self):
        if self.current >=len(self.queue):
            self.current
            raise StopIteration
    
        out = self.queue[self.current]
        self.current += 1

        return out

    def pop(self):
        while self.queue:
            node = heapq.heappop(self.queue)
            return node

    def remove(self, nodeId):
        raise NotImplementedError

    def __iter__(self):
        return self

    def __str__(self):
        return 'PQ:[%s]'%(', '.join([str(i) for i in self.queue]))

    def append(self, node):
        heapq.heappush(self.queue, node)

    def __contains__(self, key):
        self.current = 0
        return key in [n for v,n in self.queue]

    def __eq__(self, other):
        return self == other

    def size(self):
        return len(self.queue)
    
    def clear(self):
        self.queue = []
        
    def top(self):
        return self.queue[0]

    __next__ = next

# test PQ
#from search_tests import priority_queue_tests
#priority_queue_tests(PriorityQueue)


# BFS
def breadth_first_search(graph, start, goal):
    # Run a breadth-first search from start to goal and return the path."""
    if start == goal:
        return []

    frontier = []
    frontier.append([start])

    while frontier:
        path = frontier.pop(0)
        lastnode = path[-1]

        for child in graph[lastnode]:
            newpath = list(path)
            newpath.append(child)

            # check if a path is found
            if child == goal:
                return newpath

            if child not in graph.get_explored_nodes():
                if newpath not in frontier:
                    frontier.append(newpath)
    return []


# Utility function to help visually debug  code.
# graph should be a networkx graph, node_positions should be a dictionary mapping nodes to x,y coordinates
def draw_graph(graph, node_positions={}, start=None, goal=None, path=[]):

    explored = list(graph.get_explored_nodes())
    
    labels ={}
    for node in graph:
        labels[node]=node
        
    if not node_positions:
        node_positions = networkx.spring_layout(graph)

    networkx.draw_networkx_nodes(graph, node_positions)
    networkx.draw_networkx_edges(graph, node_positions, style='dashed')
    networkx.draw_networkx_labels(graph,node_positions, labels)
    
    networkx.draw_networkx_nodes(graph, node_positions, nodelist=explored, node_color='g') 

    if path:
        edges = [(path[i], path[i+1]) for i in range(0, len(path)-1)]
        networkx.draw_networkx_edges(graph, node_positions, edgelist=edges, edge_color='b')
   
    if start:
        networkx.draw_networkx_nodes(graph, node_positions, nodelist=[start], node_color='b')
    
    if goal:
        networkx.draw_networkx_nodes(graph, node_positions, nodelist=[goal], node_color='y')

    plt.plot()
    plt.show()

# Testing and visualizing BFS
#start = 'a'
#goal = 'u'
#node_positions = {n: romania.node[n]['pos'] for n in romania.node.keys()}
#romania.reset_search()
#path = breadth_first_search(romania, start, goal)
#draw_graph(romania, node_positions=node_positions, start=start, goal=goal, path=path)

#from search_tests import bfs_tests
#bfs_tests(breadth_first_search)

# Uniform-cost search
def uniform_cost_search(graph, start, goal):
    # Run uniform-cost search from start to goal and return the path

    if start == goal:
        return []

    node = 0, [start]

    frontier = PriorityQueue()
    frontier.append(node)

    while frontier.size() > 0:
        node = frontier.pop()
        pathcost = node[0]
        path = node[1]
        lastnodeid = path[len(path) - 1]

        # check if a path is found
        if lastnodeid == goal:
            return path

        for child in graph[lastnodeid]:
            newpath = list(path)
            newpath.append(child)

            if child not in graph.get_explored_nodes():
                cost = graph[lastnodeid][child]['weight']
                newpathcost = pathcost + cost
                if newpath not in frontier:
                    node = newpathcost, newpath
                    frontier.append(node)
    return []


# Test UCS
#from search_tests import ucs_tests
#ucs_tests(uniform_cost_search)


# A Star search (based on Depth-first search)

def null_heuristic(graph, v, goal):
    # Return 0 for all nodes
    return 0

def heuristic_euclid(graph, v, goal):
    # Return the Euclidean distance from node v to the goal."""
    vPos = graph.node[v]['pos']
    goalPos = graph.node[goal]['pos']
    x1 = vPos[0]
    y1 = vPos[1]

    x2 = goalPos[0]
    y2 = goalPos[1]

    euclidean_distance = math.hypot(x2-x1, y2-y1)
    return euclidean_distance


def a_star(graph, start, goal, heuristic):
    # Run A* search from the start to goal using the specified heuristic function, and return the final path and the nodes explored.

    if start == goal:
        return []

    # tuple: heuristiccost, totalpathcost, list of nodes
    node = 0, 0, [start]

    frontier = PriorityQueue()
    frontier.append(node)

    while frontier.size() > 0:
        node = frontier.pop()
        pathcost = node[1]
        path = node[2]
        lastnodeid = path[len(path) - 1]

        # check if a path is found
        if lastnodeid == goal:
            return path

        for child in graph[lastnodeid]:
            newpath = list(path)
            newpath.append(child)

            if child not in graph.get_explored_nodes():
                cost = graph[lastnodeid][child]['weight']
                newpathcost = pathcost + cost
                heuristic = int(heuristic_euclid(graph, child, goal))
                heuristiccost = newpathcost + heuristic
                #if newpath not in frontier:
                node = heuristiccost, newpathcost, newpath
                frontier.append(node)
    return []


# Test A Star Search
#from search_tests import a_star_tests
#a_star_tests(a_star, null_heuristic, heuristic_euclid)

