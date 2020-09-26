#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  4 13:03:43 2020

@author: prithvi
"""

import numpy as np
import queue


class Graph(object):
    
    def __init__(self, graph=None):
        """
        Parameters
        ----------
        graph -- dictionary with nodes as the keys and the neighbour nodes as the values of the keys

        Returns
        -------
        None.
        """
        if graph is None:
            graph = {}
        self.__graph_dict = graph
        
    def __generate_edges(self):
        """
        Generates the edges from the input graph dictionary.
        Edges are in the tuples: ['a', 'd'] represents an edge from a to d and 
        ['a'] represents an wdge from a to a itself.

        Returns
        -------
        edges -- list of edges
        """
        edges = []
        for node in self.__graph_dict.keys():
            for neighbor in self.__graph_dict[node]:
                if (neighbor == node):
                    if ([neighbor] not in edges):
                        edges.append([neighbor])
                else:
                    if ([node, neighbor] not in edges):
                        edges.append([node, neighbor])
        return edges
    
    def edges(self):
        """
        Returns
        -------
        edges -- list of the directed edges in the graph

        """
        return self.__generate_edges()
    
    def nodes(self):
        """
        Returns
        -------
        list of nodes in the graph
        """
        return list(self.__graph_dict.keys())
    
    def add_node(self, node):
        """
        This function helps add a node to the graph.
        It checks if the node is already present in the graph.
        If it is not present then the node is added to the graph dictionary.
        
        Parameters
        ----------
        node -- char
        
        Returns
        -------
        None.
        """
        if node not in self.__graph_dict.keys():
            self.__graph_dict[node] = []
            
    def add_edge(self, edge):
        """
        This function adds an edge to the graph.
        It checks if the edge is already present in the graph.
        If not present, it will add the edge
        If the edge has a node that is not present in the graphs, then that node will be added to the graph as well

        Parameters
        ----------
        edge -- list of the form [node1, node2] where node1 is the starting node 
                and node2 is the ending node 
                or
                list of form [node1] if the edges loops on the same node

        Returns
        -------
        None.
        """
        if (len(edge) == 2):
            node1, node2 = edge
            if node1 in self.__graph_dict.keys():
                self.__graph_dict[node1].append(node2)
            else:
                self.__graph_dict[node1] = [node2]
            
            if node2 not in self.__graph_dict.keys():
                self.add_node(node2)
                
        elif (len(edge) == 1):
            node1 = edge[0]
            if node1 in self.__graph_dict.keys():
                self.__graph_dict[node1].append(node1)
            else:
                self.__graph_dict[node1] = [node1]
                
    def get_adjacency(self):
        """
        Thi function generates and returns the adjacency matrix of the graph

        Returns
        -------
        adj_mat -- matrix
        """
        n = len(self.nodes())
        nodes = self.nodes()
        edges = self.edges()
        adj_mat = np.zeros((n,n))
        for edge in edges:
            if (len(edge) == 2):
                idx_1 = nodes.index(edge[0])
                idx_2 = nodes.index(edge[1])
                adj_mat[idx_2][idx_1] = 1
            elif (len(edge) == 1):
                idx_1 = nodes.index(edge[0])
                adj_mat[idx_1][idx_1] = 1
                
        return adj_mat
    
    def DFS_check(self, node, discovered, check):
        """
        This function runs the depth first search on the graph, starting from the input node.
        It maintains a check list which keeps a track of the nodes already been expanded but y
        If while depth first search, we come accross a node that has been discovered or expanded,
        we declare that the graph is cyclic

        Parameters
        ----------
        node -- char representing the node
        discovered -- list of the nodes that have been 
            DESCRIPTION.
        stack : TYPE
            DESCRIPTION.

        Returns
        -------
        bool
            DESCRIPTION.

        """
        discovered.append(node)
        check.append(node)
        
        for neighbour in self.__graph_dict[node]:
            if (neighbour not in discovered):
                if (self.DFS_check(neighbour, discovered, check) == True):
                    return True
            elif (neighbour in check):
                return True
        check.remove(node)
        return False   
    
    
    def is_cyclic(self):
        """
        

        Returns
        -------
        bool
            DESCRIPTION.

        """
        discovered = []
        check = []
        for node in self.nodes():
            if (node not in discovered):
                if (self.DFS_check(node, discovered, check) == True):
                    return True
        return False
    
    def min_dist(self, n1, n2):
        """
        This function uses Breadth First Search (BFS) on the graph to find the
        shortest distance between 2 nodes

        Parameters
        ----------
        n1 : start node
            
        n2 : end node
            

        Returns
        -------
        Minimum distance between n1 and n2

        """
        # visited[n] for keeping track  
        # of visited node in BFS  
        visited = [0] * len(self.nodes())
        
        # Initialize distances as 0  
        distance = [0] * len(self.nodes())
        Q = queue.Queue()
        distance[self.nodes().index(n1)] = 0
        
        Q.put(n1)
        visited[self.nodes().index(n1)] = True
        while (not Q.empty()):
            x = Q.get()
            
            for neighbor in self.__graph_dict[x]:
                if (visited[self.nodes().index(neighbor)]):
                    continue
                
                distance[self.nodes().index(neighbor)] = distance[self.nodes().index(x)] + 1
                Q.put(neighbor)
                visited[self.nodes().index(neighbor)] = 1
        
        return distance[self.nodes().index(n2)]
    
    def get_closeness_centrality(self):
        """
        This function uses the min_dist function to calculate the minimum distance
        between nodes and returns a dictionary of the closeness centrality 
        of all the nodes

        Returns
        -------
        cl_dict : dictionary
            contains closeness centrality of all nodes

        """
        n = len(self.nodes()) - 1
        cl_dict = {}
        
        for i in self.nodes():
            d_ij = 0
            for j in self.nodes():
                d_ij += self.min_dist(i, j)
                
            cl_dict[i] = n/d_ij
            
        return cl_dict
    
    def summary(self):
        """
        This function generates a summary of the graph 
        
        Returns
        -------
        res -- string containing the description of the graph
        """
        res = "vertices: "
        for k in self.__graph_dict:
            res += str(k) + " "
        res += "\nedges: "
        for edge in self.__generate_edges():
            res += str(edge) + " "
        res += "\nAdjacency matrix: \n"
        res += str(self.get_adjacency())
        
        return res
