#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct  1 14:49:51 2019

@author: sean
"""

import networkx as nx
import matplotlib.pyplot as plt  
from matplotlib.lines import Line2D  
import numpy as np


def plot_graph(graph_file_path,mat_file_path):
    
    G = nx.read_graphml(graph_file_path)
    
    node_data = {} 
    for n,data in G.node.items():
        node_data[n] = data['coords']
    
    edge_data={}
    for n,data in G.edges.items():
        edge_data[n] = data
    
    z=[]
    for n in edge_data:
        z.append(n)
        
    ########################plot graph################################
    fig=plt.figure()
    fig.clf()
    
    x=np.zeros(shape=G.number_of_nodes())
    y=np.zeros(shape=G.number_of_nodes())
    
    i=0
    for k in node_data:
        x[i]=node_data[k].split(",")[0] 
        y[i]=node_data[k].split(",")[1]
        i=i+1
    
    plt.scatter(x, y)
    #plt.axis('scaled')
    
    ax = fig.add_subplot(111)
    
    for i in z:
        pt1=i[0]
        pt2=i[1]
        pt1=node_data[pt1].split(",")[0:2]
        pt2=node_data[pt2].split(",")[0:2]
        x1,y1=pt1
        x2,y2=pt2
        line=Line2D([float(x1),float(x2)],[float(y1),float(y2)],color='k') 
        ax.add_line(line)
    
    data=np.loadtxt(mat_file_path)
    ax=data[:,0]
    ay=data[:,1]
    plt.plot(ax,ay,'r')
    #plt.axis('scaled')
    plt.show()
    plt.savefig('hi.jpg', bbox_inches='tight', pad_inches=0)   
    
        

#main
graph_file_path="graph2.graphml"
mat_file_path="mat2.txt"
plot_graph(graph_file_path,mat_file_path)
  
    
    
    