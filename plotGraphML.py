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
import math


def plot_graph(graph_file_path,mat_file_path):
    
    G = nx.read_graphml(graph_file_path)
    
    node_data = {} 
    for n in G.nodes:
        node_data[n] = G.nodes[n]['coords']
    
    edge_data=list(G.edges)
    
        
    ########################plot graph################################
    fig=plt.figure()
    fig.clf()
    
    xtotal=np.zeros(shape=G.number_of_nodes())
    ytotal=np.zeros(shape=G.number_of_nodes())
    
    ax = fig.add_subplot(111)
    
    i=0
    for k in node_data:
        xt=float(node_data[k].split(",")[0])
        yt=float(node_data[k].split(",")[1])
        at=float(node_data[k].split(",")[2])
        #print(xt)
        #print(xt+0.1*math.cos(at))
        #print(yt)
        #print(yt+0.1*math.sin(at))
        #print("next")
        ax.annotate("",
            xy=(xt+0.1*math.cos(at), yt+0.1*math.sin(at)), xycoords='data',
            xytext=(xt, yt), textcoords='data',
            arrowprops=dict(arrowstyle="->", connectionstyle="arc3"),)
        xtotal[i]=xt
        ytotal[i]=yt
        i=i+1

    
    plt.scatter(xtotal, ytotal)
    #plt.axis('scaled')
    
    
    
    z=len(edge_data)
    
    for i in range(z):
        pt1,pt2=edge_data[i]
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
    #plt.savefig('hi.jpg', bbox_inches='tight', pad_inches=0)   
    plt.savefig('hi.jpg',dpi=100)   
    
        

#main
graph_file_path="graph2.graphml"
mat_file_path="mat2.txt"
plot_graph(graph_file_path,mat_file_path)
  
    
    
    