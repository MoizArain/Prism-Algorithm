from asyncio.windows_events import INFINITE, NULL
import math
import heapq
import time
import os
import matplotlib.pyplot as plt
from pathlib import Path

def prims(graph, start):
    key = dict() 
    minHeap = []
    parent = dict()
    #This set adds all the nodes of the Minimum Spanning Tree
    mst = set([start]) 
    #initialize all the parents to null and the key to infinity for each node in the graph
    for v in graph:
        parent[v] = NULL
        key[v] = INFINITE

    #The starting node has a key of 0 and it is first pushed to the heap   
    key[start] = 0
    heapq.heappush(minHeap,(0,start))
    
    #check if the minHeap is not empty, if it's empty then no more vertices to visit
    while minHeap:
        #pop the vertex with minimum weight from minHeap
        weight, u = heapq.heappop(minHeap)
        #The popped node has a minimum weight value, then we add it to the mst
        if u not in mst:
            #add the node to mst 
            mst.add(u)

        for v, weight in graph[u]:
            #Only update the parent, key, and minHeap if the edge is minimum and the node is not already a part of MST
            if v not in mst and v in key and weight < key[v]:  
                parent[v] = u
                key[v] = weight
                #add the node with the weight to the minHeap
                heapq.heappush(minHeap,(weight,v))
                
    #calculates the minimum weight obtained by Prim's Algorithm
    totalWeight = 0 
    for v in key: #all the stored key at the end are equal to the minimum weight between v and parent[v]
        totalWeight = totalWeight + key[v] 
    return totalWeight

def plotResult(val):
    #sort the keys (number of vertices) of the dictionary and plot them
    plt.plot(*zip(*sorted(val) ),':k')

    # x-axis label
    plt.xlabel('Number of Vertices')
    # frequency label
    plt.ylabel('Execution Time')
    # plot title
    plt.title('Prim Algorithm plot')
    #maximize the plot size 
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    # function to show the plot
    plt.show()
    
def printSizeTime(verticeEdgeTime):
    print("Size\t\tTime of result\t\tAsymptotic Complexity")
    print(60*"-")
    for i in range(len(verticeEdgeTime)):
        print(f"{verticeEdgeTime[i][0]:7d} {' ':10s} {str(verticeEdgeTime[i][2]):20s}{(verticeEdgeTime[i][2]*(abs(verticeEdgeTime[i][1])* math.log(abs(verticeEdgeTime[i][0])))):30f}")
    print(60*"-")
    
if __name__ == '__main__':
    directory = (r'C:\Users\moiez\OneDrive\Desktop\mst_dataset')
    mapVertice_time = []
    vertice_edge_time = []
    weights=[]
    num_instances = 0 #count number of files
    totalTime = time.time()
    
    print('Weight:\t\t\tTime:\t\t\tVertices:\t\t\tFile:')

    # iterate over files in that directory
    for filename in os.listdir(directory): 
        
        f = os.path.join(directory, filename)
        # checking if it is a file
        if os.path.isfile(f): 
            num_instances = num_instances +1
            #read from the file and split it line by line
            graphRead, graph = open(f).read().splitlines(), {} 
            #count the number of edges of a graph
            edgesCount = graphRead[0].split()[1] 
            #count the number of node of a graph
            verticesCount = graphRead[0].split()[0] 
            #start reading from second line onwards, since first line displays number of nodes and edges
            for line in graphRead[1:]:
                edge = list(map(int, line.split()))
                #add the node to the graph and initialize it with an empty list 
                if edge[0] not in graph: 
                    graph[edge[0]] = []
                if edge[1] not in graph:
                    graph[edge[1]] = []

                #For each node u, append the map of second node v along with the weight (u,v) of the edge
                #For each node v, append the map of second node u along with the weight (u,v) of the edge
                graph[edge[0]].append(edge[1:])
                graph[edge[1]].append([edge[0], edge[2]])

            #calculate the time of prim's algorithm on one graph
            start = time.time() 
            weight = prims(graph,next(iter(graph)))
            end_start = float(time.time() - start)
            #append the obtained weight to the weights list to displayed it at the end and in the graph
            weights.append(weight) 

            #add the time taken corresponding to the number of nodes in a map for each graph to plot it at the end in a line chart
            mapVertice_time.append((int(verticesCount), end_start) ) 
            vertice_edge_time.append((int(verticesCount), int(edgesCount), end_start))

            print(f"{str(weight):20s} {str(end_start):30s} {verticesCount:20s} {str(Path(f).stem):10s}")

    print("------------------------------------------------\n")
        
        
    finalTotalTime = (time.time() -totalTime)    
    printSizeTime(vertice_edge_time)
    print("Average time (s):", finalTotalTime/num_instances , "\tTotal time (s):", finalTotalTime, "\nWeights: ", weights,"\n" )

    #print(mapVertice_time)
    #used to plot the result obtained as in the time needed given a certain number of edges
    plotResult(mapVertice_time)
