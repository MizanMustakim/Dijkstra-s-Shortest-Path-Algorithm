import numpy as np
import random
import matplotlib.pyplot as plt


n = 50  # Node Number

### Calculating the randomly uniform number
x1, x2 = 0.0001, 0.005
y1, y2 = 0.0001, 0.005
z1, z2 = 0.0001, 0.005

xs = []
ys = []
zs = []
for i in range(n):
    xs.append((x2 - x1)*random.random() + x1)
    ys.append((y2 - y1)*random.random() + y1)
    zs.append((z2 - z1)*random.random() + z1)


## Creating Graph with distances between the nodes
## First to make a function about measuring the distance
## Then checking whether the distance between two nodes is greater than 0.003
## And then wherever we got grater than 0.003, we make it 0. We just assume that
## the nodes are kind of adjacent nodes. So that we can get at most accurate shortest path
## and also the distance.
def distance(p1,p2):
    squared_dist = np.sum((p1-p2)**2, axis=0)
    dist = np.sqrt(squared_dist)
    return dist

graph = []
for i in range(len(xs)):
    graph.append([])
    for j in range(len(xs)):
        p1 = np.array([xs[i], ys[i], zs[i]])
        p2 = np.array([xs[j], ys[j], zs[j]])
        if distance(p1,p2) >= .002:
            graph[i].append(0)
        else:
            graph[i].append(distance(p1,p2))
graph = np.array(graph)



## Now Get start with Dijkstra Algorithm
## to get the shortest path and distance
## First building class for our algorithm

M = []  # The empty list to store the minimum distances for shortest path
D = []  # The empty list to store the distances between the nodes.
class DijkstraAlgoWithPath:
    global M
    
    def minDistance(self, dist, queue):
        minimum = float("Inf")
        min_index = -1
        
        for i in range(len(dist)):
            if dist[i] < minimum and i in queue:
                minimum = dist[i] 
                min_index = i
        return min_index
    
    def printPath(self, parent, j):
        if parent[j] == -1:                 # If 'j' is the source
            print (j+1, end="  ")
            M.append(j+1)
            return 0
        self.printPath(parent, parent[j])   #If 'j' is not the source, call the recursive function
        M.append(j+1)
        print (j+1, end="  ")
        


    def dijkstraWithPath(self, graph, src, des):
        s = src - 1
        row = len(graph)
        col = len(graph[0])
        
        dist = [float('Infinity')] * row    # initializing all distances are inifinity
        parent = [-1] * row                 # The parent array where to store the shortest path tree
        
        dist[s] = 0                         # Distance of source from itself is zero
        
        q = []                              # An empty list to store all vertices in queue
        for i in range(row):
            q.append(i)
        
        # Find the shortest path for all vertices
        while q:
            # Select the minimum distance vertex 
            # from the set of vertices 
            # which are still in the queue
            u = self.minDistance(dist, q)
            q.remove(u)     # Now remove the minimum distance element which already got
            
            # Consider the vertices which are still in the queue,
            # update the distance and parent index of the adjacent vertices
            # which are selected 
            for i in range(col):
                if graph[u][i] and i in q:  # If dist[i] in the queue
                    if dist[u] + graph[u][i] < dist[i]: # and if the total weight of path from source to destination is less than the current value of dist[i]
                        dist[i] = dist[u] + graph[u][i]
                        parent[i] = u
        self.printPath(parent, des-1)

class DijkstraAlgoWithDistance:
    global D
    
    def minDistance(self, dist, queue):
        minimum = float("Inf")
        min_index = -1
        
        for i in range(len(dist)):
            if dist[i] < minimum and i in queue:
                minimum = dist[i] 
                min_index = i
        return min_index
    
    def printSolution(self, distance, parent, src, des):
        #print(distance[des-1])
        D.append(distance[des-1])
        
    def dijkstraWithDistance(self, graph, src, des):
        s = src - 1
        row = len(graph)
        col = len(graph[0])
        
        dist = [float('Infinity')] * row    # initializing all distances are inifinity
        parent = [-1] * row                 # The parent array where to store the shortest path tree
        
        dist[s] = 0                         # Distance of source from itself is zero
        
        q = []                              # An empty list to store all vertices in queue
        for i in range(row):
            q.append(i)
        
        # Find the shortest path for all vertices
        while q:
            # Select the minimum distance vertex 
            # from the set of vertices 
            # which are still in the queue
            u = self.minDistance(dist, q)
            q.remove(u)     # Now remove the minimum distance element which already got
            
            # Consider the vertices which are still in the queue,
            # update the distance and parent index of the adjacent vertices
            # which are selected 
            for i in range(col):
                if graph[u][i] and i in q:  # If dist[i] in the queue
                    if dist[u] + graph[u][i] < dist[i]: # and if the total weight of path from source to destination is less than the current value of dist[i]
                        dist[i] = dist[u] + graph[u][i]
                        parent[i] = u
        self.printSolution(dist, parent, src, des)


### Now it's time to call our class and it's methods.
### First, we make a main function
### Then taking input from user on console,
### Calling our classes and methods.
### Plotting our desired path on 3D interface.

def main():
    global graph
    
    x = DijkstraAlgoWithPath()
    source = int(input("\nEnter the source: "))   # Take input of the source value
    des = zs.index(max(zs))+1
    print("The Shortest Path is: ")
    x.dijkstraWithPath(graph, source, des)
    
    m = DijkstraAlgoWithDistance()
    m.dijkstraWithDistance(graph, M[0], M[-1])
    print("\nThe total distance of this shortest path is: {:.4f}m".format(sum(D)))
    
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')

        
    a = [ax.plot(xs[i], ys[i], zs[i], "g*") for i in range(n) if zs[i] == max(zs)]
    b = [ax.plot(xs[i], ys[i], zs[i], "r.") for i in range(n) if zs[i] != max(zs)]

    xl = []
    yl = []
    zl = []
    for j in M:
        xl.append(xs[j-1])
        yl.append(ys[j-1])
        zl.append(zs[j-1])
    
    ax.plot(xl, yl, zl, "k:")
    

    ax.legend(["Destination Node","Source Node"])

    ax.set_xlim3d([0.0001, 0.005])
    ax.set_xlabel('X-axis')

    ax.set_ylim3d([0.0001, 0.005])
    ax.set_ylabel('Y-axis')

    ax.set_zlim3d([0.0001, 0.005])
    ax.set_zlabel('Z-axis')

    M.clear()
    D.clear()

    plt.show()
    
if __name__ == '__main__':
    main()