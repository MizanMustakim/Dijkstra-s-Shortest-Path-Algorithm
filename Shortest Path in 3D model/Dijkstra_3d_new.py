import numpy as np
import random
import matplotlib.pyplot as plt

class Node:
    
    def distance(self,p1,p2):
        squared_dist = np.sum((p1-p2)**2, axis=0)
        dist = np.sqrt(squared_dist)
        return dist

    def fat_node(self):
        n = 50

        ### Calculating the randomly uniform number
        x1, x2 = 0.01, 10
        y1, y2 = 0.01, 5
        z1, z2 = 0.01, 5
        xs, ys, zs = [],[],[]
        for i in range(n):
            xs.append((x2 - x1)*random.random() + x1)
            ys.append((y2 - y1)*random.random() + y1)
            zs.append((z2 - z1)*random.random() + z1)

        xs.sort()
        graph_fat = []
        for i in range(int(n)):
            graph_fat.append([])
            for j in range(int(n)):
                p1 = np.array([xs[i], ys[i], zs[i]])
                p2 = np.array([xs[j], ys[j], zs[j]])
                if self.distance(p1,p2) < 3:
                    graph_fat[i].append(self.distance(p1,p2))
                else:
                    graph_fat[i].append(0)
        graph_fat = np.array(graph_fat)

        return xs, ys, zs, graph_fat
    
    def skin_node(self):
        n = 50

        ### Calculating the randomly uniform number
        x1, x2 = 0.01, 10
        y1, y2 = 0.01, 5
        z1, z2 = 5.01, 10
        xs, ys, zs = [],[],[]
        for i in range(n):
            xs.append((x2 - x1)*random.random() + x1)
            ys.append((y2 - y1)*random.random() + y1)
            zs.append((z2 - z1)*random.random() + z1)

        xs.sort()
        graph_skin = []
        for i in range(int(n)):
            graph_skin.append([])
            for j in range(int(n)):
                p1 = np.array([xs[i], ys[i], zs[i]])
                p2 = np.array([xs[j], ys[j], zs[j]])
                if self.distance(p1,p2) < 3:
                    graph_skin[i].append(self.distance(p1,p2))
                else:
                    graph_skin[i].append(0)
        graph_skin = np.array(graph_skin)

        return xs, ys, zs, graph_skin

node = Node()
a = node.fat_node()     # Calling fat node function
b = node.skin_node()    # Calling Sking node function

xs_fat, ys_fat, zs_fat = a[0], a[1], a[2]   # axis points of fat nodes
graph_fat = a[3]

xs_skin, ys_skin, zs_skin = b[0], b[1], b[2]    # axis points of skin nodes
graph_skin = b[3]



M = []  # The empty list to store the minimum distances for shortest path
D = []  # The empty list to store the distances between the nodes.
class DijkstraAlgoWithPath:
    global M
    global D
    
    def minDistance(self, dist, queue):
        minimum = float("Inf")
        min_index = -1
        
        for i in range(len(dist)):
            if dist[i] < minimum and i in queue:
                minimum = dist[i] 
                min_index = i
        return min_index
    
    ### Printing path for fat medium
    def printPath_fat(self, parent, j):
        if parent[j] == -1:                 # If 'j' is the source
            print (j+1, end="  ")
            M.append(j+1)
            return 0
        self.printPath_fat(parent, parent[j])   #If 'j' is not the source, call the recursive function
        M.append(j+1)
        print (j+1, end="  ")

    ### Printing path for skin medium
    def printPath_skin(self, parent, j):
        if parent[j] == -1:                 # If 'j' is the source
            print (j+1+50, end="  ")
            M.append(j+1)
            return 0
        self.printPath_skin(parent, parent[j])   #If 'j' is not the source, call the recursive function
        M.append(j+1)
        print (j+1+50, end="  ")

    ### Dijkstra for fat medium
    def dijkstraWithPath_fat(self, graph, src, des):
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
        self.printPath_fat(parent, des-1)


    ### Dijkstra for Skin medium   
    def dijkstraWithPath_skin(self, graph, src, des):
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
        self.printPath_skin(parent, des-1)

    ### Distance Calculation    
    def distance_calc(self, graph, src, des):
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
        D.append(dist[des-1])

def fat(xl, yl, zl):
    x = DijkstraAlgoWithPath()
    des = zs_fat.index(max(zs_fat))+1
    print("\n\nIn Fat Medium:----------")
    print("\nThe target destination node in Fat medium is: ", des)
    source = int(input("Enter the source at Fat tissue medium: "))
    print("The Shortest Path in Fat tissue medium is: ")
    x.dijkstraWithPath_fat(graph_fat, source, des)
    x.distance_calc(graph_fat, M[0], M[-1])
    print("\n\nThe total distance of this shortest path in Fat medium is: {:.4f}mm".format(sum(D)))
    
    ### Storing the axis value of shortest path's nodes
    for j in M:
        xl.append(xs_fat[j-1])
        yl.append(ys_fat[j-1])
        zl.append(zs_fat[j-1])
    
    M.clear()
    D.clear()

def skin(xl, yl, zl):
    x = DijkstraAlgoWithPath()
    des = zs_skin.index(max(zs_skin))+1
    print("\n\nIn Skin Medium:----------")
    print("\nThe target destination node in Skin medium is: ", des+50)
    source = int(input("Enter the source at Skin tissue medium: "))
    print("The Shortest Path in Skin tissue medium is: ")
    x.dijkstraWithPath_skin(graph_skin, source-50, des)
    x.distance_calc(graph_skin, M[0], M[-1])
    print("\n\nThe total distance of this shortest path in Skin medium is: {:.4f}mm".format(sum(D)))

    ### Storing the axis value of shortest path's nodes
    for j in M:
        xl.append(xs_skin[j-1])
        yl.append(ys_skin[j-1])
        zl.append(zs_skin[j-1])
    
    M.clear()
    D.clear()

def plotting(xl_fat, yl_fat, zl_fat, xl_skin, yl_skin, zl_skin):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    [ax.plot(xs_skin[i], ys_skin[i], zs_skin[i], "ro") for i in range(50) if zs_skin[i] == max(zs_skin)]
    [ax.plot(xs_fat[i], ys_fat[i], zs_fat[i], "mo") for i in range(50) if zs_fat[i] == max(zs_fat)]
    [ax.plot(xs_fat[i], ys_fat[i], zs_fat[i], "g.") for i in range(50) if zs_fat[i] != max(zs_fat)]
    [ax.plot(xs_skin[i], ys_skin[i], zs_skin[i], "b.") for i in range(50) if zs_skin[i] != max(zs_skin)]

    ax.legend(["Nano Router-1","Nano Router-2"])

    ax.plot(xs_fat, ys_fat, zs_fat, "y:")
    ax.plot(xs_skin, ys_skin, zs_skin, "y:")

    ax.plot(xl_fat, yl_fat, zl_fat, "k-")
    ax.plot(xl_skin, yl_skin, zl_skin, "k-")

    ax.set_xlim3d([0, 10])
    ax.set_xlabel('X-distance (mm)')

    ax.set_ylim3d([0, 5])
    ax.set_ylabel('Y-distance (mm)')

    ax.set_zlim3d([0, 10])
    ax.set_zlabel('Z-distance (mm)')

    ax.text(8.5,5,4.5, "Fat", color='green')
    ax.text(8.5,5,9, "Skin", color='blue')

    ax.xaxis.pane.set_facecolor("Orange")
    ax.yaxis.pane.set_facecolor("Orange")
    ax.zaxis.pane.set_facecolor("Orange")

    plt.show()


if __name__ == '__main__':
    xl_fat, yl_fat, zl_fat = [],[],[]
    xl_skin, yl_skin, zl_skin = [],[],[]

    print("\n----------3D Simulation Model Testing----------")
    print('''\n\nHere our simulation will declare the destination
node by itself randomly. The top node of the surface 
will be choosen as the destination node.''')

    fat(xl_fat, yl_fat, zl_fat)
    skin(xl_skin, yl_skin, zl_skin)
    plotting(xl_fat, yl_fat, zl_fat, xl_skin, yl_skin, zl_skin)

    print("\n\t\t \\\\\\ Thank You ///")