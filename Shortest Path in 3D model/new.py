import numpy as np
import random
import matplotlib.pyplot as plt



class Node:
    
    def distance(self,p1,p2):
        squared_dist = np.sum((p1-p2)**2, axis=0)
        dist = np.sqrt(squared_dist)
        return dist
    def node(self):
        n = 50

        ### Calculating the randomly uniform number
        x1, x2 = 0.01, 10
        y1, y2 = 0.01, 5
        z1, z2 = 0.01, 5
        xs_fat, ys_fat, zs_fat = [],[],[]
        for i in range(n):
            xs_fat.append((x2 - x1)*random.random() + x1)
            ys_fat.append((y2 - y1)*random.random() + y1)
            zs_fat.append((z2 - z1)*random.random() + z1)

        x3, x4 = 0.01, 10
        y3, y4 = 0.01, 5
        z3, z4 = 5.01, 10
        xs_skin, ys_skin, zs_skin = [],[],[]
        for i in range(n):
            xs_skin.append((x4 - x3)*random.random() + x3)
            ys_skin.append((y4 - y3)*random.random() + y3)
            zs_skin.append((z4 - z3)*random.random() + z3)
        
        xs, ys, zs = [],[],[]
        xs.extend(xs_fat); xs.extend(xs_skin)
        ys.extend(ys_fat); ys.extend(ys_skin)
        zs.extend(zs_fat); zs.extend(zs_skin)
        
        graph = []
        for i in range(len(xs)):
            graph.append([])
            for j in range(len(xs)):
                p1 = np.array([xs[i], ys[i], zs[i]])
                p2 = np.array([xs[j], ys[j], zs[j]])
                if self.distance(p1,p2) < 3.5:
                    graph[i].append(self.distance(p1,p2))
                else:
                    graph[i].append(0)
        graph = np.array(graph)

        return xs, ys, zs, graph

node = Node()
a = node.node() 
xs, ys, zs = a[0], a[1], a[2]   # axis points of fat nodes
graph = a[3]



class DijkstraAlgoWithPath:

    def __init__(self):

        '''Initializing the instances'''

        self.min_dis_index = []
        self.short_dis = []
    
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
            # print (j+1, end="  ")
            self.min_dis_index.append(j+1)
            return 0
        self.printPath(parent, parent[j])   #If 'j' is not the source, call the recursive function
        self.min_dis_index.append(j+1)
        # print (j+1, end="  ")

    def distance(self):
        '''Return the Distance of the measured path'''

        return self.short_dis

    def path(self):
        '''Return the Shortest Path'''

        return self.min_dis_index

    def dijkstraWithPath(self, graph, src, des):
        source = src - 1
        row = len(graph)
        col = len(graph[0])
        
        dist = [float('Infinity')] * row    # initializing all distances are inifinity
        parent = [-1] * row                 # The parent array where to store the shortest path tree
        
        dist[source] = 0                         # Distance of source from itself is zero
        
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

        self.short_dis.append(dist[des-1])      #The measured Distance
        return self.printPath(parent, des-1)    
        
        



def main():
    global graph
    
    short_dis = []

    x = DijkstraAlgoWithPath()
    M = x.path()
    D = x.distance()

    s = sorted(range(len(zs)), key=lambda i: zs[i], reverse=True)[:5]
    source = int(input("\nEnter the source node: "))   # Take input of the source value

    for index, value in enumerate(s):

        print(f"\n*** Simulation with top node {index+1} ***")

        
        des = value + 1
        x.dijkstraWithPath(graph, source, des)
        
        print("\nThe target destination node is: ", des)
        
        print("The Shortest Path is: ", *M)
        
        
        fat = []
        skin = []
        skin.extend(list(filter(lambda data: data > 50, M)))
        fat.extend(list(filter(lambda data: data <= 50, M)))

        M.clear();D.clear()



        '''Separting the path into two types of nodes in between Fat and Skin medium'''

        x.dijkstraWithPath(graph, fat[0], fat[-1])
        print("\nThe nodes in  Fat tissue are: ", *fat)

        x.dijkstraWithPath(graph, skin[0], skin[-1])
        print("The nodes in  Skin tissue are: ", *skin)
        
        
        dis = D[0] + D[1] * 1.471429    #Calculating with distance relation.
        short_dis.append(dis)
        print("The total distance of this shortest path is: {:.4f} mm".format(dis))

        fig = plt.figure()
        ax = fig.add_subplot(111,projection='3d')


        ax.plot(xs[value], ys[value], zs[value], "ro")  #Destination
        [ax.plot(xs[i], ys[i], zs[i], "g.") for i in range(len(xs)) if zs[i] <= 5]      #Fat Nodes
        [ax.plot(xs[i], ys[i], zs[i], "b.") for i in range(len(xs)) if zs[i] > 5 and zs[i] != zs[value]]    #Skin Nodes

        xl, yl, zl= [],[],[]
        for j in M:
            xl.append(xs[j-1])
            yl.append(ys[j-1])
            zl.append(zs[j-1])
        
        ax.plot(xl, yl, zl, "k:")
        

        ax.legend(["Destination Node","Source Node"])

        ax.set_xlim3d([0, 10])
        ax.set_xlabel('X-distance (mm)')

        ax.set_ylim3d([0, 5])
        ax.set_ylabel('Y-distance (mm)')

        ax.set_zlim3d([0, 10])
        ax.set_zlabel('Z-distance (mm)')

        ax.text(8.5,5,4.5, "Fat", color='green')
        ax.text(8.5,5,9, "Skin", color='blue')
        if index == 0:
            plt.title(f"Simulation with top {index+1}st node")
        elif index == 1:
            plt.title(f"Simulation with top {index+1}nd node")
        elif index == 2:
            plt.title(f"Simulation with top {index+1}rd node")
        else:
            plt.title(f"Simulation with top {index+1}th node")

        M.clear(); D.clear()
        fat.clear(); skin.clear()
        xl.clear(); yl.clear(); zl.clear()
        
        plt.show()
    
    n = short_dis.index(min(short_dis))
    print("\n\nFinally, the selected destination node is {}\nAnd the shortest distance from the source to the destination is {:.4f}mm.".format(s[n]+1, min(short_dis)))
    
if __name__ == '__main__':

    print("\n----------Start 3D Simulation Model Testing----------")
    print('''\n\nHere our simulation will declare the destination
node by itself randomly. The top node of the surface 
will be choosen as the destination node.''')

    main()