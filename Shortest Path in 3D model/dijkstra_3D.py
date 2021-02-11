import numpy as np
import random
import matplotlib.pyplot as plt
import DijkstraAlgo as da


n = 50  # Node Number

### Calculating the randomly uniform number
x1, x2 = 0.0001, 0.005
y1, y2 = 0.0001, 0.005
z1, z2 = 0.0001, 0.005

xs, ys, zs = [],[],[]
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
        if distance(p1,p2) >= .004:
            graph[i].append(0)
        else:
            graph[i].append(distance(p1,p2))
graph = np.array(graph)



'''Now Get start with Dijkstra Algorithm
To get the shortest path and distance, 
First, import DijkstraAlgo.
Then taking input from user on console,
Calling our classes and methods.
Plotting our desired path on 3D interface.
'''

def main():
    global graph
    
    x = da.DijkstraAlgorithm()

    M = x.path()        #Node list of shortest path
    D = x.distance()    #Distance

    des = zs.index(max(zs))+1
    print("\n----------Start 3D Simulation Model Testing----------")
    print('''\n\nHere our simulation will declare the destination
node by itself randomly. The top node of the surface 
will be choosen as the destination node.''')
    print("\n\nThe target destination node is: ", des)
    source = int(input("\nEnter the source: "))   # Take input of the source value
    
    print("\nThe Shortest Path is: ")
    x.dijkstraWithPath(graph, source, des)
    

    x.dijkstraWithPath(graph, M[0], M[-1])
    print("\nThe total distance of this shortest path is: {:.4f} m".format(*D))
    
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')


    [ax.plot(xs[i], ys[i], zs[i], "g*") for i in range(n) if zs[i] == max(zs)]
    [ax.plot(xs[i], ys[i], zs[i], "r.") for i in range(n) if zs[i] != max(zs)] 

    xl, yl, zl= [],[],[]
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