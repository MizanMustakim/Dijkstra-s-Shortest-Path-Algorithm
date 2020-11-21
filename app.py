from collections import defaultdict

class D:
    def minDistance(self, dist, queue):
        minimum = float("Inf")
        min_index = -1
        
        for i in range(len(dist)):
            if dist[i] < minimum and i in queue:
                minimum = dist[i]
                min_index = i
        return min_index
    
    def printPath(self, parent, j):
        if parent[j] == -1:
            print (j+1, end="  ")
            return
        self.printPath(parent, parent[j])
        print (j+1, end="  ")

    
    def printSolution(self, dist, parent):
        global s
        #src = 0
        des = int(input("Enter the destination: "))
        #for i in range(1, len(dist)):
        print("The Shortest distance from souce{} to destination{} is {:.2f}".format(s, des, dist[des-1]))
        print("The route is: "), self.printPath(parent, des-1)

    def djkstra(self, graph, src):
        row = len(graph)
        col = len(graph[0])
        
        dist = [float('Infinity')] * row
        parent = [-1] * row
        
        dist[src] = 0
        
        q = []
        for i in range(row):
            q.append(i)
        while q:
            u = self.minDistance(dist, q)
            q.remove(u)
            
            for i in range(col):
                if graph[u][i] and i in q:
                    if dist[u] + graph[u][i] < dist[i]:
                        dist[i] = dist[u] + graph[u][i]
                        parent[i] = u
        return self.printSolution(dist, parent)
        
x = D()
# graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0], 
#          [4, 0, 8, 0, 0, 0, 0, 11, 0], 
#          [0, 8, 0, 7, 0, 4, 0, 0, 2], 
#          [0, 0, 7, 0, 9, 14, 0, 0, 0], 
#          [0, 0, 0, 9, 0, 10, 0, 0, 0], 
#          [0, 0, 4, 14, 10, 0, 2, 0, 0], 
#          [0, 0, 0, 0, 0, 2, 0, 1, 6], 
#          [8, 11, 0, 0, 0, 0, 1, 0, 7], 
#          [0, 0, 2, 0, 0, 0, 6, 7, 0]]

graph = [[0, .8, 0, .8, 1.13, 0],
        [.8, 0, .8, 1.13, .8, 1.13],
        [0, .8, 0, 0, 1.13, .8],
        [.8, 1.13, 0, 0, .8, 0],
        [1.13, .8, 1.13, .8, 0, .8],
        [0, 1.13, .8, 0, .8,0]]
s = int(input("Enter the source: "))
x.djkstra(graph, s-1)
