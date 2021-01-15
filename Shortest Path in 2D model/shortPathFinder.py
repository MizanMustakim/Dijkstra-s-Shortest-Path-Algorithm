class DijkstraAlgorithm:
    
    # This function is to find the minimum distance value of vertex
    # from the set of vertices still in queue

    def minDistance(self, dist, queue):
        minimum = float("Inf")      # initialize the minimum value
        min_index = -1              # initialize the minimum index
        
        # from the distance array, find one which has min value
        # and is still in queue
        for i in range(len(dist)):
            if dist[i] < minimum and i in queue:
                minimum = dist[i]
                min_index = i
        return min_index
    
    # This function is to find the shortest path
    # from the source to point 'j' by using parent array

    def printPath(self, parent, j):
        if parent[j] == -1:                 # If 'j' is the source
            print (j+1, end="  ")
            return 0
        self.printPath(parent, parent[j])   #If 'j' is not the source, call the recursive function
        print (j+1, end="  ")

    # A utility function to construct the distance
    # and to print the shortest route

    def printSolution(self, dist, parent, src):
        des = int(input("Enter the destination: "))         # Taking input of the desired destination
        print("The Shortest distance from source {} to destination {} is {:.2f}".format(src, des, dist[des-1]))
        print("The route is: "), self.printPath(parent, des-1)  # Calling the printPath function to print the route

    # A function that implements the Dijkstra's single source 
    # shortest path algorithm for a graph represented 
    # using adjacency matrix representation

    def dijkstra(self, graph, src):
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
        self.printSolution(dist, parent, src)    # Return the constructed distance 
        