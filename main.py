from shortPathFinder import DijkstraAlgorithm
from Graph import graph

def main():
    x = DijkstraAlgorithm()
    source = int(input("Enter the source: "))
    x.dijkstra(graph, source)

if __name__ == '__main__':
    main()