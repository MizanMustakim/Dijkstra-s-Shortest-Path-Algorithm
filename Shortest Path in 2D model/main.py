import DijkstraAlgo as da
from Graph import graph

def main():
    x = da.DijkstraAlgorithm()
    source = int(input("\nEnter the source: "))   # Take input of the source value
    destination = int(input("Enter the destination: "))
    x.dijkstraWithPath(graph, source, destination)

    shortest_path = x.path()
    distance = x.distance()

    print("\nThe shortest route: ")
    print(*shortest_path)   #It will print the path
    print("The shortest distance is {:.3f}".format(*distance))         #It will print the distance


if __name__ == '__main__':
    main()