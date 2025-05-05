import heapq
from collections import defaultdict
from math import sqrt

def euclidean_distance(coord1, coord2):
    x1, y1 = coord1
    x2, y2 = coord2
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def a_star(graph, coordinates, start, goal):
    open_list = []
    heapq.heappush(open_list, (euclidean_distance(coordinates[start], coordinates[goal]), 0, start, [start]))
    g_score = {start: 0}

    while open_list:
        f, g, current, path = heapq.heappop(open_list)

        if current == goal:
            return g, path  # Distance and path

        for neighbor in graph[current]:
            tentative_g = g + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + euclidean_distance(coordinates[neighbor], coordinates[goal])
                heapq.heappush(open_list, (f_score, tentative_g, neighbor, path + [neighbor]))

    return -1, []  # No path

def assign_coordinates(graph):
    coordinates = dict()
    visited = set()
    queue = []

    start_city = next(iter(graph))
    coordinates[start_city] = (0, 0)
    visited.add(start_city)
    queue.append(start_city)

    while queue:
        current = queue.pop(0)
        x, y = coordinates[current]
        dx_dy = [(1,0), (0,1), (-1,0), (0,-1)]
        dir_idx = 0

        for neighbor in graph[current]:
            if neighbor not in visited:
                dx, dy = dx_dy[dir_idx % 4]
                coordinates[neighbor] = (x + dx, y + dy)
                dir_idx += 1
                visited.add(neighbor)
                queue.append(neighbor)

    return coordinates

def read_input(filename):
    with open(filename, 'r') as f:
        lines = f.read().strip().splitlines()

    N, M = map(int, lines[0].split())
    graph = defaultdict(list)

    for i in range(1, N + 1):
        a, b = map(int, lines[i].split())
        graph[a].append(b)
        graph[b].append(a)

    queries = []
    for i in range(N + 1, N + 1 + M):
        start, end = map(int, lines[i].split())
        queries.append((start, end))

    return graph, queries

def main():
    filename = 'inpcities.txt'
    graph, queries = read_input(filename)
    coordinates = assign_coordinates(graph)

    for start, end in queries:
        distance, path = a_star(graph, coordinates, start, end)
        print(distance)
        print("->".join(map(str, path)))

if __name__ == '__main__':
    main()




'''
Theory:
The A* (A-Star) algorithm is a popular graph traversal and pathfinding algorithm used to find the shortest path between two nodes. It combines features of Dijkstra's algorithm and a heuristic to efficiently guide the search.

The formula used is:
    f(n) = g(n) + h(n)
Where:
- g(n): actual cost from start to node n
- h(n): estimated cost from node n to the goal (heuristic)
- f(n): estimated total cost of the path through n

In this question:
- Nodes are cities.
- All direct city connections (edges) have cost 1.
- The heuristic h(n) is the Euclidean distance between cities, computed from (x, y) coordinates assigned to each node.

Logic:
1. The graph is read from a file containing city connections and queries.
2. Each city is assigned a coordinate (x, y) to enable heuristic calculation.
3. A* is applied to each query pair (start, goal) to calculate the shortest path.
4. For each path, the algorithm prints the number of connections (distance) and the exact path taken.

A* is efficient for this type of problem since it guarantees the shortest path if the heuristic (Euclidean) is admissible (i.e., never overestimates the actual cost).
'''
