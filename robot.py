import heapq

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# ----- A* algorithm ------
def a_star(start, goal):
    # Priority queue (min-heap) for open set, storing (f_score, g_score, current_node)
    open_set = []
    # Push the start node with initial f_score (Manhattan distance) and g_score (0)
    heapq.heappush(open_set, (manhattan(start, goal), 0, start))
    # Dictionary to store the cost from start to each node (g_score)
    g_score = {start: 0}
    
    while open_set:
        # Pop the node with the smallest f_score
        _, cost, current = heapq.heappop(open_set)
        
        # If we've reached the goal, return the g_score (total distance)
        if current == goal:
            return g_score[current]
        
        # Explore all 4 possible neighbors (up, down, left, right)
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            # Tentative g_score is current g_score + 1 (since each move costs 1)
            tentative_g = g_score[current] + 1
            
            # If this path to neighbor is better than any previous one, update
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + manhattan(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
    
    # Return -1 if no path found
    return -1

# Function to compute the total path length by summing distances between consecutive points
def compute_total_path_length(points):
    # If there are no points or only one point, path length is invalid
    if not points or len(points) < 2:
        return -1
    
    total = 0
    # Iterate through each pair of consecutive points
    for i in range(len(points) - 1):
        # Calculate the Manhattan distance between current point and next point
        dist = a_star(points[i], points[i+1])
        # If any segment is invalid, return -1
        if dist == -1:
            return -1
        total += dist
    return total

# Main function to handle input and output
def main():
    try:
        # Read the number of points
        n = int(input().strip())
        if n <= 0:
            print(-1)
            return
        points = []
        for _ in range(n):
            # Read each point (x, y)
            parts = input().strip().split()
            # If point doesn't have exactly 2 coordinates, it's invalid
            if len(parts) != 2:
                print(-1)
                return
            x, y = map(int, parts)
            # Check if coordinates are within bounds
            if not (0 <= x <= 1000000 and 0 <= y <= 1000000):
                print(-1)
                return
            points.append((x, y))
        
        print(compute_total_path_length(points))
    except:
        print(-1)

if __name__ == "__main__":
    main()
