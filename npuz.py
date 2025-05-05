'''
The first line of the input contains an integer k, the size of the square grid. 
k * k lines follow each line containing an integer I on the 
tile starting from the top left to bottom right. The empty cell is represented by the number 0.
'''

import heapq

# ------- A* Algorithm Helper Functions -----------

def get_goal_state(k):
    """Generate the goal state for a kxk puzzle.
    For 3x3: (0, 1, 2, 3, 4, 5, 6, 7, 8)
    """
    return tuple(range(k * k))

def manhattan_distance(state, k):
    """Calculate Manhattan distance heuristic for a given state.
    The sum of distances of each tile from its goal position."""
    distance = 0
    for idx, value in enumerate(state):
        if value == 0:  # Skip the empty tile
            continue
        # Calculate target position (where the tile should be)
        target_x, target_y = divmod(value, k)
        # Calculate current position of the tile
        current_x, current_y = divmod(idx, k)
        # Add Manhattan distance for this tile
        distance += abs(target_x - current_x) + abs(target_y - current_y)
    return distance

def get_neighbors(state, k):
    """Generate all possible neighboring states by moving the empty tile (0).
    Returns list of (neighbor_state, move) tuples."""
    neighbors = []
    zero_idx = state.index(0)  # Find position of empty tile
    x, y = divmod(zero_idx, k)  # Convert to 2D coordinates

    # Possible moves: UP, DOWN, LEFT, RIGHT with their coordinate changes
    moves = {'UP': (-1, 0), 'DOWN': (1, 0), 'LEFT': (0, -1), 'RIGHT': (0, 1)}
    
    for move, (dx, dy) in moves.items():
        nx, ny = x + dx, y + dy  # Calculate new position
        # Check if new position is within bounds
        if 0 <= nx < k and 0 <= ny < k:
            n_idx = nx * k + ny  # Convert back to 1D index
            # Create new state by swapping empty tile with neighbor
            new_state = list(state)
            new_state[zero_idx], new_state[n_idx] = new_state[n_idx], new_state[zero_idx]
            neighbors.append((tuple(new_state), move))
    return neighbors

# ------- A* Algorithm Main Function -----------

def a_star(start, k):
    """A* algorithm implementation for solving n-puzzle.
    Returns sequence of moves to reach goal state."""
    # Get the goal state configuration for a kxk puzzle (e.g., [0, 1, 2, 3, 4, 5, 6, 7, 8] for 3x3 puzzle)
    goal = get_goal_state(k)
    
    # Priority queue (open set) initialized with the starting state.
    # Each element is a tuple: (f_score, g_score, current_state, path).
    # f_score = g_score (cost to reach current state) + heuristic (Manhattan distance to goal).
    open_set = [(manhattan_distance(start, k), 0, start, [])]
    
    # A set to keep track of visited states (to prevent cycles).
    visited = set()

    while open_set:
        # Pop the state with the lowest f_score from the priority queue.
        f, g, current, path = heapq.heappop(open_set)
        
        # Check if the current state matches the goal state.
        if current == goal:
            # If goal is reached, return the sequence of moves that lead to the goal.
            return path

        # Skip processing if this state has already been visited.
        if current in visited:
            continue
        
        # Mark the current state as visited.
        visited.add(current)

        # Explore all neighboring states (i.e., states reachable by moving the empty tile).
        for neighbor, move in get_neighbors(current, k):
            if neighbor not in visited:
                # Calculate the f_score for the neighbor state:
                # f_score = g_score (cost so far + 1 for this move) + heuristic (Manhattan distance).
                heapq.heappush(open_set, (
                    g + 1 + manhattan_distance(neighbor, k),  # f_score: g+1 (new cost) + heuristic
                    g + 1,  # g_score: increment by 1 for the move from the current state
                    neighbor,  # Neighbor state
                    path + [move]  # Add this move to the path
                ))

    # Return an empty list if no solution is found (i.e., the goal is unreachable).
    return [] 

# ---------- MAIN PROGRAM ----------
if __name__ == "__main__":
    # Read input from file
    with open('inpnpuzzle.txt', 'r') as f:
        data = list(map(int, f.read().split()))
    
    k = data[0]  # First number is puzzle size (k for kxk puzzle)
    board_values = tuple(data[1:])  # Rest is the initial puzzle configuration
    
    # Solve the puzzle using A* algorithm
    result = a_star(board_values, k)
    
    # Output results: number of moves followed by each move
    print(len(result))
    for move in result:
        print(move)




















'''
# A* Algorithm for Solving N-Puzzle

### Theory:
The A* (A-star) algorithm is a popular and efficient algorithm used in pathfinding and graph traversal. It finds the shortest path from a start state to a goal state by combining the advantages of both Uniform Cost Search (minimizing cost) and Greedy Best-First Search (minimizing estimated distance to goal). A* uses a heuristic to estimate the cost to reach the goal, helping to prioritize the most promising paths.

The algorithm works by maintaining a priority queue (open set) of states, where each state is evaluated based on the sum of two factors:
- `g(n)`: The actual cost to reach the current state from the start state.
- `h(n)`: The estimated cost from the current state to the goal state, typically computed using a heuristic function.

The total cost function `f(n)` is:
    f(n) = g(n) + h(n)
Where `g(n)` is the cost so far, and `h(n)` is the heuristic estimate of the remaining cost.

A* ensures that the path with the lowest cost (sum of actual and estimated costs) is explored first, leading to an optimal solution when the heuristic function is admissible (never overestimates the true cost).

### Algorithm for A* Search:

1. **Input**:
   - A start state and a goal state (both are kxk puzzle configurations).
   - The puzzle size `k` (for a kxk grid).

2. **Initialization**:
   - Define the goal state as a sequence of numbers from 0 to (k*k - 1).
   - Initialize the open set as a priority queue with the start state.
   - Initialize the visited set to keep track of explored states.
   - Set the initial `g_score` of the start state to 0, and `h_score` to the Manhattan distance from the start state to the goal.

3. **Main Loop**:
   - While the open set is not empty:
     1. Pop the state with the lowest `f_score` (i.e., `f_score = g_score + h_score`).
     2. If the current state matches the goal state, return the sequence of moves leading to the goal.
     3. Skip the state if it has already been visited.
     4. Add the state to the visited set.
     5. For each neighbor of the current state (generated by sliding the empty tile):
        - If the neighbor has not been visited, calculate its `f_score`, and add it to the open set with the current path extended by the move.
   - If the open set becomes empty without finding the goal, return an empty list indicating no solution.

4. **Output**:
   - If the goal state is reached, return the sequence of moves.
   - If no solution exists, return an empty list.

### Pseudocode:

1. `f_score = g_score + h_score`
2. For each state, calculate the Manhattan distance as a heuristic:
   - `h(n) = sum(abs(target_x - current_x) + abs(target_y - current_y))`
3. Use a priority queue to explore the state with the minimum `f_score`.

### Example of Manhattan Distance Formula:
For a given tile in the puzzle, if the current tile is at position `(x, y)` and the target tile should be at position `(target_x, target_y)`, the Manhattan distance for that tile is:
    h(n) = |x - target_x| + |y - target_y|

This heuristic ensures that the algorithm efficiently searches through the state space by considering both the cost of reaching a state and the estimated cost to reach the goal.


'''
