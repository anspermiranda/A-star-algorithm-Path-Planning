'''
This program is an implementation of an autonomous A* algorithm for path planning on a 2D grid (1 = path, 0 = obstacle). 
The program determines the most efficient, obstacle-free path between two points by finding the shortest distance traveled.

Method Summary:
1. Input Validation: Verify the presence of start and end coordinates before proceeding.
2. Optimized Data Storage: 
   - Uses Dictionaries for the "Frontier" data structure, enabling instant lookup of coordinates.
   - More efficient than "List Method" (see below after the code), which requires scanning all items sequentially, 
     resulting in lag on large maps.
3. Initialization: Set initial node cost to 0, calculate Euclidean distance to Goal.
4. Main Search Loop:
   - Select coordinate with minimum total Cost (f-score).
   - If Goal found: Reconstruct path by tracing "parent" pointers.
   - If not: Mark coordinate as "explored" for redundancy prevention.
5. Neighbor Expansion:
   - Check 4-directional neighbors (Up, Down, Left, Right).
   - Exclude boundary or obstacle neighbors.
   - Update records if a neighbor has a shorter path than previously known.
6. Final Output: Return the [Start -> End] path or a warning if blocked.
'''

def do_a_star(grid, start, end, display_message):
    # 1. Start Checklist & Validation (Error Handling)
    # Prevents crashes if user has not specified start/end points in GUI.
    if start is None or end is None:
        display_message("Start or End position not set!", "WARN")
        return []
    
    # 2. Initialize Variables and Data Structures
    # Get grid size for boundary detection.
    COL = len(grid)
    ROW = len(grid[0])
    
    # open_set: The 'Frontier' - nodes that have been discovered but not yet explored.
    # Format: { (col, row): f_n } 
    # Using a dictionary allows for quick lookups and cost updates.
    open_set = {}
    
    # closed_set: The 'Interior' - nodes that have been completely explored.
    # Format: { (col, row): True }
    closed_set = {}
    
    # Tracking Dictionaries:
    # g_score: The cost from start node to current node.
    # parents: The 'parent' of each node for backtracing.
    g_score = {}
    parents = {}

    # 3. Setup the Start Node
    r = 1 # Scaling Factor for Heuristic Weight.
    
    # g(n): Path Cost from Start (0 since we are already there.)
    g_score[start] = 0
    
    # h(n): Heuristic – Euclidean Distance: sqrt{(x_2-x_1)^2 + (y_2-y_1)^2}
    h_start = r * ((end[0] - start[0])**2 + (end[1] - start[1])**2)**0.5
    
    # f(n) = g(n) + h(n)
    open_set[start] = g_score[start] + h_start
    parents[start] = None # Start node has no parent

    display_message("Initializing A* Search Algorithm...", "INFO")
    display_message("Searching for path from " + str(start) + " to " + str(end), "INFO")

    # 4. Main Search Loop
    # The loop continues as long as the Frontier (Open Set) is not empty.
    while open_set:
        # Selection: Find the node in the Open Set (Frontier) with the minimum f(n).
        current = None
        for node in open_set:
            # Compare the f(n) values to select the best node.
            if current is None or open_set[node] < open_set[current]:
                current = node

        # 5. GOAL CHECK: Verify if the current node 'n' is the target destination.
        if current == end:
            display_message("The destination cell is found", "DEBUG")
            
            # PATH RECONSTRUCTION (BACKTRACING):
            # Trace the sequence of movements from the Goal back to the Start.
            path = []
            temp = current
            
            # Iteratively follow 'parent' pointers until the Start node (None) is reached.
            while temp is not None:
                path.append(temp)
                temp = parents[temp]
            
            # Reverse the sequence to get the [Start -> End] path for the Robot.
            path.reverse() 
            display_message("Path successfully calculated.", "INFO")
            return path

        # Move node from Frontier (open_set) to Interior (closed_set).
        # Prevent re-visiting the node and hence the formation of an infinite loop.
        del open_set[current]
        closed_set[current] = True

        # 6. Neighbor Expansion : Expand the nodes in the 4-Way direction.
        curr_col, curr_row = current
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)] # Up, Down, Left, Right
        
        for a, b in directions:
            neighbor = (curr_col + a, curr_row + b)

            # VALIDATION: Ensure the neighbor is within grid bounds, 
            # not an obstacle (0), and not fully explored (Closed Set).
            if 0 <= neighbor[0] < COL and 0 <= neighbor[1] < ROW:
                if grid[neighbor[0]][neighbor[1]] == 0 or neighbor in closed_set:
                    continue
                
                # g(n): Actual Cost from Start to Neighbor (Incremental Unit Cost = 1)
                g_n = g_score[current] + 1
                
                # OPTIMIZATION: Process neighbor if it's new or if a shorter path is discovered.
                if neighbor not in g_score or g_n < g_score[neighbor]:
                    # Update the 'parent' pointer and the best-known path cost g(n).
                    parents[neighbor] = current
                    g_score[neighbor] = g_n
                    
                    # h(n): Heuristic function to find the cost to the Goal node based on the Euclidean distance.
                    h_n = r * ((neighbor[0] - end[0])**2 + (neighbor[1] - end[1])**2)**0.5
                    
                    # f(n) = g(n) + h(n): Total estimated cost for this path.
                    # Place or update the neighbor in the Frontier (open_set).
                    open_set[neighbor] = g_n + h_n

    # 7. Failure State Update
    # This status occurs when the open_set is empty and the goal was never reached.
    display_message("No path found between start and goal.", "WARN")
    return []

# end of file


'''
REFERENCE: PATH PLANNING USING LIST METHOD (OLD VERSION)
The code below is given as a reference to illustrate the transition from a List-based Frontier to an optimized Dictionary-based Frontier.

def do_a_star(grid, start, end, display_message):

    # 1. Start Checklist & Validation(Error Handling)
    # This is to prevent crashes in case the user forgets to set the start or end points in the GUI.
    if start is None or end is None:
        display_message("Start or End position not set!", "WARN")
        return []
    
    # 2. Initialize variables and lists
    # Get the size of the grid for boundary detection.
    COL = len(grid)
    ROW = len(grid[0])
    
    open_list = []      # The 'Frontier': Nodes found but not yet expanded
    closed_list = []    # The 'Interior': Nodes already fully explored
    path = []           # Final list of (col, row) coordinates

    # Coordinates for start and end
    x, y = start[0], start[1]
    xn, yn = end[0], end[1]

    # 3. Setup the Start Node
    r = 1 # Scaling factor for heuristic weight

    # Path cost : g(n) uses Manhattan distance : Path cost from start 
    g_start = 0 # 0 because initially it is in start node, hasn't moved to neighbour nodes

    # Heuristic : h(n) uses Euclidean distance: sqrt((x2-x1)^2 + (y2-y1)^2)
    h_start = r * ((xn-x)**2+(yn-y)**2)**0.5

    # Cost function for Start Node : f(n) = g(n) + h(n)
    f_start = g_start + h_start

    # Node format: (col, row, g(n), h(n), f(n), parent_node)
    start_node = (x,y,g_start,h_start,f_start,None)
    open_list.append(start_node)

    display_message("Initializing A* Search Algorithm...", "INFO")
    display_message("Searching for path from " + str(start) + " to " + str(end), "[INFO]")
    
    # 4. Main Search Loop
    # The loop continues while the frontier is not empty
    while open_list:
        # Search for node with lowest f(n). This ensures we explore the most promising path first
        # Assume the first node is the lowest
        best_node_index = 0
        
        # Search the list to see if there are other lowest f(n) values
        for i in range(len(open_list)):
            # index 4 is the f(n) value
            if open_list[i][4] < open_list[best_node_index][4]:
                best_node_index = i
        
        # Select lowest node for expansion
        current = open_list.pop(best_node_index)
        curr_col, curr_row = current[0], current[1]

        # Add to Interior set to avoid revisiting and creating infinite loops
        closed_list.append((curr_col, curr_row))

        # 5. Goal Check: If we are at the goal, back trace to find path
        if (curr_col, curr_row) == end:
            display_message("The destination cell is found", "DEBUG")

            # Path Reconstruction (Backtracing):
            # Iteratively follow the 'parent' pointers from the goal back to the start.
            temp = current
            while temp is not None:
                path.append((temp[0], temp[1]))
                temp = temp[5] # Move to the parent node saved in index 5
            
            path.reverse() # Reverses the list to get Start -> End sequence
            break # Exit loop to reach the return statement

        # 6. Neighbor Expansion
        # Direction Constraints: Up, Down, Left, Right
        directions = [(0,-1), (0,1), (-1,0), (1,0)]
        
        # Evaluate Neighbors (based on Direction Constraints)
        for a, b in directions:
            ncol, nrow = curr_col + a, curr_row + b

            # Check boundaries : Ensure neighbor exist within the grid limits
            if 0 <= ncol < COL and 0 <= nrow < ROW:
            # Check obstacles : 1 is empty path, 0 is a wall
                if grid[ncol][nrow] == 1:
                    # INTERIOR CHECK: skip if node is already fully explored
                    if (ncol, nrow) in closed_list:
                        continue

                    # COST CALCULATION:
                    # g_n: Number of steps (i.e Path cost) from start (incremented by 1 for each step) 
                    # h_n: Straight-line distance estimated cost to goal
                    g_n= r * (current[2]+1)
                    h_n = r * ((ncol-xn)**2+(nrow-yn)**2)**0.5
                    f_n = g_n + h_n

                    # FRONTIER CHECK : skip if already in open_list
                    already_in_list = False
                    for node in open_list:
                        if node[0] == ncol and node[1] == nrow:
                            already_in_list = True
                            break
                    
                    if not already_in_list:
                        open_list.append((ncol, nrow, g_n, h_n, f_n, current))
    
    # 7. Final Status Update
    if not path:
        # Warn user if search is complete without reaching a goal (path blocked)
        display_message("No path found between start and goal.", "WARN")
    else:
        # Log messages to indicate search is complete successfully
        display_message("Path successfully calculated.", "INFO")

    # Return required (col, row) list of coordinates back to gui.py
    return path
#end of file
'''
