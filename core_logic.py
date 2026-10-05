import math
import heapq

def heuristic(node_a, node_b):
    """
    Calculates the Euclidean (straight-line) distance between two points.
    We use this as the 'H' (Heuristic) score for A*.
    """
    x1, y1 = node_a
    x2, y2 = node_b
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def run_astar(start_node_id, target_node_id, nodes_dict, edges_list):
    """
    
    start_node_id: The ID of the starting node (e.g. (group_id, local_id))
    target_node_id: The ID of the destination node
    nodes_dict: Dictionary mapping node_id -> (X, Y) coordinates
    edges_list: List of tuples representing roads: (node_a_id, node_b_id)
    """
    
    # ---------------------------------------------------------
    # STEP 1: Build an Adjacency List (A map of neighbors)
    # ---------------------------------------------------------
    # edges_list is just a flat list of roads. To easily find "which roads connect to Node A",
    # we convert it into a dictionary where Key = Node, Value = List of Neighbors.
    neighbors = {node_id: [] for node_id in nodes_dict.keys()}
    
    for u, v in edges_list:
        if u in neighbors and v in neighbors:
            # We treat roads as two-way (bidirectional) for now to ensure we can route properly
            neighbors[u].append(v)
            neighbors[v].append(u)

    # ---------------------------------------------------------
    # STEP 2: Setup A* Data Structures
    # ---------------------------------------------------------
    
    # 'open_set' is our Priority Queue. 
    # It always gives us the node with the lowest F score next.
    # We store tuples inside it: (F_score, node_id)
    open_set = []
    heapq.heappush(open_set, (0, start_node_id))
    
    # 'came_from' helps us reconstruct the path backward once we hit the target.
    came_from = {}
    
    # 'g_score' tracks the exact distance traveled from Start to a specific Node.
    # By default, distance to all nodes is Infinity, except the Start node (0).
    g_score = {node_id: float('inf') for node_id in nodes_dict.keys()}
    g_score[start_node_id] = 0
    
    # 'f_score' is (G Score + Heuristic). It guesses total trip distance.
    f_score = {node_id: float('inf') for node_id in nodes_dict.keys()}
    f_score[start_node_id] = heuristic(nodes_dict[start_node_id], nodes_dict[target_node_id])
    
    # ---------------------------------------------------------
    # STEP 3: The Main A* Loop
    # ---------------------------------------------------------
    while open_set:
        # Pop the node with the absolute lowest F score from the Priority Queue
        current_f, current_node = heapq.heappop(open_set)
        
        # WE WON! We reached the target node.
        if current_node == target_node_id:
            # Let's trace our steps backward using the 'came_from' map
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start_node_id)
            
            # The path is traced backward, so we reverse it to get Start -> End
            path.reverse()
            return path
            
        # If we are not at the target, let's explore all roads connected to 'current_node'
        for neighbor in neighbors[current_node]:
            # 1. Calculate the distance between current_node and this neighbor
            coord_current = nodes_dict[current_node]
            coord_neighbor = nodes_dict[neighbor]
            distance_to_neighbor = heuristic(coord_current, coord_neighbor)
            
            # 2. Calculate what the G score WOULD be if we take this road
            tentative_g_score = g_score[current_node] + distance_to_neighbor
            
            # 3. If this new G score is BETTER (lower) than a route we found previously:
            if tentative_g_score < g_score[neighbor]:
                # Update our records because we found a faster shortcut!
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                
                # Calculate new F score (New G + Heuristic to target)
                new_f_score = tentative_g_score + heuristic(coord_neighbor, nodes_dict[target_node_id])
                f_score[neighbor] = new_f_score
                
                # Add it to the priority queue so we can explore it soon
                heapq.heappush(open_set, (new_f_score, neighbor))
                
    # If the Priority Queue becomes empty and we never hit the target, it means
    # there is no physical road connecting the two points. Return empty list.
    return []           
