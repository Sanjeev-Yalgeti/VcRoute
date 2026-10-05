import pygame
import sys
import math
from core_logic import run_astar

# ==========================================
# 1. PARSER: Read paths.ipl
# ==========================================
def load_paths(filepath):
    # Add any Group IDs here that you want to completely erase from the map!
    EXCLUDED_GROUPS = set(
        [1473, 328, 329, 327, 322, 320, 323, 324, 332, 330, 331, 682, 683, 684, 1300, 1284, 1276, 1277, 1278, 1279, 1280]
        + list(range(1081, 1092))
        + list(range(1285, 1316))
    )
    
    nodes = {}  # Key: (group_id, local_id), Value: (X, Y)
    edges = []  # Tuple of ((group, id), (group, id))
    
    current_group_id = -1
    in_path_section = False
    local_node_idx = 0
    
    try:
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                
                if line.lower() == 'path':
                    in_path_section = True
                    continue
                if line.lower() == 'end' and in_path_section:
                    in_path_section = False
                    continue
                    
                if in_path_section:
                    parts = [p.strip() for p in line.split(',')]
                    
                    # Group Header usually has 2 parts: GroupType, NextGroupID (e.g. "1, -1")
                    if len(parts) == 2:
                        current_group_id += 1
                        local_node_idx = 0
                        
                    # Node line usually has 12 parts
                    elif len(parts) >= 12:
                        if current_group_id > 1400 or current_group_id in EXCLUDED_GROUPS:
                            continue # Completely skip this group
                            
                        try:
                            node_type = int(parts[0])
                            next_node_idx = int(parts[1])
                            
                            # Parse coordinates
                            x = float(parts[3])
                            y = float(parts[4])
                            
                            # Lane information (often columns 7 and 8)
                            left_lanes = int(parts[7])
                            right_lanes = int(parts[8])
                            
                            # Filter: If a path has 0 lanes, it's usually a pedestrian, boat, or event path.
                            if left_lanes == 0 and right_lanes == 0:
                                local_node_idx += 1
                                continue
                            
                            # Save node
                            node_key = (current_group_id, local_node_idx)
                            nodes[node_key] = (x, y)
                            
                            # Save internal edge if it connects to a next node
                            if next_node_idx != -1:
                                next_node_key = (current_group_id, next_node_idx)
                                edges.append((node_key, next_node_key))
                                
                            local_node_idx += 1
                        except ValueError:
                            pass # Skip lines that don't parse correctly
    except FileNotFoundError:
        print(f"Error: Could not find {filepath}")
        
    # --- STITCHING DISCONNECTED GROUPS ---
    # GTA VC paths are broken into groups. To route across the whole city, 
    # we need to connect nodes from different groups that physically touch (Intersections)
    print("Stitching map intersections together (this takes a second)...")
    grid = {}
    for node_key, (wx, wy) in nodes.items():
        # Group nodes into 50x50 spatial buckets for super fast distance checking
        gx = int(wx / 50)
        gy = int(wy / 50)
        if (gx, gy) not in grid: grid[(gx, gy)] = []
        grid[(gx, gy)].append(node_key)
        
    cross_group_edges = 0
    for (gx, gy), cell_nodes in grid.items():
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor_cell = (gx + dx, gy + dy)
                if neighbor_cell in grid:
                    for n1 in cell_nodes:
                        for n2 in grid[neighbor_cell]:
                            if n1[0] != n2[0]: # If they are in different groups
                                x1, y1 = nodes[n1]
                                x2, y2 = nodes[n2]
                                dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)
                                if dist < 30.0: # Huge increase to catch long bridges
                                    edges.append((n1, n2))
                                    cross_group_edges += 1

    # --- MANUAL EDGE OVERRIDES ---
    # Connect long straight gaps/bridges in original game files (e.g. Group 363 <-> Group 364)
    manual_connections = [
        ((363, 9), (364, 6)),
        ((364, 6), (363, 9)),
        ((363, 0), (364, 1)),
        ((364, 1), (363, 0)),
    ]
    for n1, n2 in manual_connections:
        if n1 in nodes and n2 in nodes:
            edges.append((n1, n2))

    print(f"Parsed {len(nodes)} nodes, {len(edges) - cross_group_edges} internal edges, and {cross_group_edges} intersection edges.")
    return nodes, edges

# ==========================================
# 2. VIEWER: Pygame UI with Pan & Zoom
# ==========================================
def main():
    nodes, edges = load_paths('GameFiles/paths.ipl')
    
    pygame.init()
    
    # Load background map and maintain aspect ratio!
    try:
        bg_image = pygame.image.load('static_images/map.png')
    except Exception:
        try:
            bg_image = pygame.image.load('map.png')
        except Exception as e:
            print(f"Warning: Could not load map. {e}")
            bg_image = pygame.Surface((1000, 800))
            bg_image.fill((30, 30, 30))

    # Scale image uniformly so it fits well on a monitor (Max height 900)
    img_w, img_h = bg_image.get_size()
    max_height = 750
    if img_h > max_height:
        ratio = max_height / img_h
        img_w = int(img_w * ratio)
        img_h = int(img_h * ratio)
        bg_image = pygame.transform.scale(bg_image, (img_w, img_h))

    WIDTH, HEIGHT = img_w, img_h
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("GTA VC GPS - Map Aligner")
        
    # Coordinate Transform Variables (Perfectly Calibrated)
    offset_x = 394.5
    offset_y = 335.0
    scale = 0.013
    
    # Helper to convert Game World coords to Pygame Screen coords
    def world_to_screen(wx, wy):
        # We invert WY because in-game North is +Y, but Pygame North is -Y
        sx = (wx * scale) + offset_x
        sy = (-wy * scale) + offset_y 
        return int(sx), int(sy)

    def get_closest_node(mouse_pos):
        mx, my = mouse_pos
        closest_node = None
        min_dist = float('inf')
        
        for node_key, (wx, wy) in nodes.items():
            sx, sy = world_to_screen(wx, wy)
            dist = math.sqrt((sx - mx)**2 + (sy - my)**2)
            if dist < min_dist and dist < 20: # Must click within 20 pixels
                min_dist = dist
                closest_node = node_key
                
        return closest_node

    clock = pygame.time.Clock()
    running = True
    show_nodes = False # Toggle this with SPACE
    
    start_node = None
    target_node = None
    calculated_path = []
    
    print("\n--- CONTROLS ---")
    print("Arrow Keys : Move (Pan) the road network")
    print("W / S      : Zoom In / Zoom Out")
    print("SPACEBAR   : Toggle visible road network (for alignment)")
    print("Left Click : Set Start Location (Green)")
    print("Right Click: Set Target Location (Red)\n")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    show_nodes = not show_nodes
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()
                clicked_node = get_closest_node(mouse_pos)
                
                if clicked_node:
                    if event.button == 1: # Left Click
                        start_node = clicked_node
                        print(f"Start Node Set: {start_node}")
                    elif event.button == 3: # Right Click
                        target_node = clicked_node
                        print(f"Target Node Set: {target_node}")
                        
                    # Run A* if we have both points
                    if start_node and target_node:
                        print("Calculating A* Path...")
                        calculated_path = run_astar(start_node, target_node, nodes, edges)
                        if calculated_path:
                            print(f"Path found! ({len(calculated_path)} nodes)")
                        else:
                            print("No valid route found between these points.")
                
        # Handle continuous key presses for smooth panning/zooming
        keys = pygame.key.get_pressed()
        pan_speed = 10
        zoom_speed = 0.005
        
        if keys[pygame.K_LEFT]:  offset_x -= pan_speed
        if keys[pygame.K_RIGHT]: offset_x += pan_speed
        if keys[pygame.K_UP]:    offset_y -= pan_speed
        if keys[pygame.K_DOWN]:  offset_y += pan_speed
        
        if keys[pygame.K_w]:     scale += zoom_speed
        if keys[pygame.K_s]:     scale = max(0.001, scale - zoom_speed)
                
        # Draw everything
        screen.blit(bg_image, (0, 0))
        
        # Draw raw network if toggled ON (For easy alignment)
        if show_nodes:
            for n1_key, n2_key in edges:
                if n1_key in nodes and n2_key in nodes:
                    x1, y1 = nodes[n1_key]
                    x2, y2 = nodes[n2_key]
                    pygame.draw.line(screen, (0, 255, 255), world_to_screen(x1, y1), world_to_screen(x2, y2), 1)
            for gx, gy in nodes.values():
                pygame.draw.circle(screen, (255, 0, 0), world_to_screen(gx, gy), 2)
        
        # Draw calculated A* Path (Thick Magenta Line with Black Outline)
        if len(calculated_path) > 1:
            for i in range(len(calculated_path) - 1):
                n1 = calculated_path[i]
                n2 = calculated_path[i+1]
                x1, y1 = nodes[n1]
                x2, y2 = nodes[n2]
                p1 = world_to_screen(x1, y1)
                p2 = world_to_screen(x2, y2)
                
                # Draw a slightly thicker black line underneath for an "outline" effect
                pygame.draw.line(screen, (0, 0, 0), p1, p2, 12)
                # Draw the bright GPS line on top
                pygame.draw.line(screen, (255, 50, 255), p1, p2, 8) 
                
        # Draw ONLY Start and Target Nodes (Hide the rest of the intersections)
        if start_node:
            sx, sy = nodes[start_node]
            pygame.draw.circle(screen, (0, 255, 0), world_to_screen(sx, sy), 10) # Big Green
            pygame.draw.circle(screen, (0, 0, 0), world_to_screen(sx, sy), 10, 2) # Black outline
            
        if target_node:
            tx, ty = nodes[target_node]
            pygame.draw.circle(screen, (255, 0, 0), world_to_screen(tx, ty), 10) # Big Red
            pygame.draw.circle(screen, (0, 0, 0), world_to_screen(tx, ty), 10, 2) # Black outline
            
        # UI Overlay to show current values
        font = pygame.font.SysFont(None, 24)
        info_text = f"Offset X: {offset_x:.1f} | Offset Y: {offset_y:.1f} | Scale: {scale:.4f}"
        text_surface = font.render(info_text, True, (255, 255, 255), (0, 0, 0))
        screen.blit(text_surface, (10, 10))
            
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    print(f"\nFinal Alignment Values:")
    print(f"OFFSET_X = {offset_x}")
    print(f"OFFSET_Y = {offset_y}")
    print(f"SCALE = {scale}")
    sys.exit()

if __name__ == '__main__':
    main()
