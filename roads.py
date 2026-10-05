import cv2
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from skan import csr

# 1. LOAD IMAGE AND INVERT
# Load map in grayscale
img = cv2.imread('game_map.png', cv2.IMREAD_GRAYSCALE)

# Threshold to isolate black elements. 
# Roads are very dark, so anything below 40 becomes white (255) on a black background.
_, binary_map = cv2.threshold(img, 40, 255, cv2.THRESH_BINARY_INV)

# 2. FILTER OUT THIN COASTLINES AND WATERMARKS
# Find all black shapes on the map
contours, _ = cv2.findContours(binary_map, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
cleaned_map = np.zeros_like(binary_map)

for cnt in contours:
    # Calculate the thickness/area of the line segment
    area = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt, True)
    
    # Coastline shapes are extremely long but have almost zero filled area.
    # This metric filters out thin boundary lines while keeping thick roads.
    if perimeter > 0:
        thickness_metric = area / perimeter
        if thickness_metric > 1.2:  # Adjust this slightly if thin roads disappear
            cv2.drawContours(cleaned_map, [cnt], -1, 255, -1)

# Smooth out intersections and fill tiny text gaps
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
cleaned_map = cv2.morphologyEx(cleaned_map, cv2.MORPH_CLOSE, kernel)

# 3. SKELETONIZE (Get 1-pixel centerlines)
bool_map = cleaned_map > 0
skeleton = skeletonize(bool_map)

# 4. BUILD STRUCTURAL GRAPH USING SKAN
pixel_graph, coordinates = csr.skeleton_to_csgraph(skeleton)
G = nx.Graph()

# Add intersections/nodes (storing position as X, Y)
for i, coord in enumerate(coordinates):
    G.add_node(i, pos=(int(coord[1]), int(coord[0])))

# Link nodes with simplified edges
from scipy.sparse import coo_matrix
coo = pixel_graph.tocoo()
for u, v in zip(coo.row, coo.col):
    if u < v:
        pos_u = G.nodes[u]['pos']
        pos_v = G.nodes[v]['pos']
        dist = np.linalg.norm(np.array(pos_u) - np.array(pos_v))
        G.add_edge(u, v, weight=dist)

# 5. VISUALIZE RESULTS TO VERIFY ACCURACY
plt.figure(figsize=(10, 10))
# Plot original image as background
plt.imshow(cv2.cvtColor(img, cv2.COLOR_GRAY2RGB))

# Overlay extracted graph nodes and edges
pos = nx.get_node_attributes(G, 'pos')
nx.draw_networkx_nodes(G, pos, node_size=12, node_color='red', alpha=0.8)
nx.draw_networkx_edges(G, pos, edge_color='cyan', width=1.5, alpha=0.6)

plt.title(f"Extracted Nodes: {len(G.nodes)} | Edges: {len(G.edges)}")
plt.axis('off')
plt.show()
