import rdflib
from rdflib.extras.external_graph_libs import rdflib_to_networkx_multidigraph
import networkx as nx
import matplotlib.pyplot as plt
import os
import sys

url = '/home/batsy/rnd/src/modelling_tools/modelling_tools/other_files/sample.jsonld'

g = rdflib.ConjunctiveGraph()
g.parse(url, format='json-ld')

# Convert the rdflib graph to a NetworkX MultiDiGraph
G = nx.MultiDiGraph()

for s, p, o in g:
    # if predicate is of type rdf-syntax-ns#type: None, then make it of type "a"
    print(p)
    G.add_edge(s, o, key=p)

# Create a plot
plt.figure(figsize=(16, 12))  # Increase the figure size

pos = nx.spring_layout(G, iterations=10000, k=10)

# Draw nodes with larger size and more transparency
nx.draw_networkx_nodes(G, pos, node_size=30000,
                       node_color='lightblue', alpha=0.3)

# Draw edges with reduced width and transparency
nx.draw_networkx_edges(G, pos, width=3, node_size=30000,
                       alpha=0.6, edge_color='gray', arrowsize=30)

# Draw edge labels manually with larger font size
edge_labels = {(s, o, k): v for s, o, k, v in G.edges(data='r', keys=True)}
for edge, label in edge_labels.items():
    s, o, k = edge
    label_text = f"{k.split('#')[-1]}"
    label_text = label_text.replace('type', 'a')
    x = (pos[s][0] + pos[o][0]) / 2
    y = (pos[s][1] + pos[o][1]) / 1.75
    plt.text(x, y, label_text, fontsize=18, color='black')  # Increase font size

# Draw node labels with larger font size
# Extract the last part of the URI as labels
node_labels = {n: n.split("/")[-1] for n in G.nodes()}
nx.draw_networkx_labels(G, pos, labels=node_labels,
                        font_size=22, font_color='black')  # Increase font size

# Remove axis
plt.axis('off')

# Show the plot
plt.show()
