import rdflib
from rdflib.extras.external_graph_libs import rdflib_to_networkx_multidigraph
import networkx as nx
import matplotlib.pyplot as plt
import os
import sys

root_path = os.path.join(os.path.dirname(__file__), '..')
models = f'{root_path}/models/tasks/partial_spec/'
url = f'{models}frames.jsonld'

url = '/home/batsy/rnd/src/dsl/scripts/sample.jsonld'

g = rdflib.ConjunctiveGraph()
g.parse(url, format='json-ld')
print(g.serialize(format='turtle'))

# Convert the rdflib graph to a NetworkX MultiDiGraph
G = nx.MultiDiGraph()

for s, p, o in g:
    # if predicate is of type rdf-syntax-ns#type: None, then make it of type "a"
    print(p)
    G.add_edge(s, o, key=p)

# Create a plot
plt.figure(figsize=(16, 12))  # Increase the figure size

# Use a different layout algorithm with adjusted parameters
# You can experiment with different layout algorithms and parameters
pos = nx.spring_layout(G, iterations=10000, k=10)
# get the seed of the layout algorithm
print(pos)

# Define the node order for each shell
# shell_order = [
#     [rdflib.term.URIRef('http://example.com/planet#Planet'), rdflib.term.URIRef('http://example.com/human#Human'), rdflib.term.URIRef('http://example.com/justice_league#JL')],
#     [rdflib.term.URIRef('http://example.com/planet#earth'), rdflib.term.URIRef('http://example.com/justice_league#justice_league')],
#     [rdflib.term.Literal('Earth', datatype=rdflib.term.URIRef('xsd:string')), rdflib.term.Literal('Bruce Wayne', datatype=rdflib.term.URIRef('xsd:string')), rdflib.term.Literal('Barry Allen', datatype=rdflib.term.URIRef('xsd:string'))],
#     [rdflib.term.URIRef('http://example.com/human#bruce_wayne'), rdflib.term.URIRef('http://example.com/human#barry_allen')],
# ]

shell_order = [

    [
        rdflib.term.URIRef('http://example.com/jl#league'),

    ],

    [

        rdflib.term.URIRef('http://example.com/kryptonian#kal-el'),
        rdflib.term.URIRef('http://example.com/human#bruce_wayne'),

    ],

    [
        rdflib.term.URIRef('http://example.com/planet#Planet'),
        rdflib.term.URIRef('http://example.com/human#Human'),
        rdflib.term.URIRef('http://example.com/jl#JL'),
        rdflib.term.URIRef('http://example.com/kryptonian#Kryptonian'),
    ],

    [

        rdflib.term.URIRef('http://example.com/planet#krypton'),
        rdflib.term.URIRef('http://example.com/planet#earth'),

    ],

    [
        rdflib.term.Literal(
            'Bruce Wayne', datatype=rdflib.term.URIRef('xsd:string')),
        rdflib.term.Literal(
            'Earth', datatype=rdflib.term.URIRef('xsd:string')),
        rdflib.term.Literal(
            'Kal-El', datatype=rdflib.term.URIRef('xsd:string')),
        rdflib.term.Literal(
            'Krypton', datatype=rdflib.term.URIRef('xsd:string')),
    ],
]

# Use the shell layout algorithm
# pos = nx.shell_layout(G, nlist=shell_order, rotate=45)


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
    y = (pos[s][1] + pos[o][1]) / 2
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
