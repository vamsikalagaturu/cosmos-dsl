#!/usr/bin/env python3

import os
import rdflib
from jinja2 import Environment, FileSystemLoader

# Create an RDF graph and parse the monitor instance file
g = rdflib.Graph()
g.parse('instances/position_instance.jsonld', format='json-ld')

# Define the namespaces used in the file
ns = { 
    "position": rdflib.Namespace("http://example.com/position#"),
    "coord-frame": rdflib.Namespace("http://example.com/coordinate_frame#")
}

# print the triples in the graph
for s, p, o in g:
    print(s, p, o)

# get the coord-frame
node = rdflib.URIRef("http://example.com/position#p1-coord")
coord_frame = g.value(s, ns["position"]["in-frame"])

# print the coord-frame
print(coord_frame)
