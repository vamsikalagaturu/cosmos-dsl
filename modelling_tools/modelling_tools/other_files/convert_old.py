#!/usr/bin/env python3

import os, sys
import rdflib
from jinja2 import Environment, FileSystemLoader
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Create an RDF graph and parse the monitor instance file
g = rdflib.ConjunctiveGraph()
root_path = os.path.join(os.path.dirname(__file__), '..')
g.parse(root_path + '/models/tasks/xyz_motion/monitors.jsonld', format='json-ld')

# print the graph
print(g.serialize(format='turtle'))