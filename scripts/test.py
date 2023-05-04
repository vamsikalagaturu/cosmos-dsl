#!/usr/bin/env python3

import os
import rdflib

# Create an RDF graph and parse the monitor instance file
g = rdflib.ConjunctiveGraph()
g.parse('instances/position_instance.jsonld', format='json-ld')
g.parse('instances/position_coord_instance.jsonld', format='json-ld')

# print the graph
print(g.serialize(format='turtle'))

# print the triples in the graph
for s, p, o in g:
    print((s, p, o))