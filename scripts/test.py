#!/usr/bin/env python3

import os
import rdflib

# Create an RDF graph and parse the monitor instance file
g = rdflib.ConjunctiveGraph()
g.parse('models/points.jsonld', format='json-ld')
g.parse('models/frames.jsonld', format='json-ld')
g.parse('models/position.jsonld', format='json-ld')
g.parse('models/position_coord.jsonld', format='json-ld')
g.parse('models/distances.jsonld', format='json-ld')
g.parse('models/monitors.jsonld', format='json-ld')

# print the graph
print(g.serialize(format='turtle'))

ROB = "http://example.com"
POINT = rdflib.Namespace(ROB + "/point#")
FRAME = rdflib.Namespace(ROB + "/frame#")
SPATREL = rdflib.Namespace(ROB + "/spatial_relations#")
COORD = rdflib.Namespace(ROB + "/coordinate#")
DIST = rdflib.Namespace(ROB + "/distance#")
MONITOR = rdflib.Namespace(ROB + "/monitor#")
QUDT = rdflib.Namespace("http://qudt.org/schema/qudt/")

print("-"*20)

def recursive_triples(subject, graph):
    for s, p, o in graph.triples((subject, None, None)):
        print(s, p, o)
        for p1, o1 in graph.predicate_objects(o):
            print(o, p1, o1)
            if isinstance(o1, rdflib.term.URIRef):
                recursive_triples(o1, graph)


recursive_triples(None, g)