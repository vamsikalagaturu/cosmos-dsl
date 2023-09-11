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

HUMAN = rdflib.Namespace("http://example.com/human#")
EX_HUMAN = rdflib.Namespace("http://example_human.com/")

# get the name of the human
for s, p, o in g.triples((None, HUMAN.name, None)):
    print(s, o)

query = f"""
    PREFIX human: <{HUMAN}>
    PREFIX ex_human: <{EX_HUMAN}>

    SELECT ?name ?human
    WHERE {{
        ?human human:name ?name .
    }}
"""

for row in g.query(query):
    print(row)