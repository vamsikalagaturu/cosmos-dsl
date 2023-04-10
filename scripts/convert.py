#!/usr/bin/env python3

import os
import rdflib
from jinja2 import Environment, FileSystemLoader

# Create an RDF graph and parse the monitor instance file
g = rdflib.Graph()
g.parse('examples/monitor_instance.jsonld', format='json-ld')

# Define the namespaces used in the file
ns = {
    "monitor": rdflib.Namespace("http://example.com/monitor#"),
    "point": rdflib.Namespace("http://example.com/point#")
}

# Get the two point nodes from the monitor instance
monitor_node = rdflib.URIRef("http://example.com/monitors/AB")
point1_node = g.value(monitor_node, ns["monitor"]["point1"])
point2_node = g.value(monitor_node, ns["monitor"]["point2"])

# Get the values for x, y, and z of the two points
point1_x = g.value(point1_node, ns["point"]["x"])
point1_y = g.value(point1_node, ns["point"]["y"])
point1_z = g.value(point1_node, ns["point"]["z"])
point2_x = g.value(point2_node, ns["point"]["x"])
point2_y = g.value(point2_node, ns["point"]["y"])
point2_z = g.value(point2_node, ns["point"]["z"])

# Print the values for the two points
print("Point 1: ({}, {}, {})".format(point1_x, point1_y, point1_z))
print("Point 2: ({}, {}, {})".format(point2_x, point2_y, point2_z))

print()

# Jinja2 template for the monitor instance
env = Environment(loader=FileSystemLoader('templates'))

template = env.get_template('src/distances.cpp.jinja2')

p1 = {'x': point1_x, 'y': point1_y, 'z': point1_z}
p2 = {'x': point2_x, 'y': point2_y, 'z': point2_z}

# Render the template
result = template.render(p1=p1, p2=p2)

# create a file and write the result to it
dest = 'ws/'

# check how many folders are in the path
folder_names = [name for name in os.listdir(dest) if os.path.isdir(os.path.join(dest, name))]
# check if empty
if not folder_names:
    new_folder = dest + 'test_1/'
else:
    folder_names.sort()
    last_folder = folder_names[-1]
    new_folder = dest + 'test_' + str(int(last_folder.split('_')[-1]) + 1) + '/'

# get the path from the template file name
path = template.name.split('/')[0:-1]
path = '/'.join(path)

# create the new folder
os.makedirs(new_folder + path, exist_ok=True)

with open(new_folder + template.name.strip('.jinja2'), 'w+') as f:
    f.write(result)