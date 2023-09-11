import json
from rdflib import Graph, URIRef

# JSON-LD data
json_ld_data = '''
{
  "@context": {
    "robotConfig": "http://example.org/robotConfig#",
    "xsd": "http://www.w3.org/2001/XMLSchema#",
    "initial-state": {
      "@id": "robotConfig:initial-state",
      "@container": "@list"
    }
  },
  "@id": "rob:kinova_config",
  "@type": "robotConfig:RobotConfig",
  "initial-state": [0.0, 0.0, 0.0, -1.57, 0.0, -1.57, 0.0]
}
'''

# Parse JSON-LD data
g = Graph()
g.parse(data=json_ld_data, format='json-ld')

# Define the namespace prefix
robotConfig = URIRef("http://example.org/robotConfig#")

# Get the initial-state
initial_state = g.value(subject=URIRef("rob:kinova_config"), predicate=robotConfig["initial-state"])

# Print the initial-state
print(initial_state)