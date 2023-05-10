# Author: Sven Schneider

import resolver
import rdflib
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


if __name__ == "__main__":
    METAMODELS = "https://example.com/metamodels/entities/"
    MODELS = "https://example.com/instances/"

    url_map = {
        METAMODELS: "metamodels/entities/",
        MODELS: "instances/"
    }
    resolver.install(resolver.IriToFileResolver(url_map))

    g = rdflib.Graph()
    g.parse(MODELS + "point_instance.jsonld", format="json-ld")

    print(g.serialize(format="turtle"))
