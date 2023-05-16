# Author: Sven Schneider

import resolver
import rdflib
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


if __name__ == "__main__":
    METAMODELS = "https://example.com/point#"
    MODELS = "https://example.com/rob#"

    url_map = {
        METAMODELS: "metamodels/entities/",
        MODELS: "models/"
    }
    resolver.install(resolver.IriToFileResolver(url_map))

    g = rdflib.ConjunctiveGraph()
    g.parse("models/points.jsonld", format="json-ld")

    print(g.serialize(format="turtle"))
