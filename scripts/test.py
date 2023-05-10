#!/usr/bin/env python3

import os
import rdflib
from rdflib.namespace import OWL, RDF, RDFS, XSD

from jinja2 import Environment, FileSystemLoader

from utils import Utils

class Convert:
    def __init__(self):
        self.utils = Utils()

        # relative paths to models
        self.models = 'models/'
        
        self.templates_env = Environment(loader=FileSystemLoader('templates'))

        self.model_names = ['points', 'frames', 'position', 'position_coord', 'distances', 'monitors']

    def parse_models(self, model_names: list) -> rdflib.ConjunctiveGraph:
        # Create an RDF graph and parse the monitor instance file
        g = rdflib.ConjunctiveGraph()
        for model_name in model_names:
            g.parse(self.models + model_name + '.jsonld', format='json-ld')

        return g

    def main(self):

        g = self.parse_models(convert.model_names)

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

        template = self.templates_env.get_template('src/monitors.cpp.jinja')

        result = template.render({
            "g": g,
            "RDF": RDF,
            'POINT': POINT,
            'FRAME': FRAME,
            'SPATREL': SPATREL,
            'COORD': COORD,
            'DIST': DIST,
            'MONITOR': MONITOR,
            'QUDT': QUDT,
        })

        self.utils.write_to_file(result, template.name)

    def test(self):
        g = self.parse_models(convert.model_names)

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

        # get monitors
        for monitor in g.subjects(RDF.type, MONITOR["Monitor"]):
            # if monitor is a distance monitor
            if (monitor, RDF.type, MONITOR["MonitorDistance"]) in g:
                # get comparison operator
                comp_op = g.value(monitor, MONITOR["comparision_operator"])
                # get threshold
                threshold = g.value(monitor, MONITOR["threshold"])
                print(threshold)
                # get distance IRI
                distance_iri = g.value(monitor, MONITOR["distance"])
                
                if (distance_iri, RDF.type, DIST["CoordPositionToPosition"]) in g:
                    from_pos_coord = g.value(distance_iri, DIST["from-position-coord"])
                    to_pos_coord = g.value(distance_iri, DIST["to-position-coord"])

                    # get ref-frames
                    from_pos_coord_f = g.value(from_pos_coord, COORD["as-seen-by"])
                    to_pos_coord_f = g.value(to_pos_coord, COORD["as-seen-by"])

                    # get units
                    from_pos_coord_u = g.value(from_pos_coord, QUDT["unit"]).split("/")[-1]
                    
                    # TODO: get the value of the unit

                    to_pos_coord_u = g.value(to_pos_coord, QUDT["unit"]).split("/")[-1]

                    if not from_pos_coord_u == to_pos_coord_u:
                        # convert to same units
                        raise NotImplementedError

                    if not from_pos_coord_f == to_pos_coord_f:
                        # transform into same frame
                        raise NotImplementedError
                    
                    # get coordinates
                    from_pos_coord_x = g.value(from_pos_coord, COORD["x"])
                    from_pos_coord_y = g.value(from_pos_coord, COORD["y"])
                    from_pos_coord_z = g.value(from_pos_coord, COORD["z"])

                    to_pos_coord_x = g.value(to_pos_coord, COORD["x"])
                    to_pos_coord_y = g.value(to_pos_coord, COORD["y"])
                    to_pos_coord_z = g.value(to_pos_coord, COORD["z"])

                else:
                    raise NotImplementedError
                
            else:
                raise NotImplementedError
                

if __name__ == '__main__':
    convert = Convert()
    convert.main()
    # convert.test()