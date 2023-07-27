#!/usr/bin/env python3

import os
import sys
import argparse
import rdflib
from rdflib.namespace import OWL, RDF, RDFS, XSD

from jinja2 import Environment, FileSystemLoader

from dsl.scripts.utils.utils import Utils

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class Convert:
    def __init__(self, debug=True):
        self.debug = debug

        self.utils = Utils()

        # relative paths to models
        root_path = os.path.join(os.path.dirname(__file__), '..')
        self.models = f'{root_path}/models/'
        
        self.templates_env = Environment(loader=FileSystemLoader(f'{root_path}/templates'))

        self.model_names = ['points', 'frames', 'position', 'position_coord', 'distances', 'distance_coord', 'monitors']

    def parse_models(self, model_names: list) -> rdflib.ConjunctiveGraph:
        # Create an RDF graph and parse the monitor instance file
        g = rdflib.ConjunctiveGraph()
        for model_name in model_names:
            g.parse(self.models + model_name + '.jsonld', format='json-ld')

        return g
    
    def print_graph(self):
        g = self.parse_models(convert.model_names)

        # print the graph
        print(g.serialize(format='turtle'))

        print("-"*20)

    def main(self):

        g = self.parse_models(convert.model_names)

        # print the graph
        if self.debug:
            self.print_graph()

        ROB = "http://example.com"
        POINT = rdflib.Namespace(ROB + "/point#")
        FRAME = rdflib.Namespace(ROB + "/frame#")
        SPATREL = rdflib.Namespace(ROB + "/spatial_relations#")
        COORD = rdflib.Namespace(ROB + "/coordinate#")
        DIST = rdflib.Namespace(ROB + "/distance#")
        MONITOR = rdflib.Namespace(ROB + "/monitor#")
        QUDT = rdflib.Namespace("http://qudt.org/schema/qudt/")

        template = self.templates_env.get_template('src/monitors.cpp.jinja')

        mons = self.test()

        if self.debug:
            print(mons)

        result = template.render({
            "monitors": mons,
            "ns": ROB+"/rob#"
        })

        self.utils.write_to_file(result, template.name)

    def test(self):
        g = self.parse_models(convert.model_names)

        ROB = "http://example.com"
        POINT = rdflib.Namespace(ROB + "/point#")
        FRAME = rdflib.Namespace(ROB + "/frame#")
        SPATREL = rdflib.Namespace(ROB + "/spatial_relations#")
        COORD = rdflib.Namespace(ROB + "/coordinate#")
        DIST = rdflib.Namespace(ROB + "/distance#")
        DISTCOORD = rdflib.Namespace(ROB + "/distCoord#")
        MONITOR = rdflib.Namespace(ROB + "/monitor#")
        THRESHOLD = rdflib.Namespace(ROB + "/threshold#")
        QUDT = rdflib.Namespace("http://qudt.org/schema/qudt/")

        mons = []

        # get monitors
        for monitor in g.subjects(RDF.type, MONITOR["Monitor"]):
            mon = {}
            # if monitor is a distance monitor
            if (monitor, RDF.type, MONITOR["DistanceMonitor"]) in g:
                # get comparison operator
                comp_op = str(g.value(monitor, MONITOR["constraint"]))
                # get threshold
                threshold = g.value(monitor, MONITOR["threshold"])

                threshold_value = float(g.value(threshold, THRESHOLD["threshold-value"]))
                threshold_unit = str(g.value(threshold, QUDT["unit"]))

                mon['comp_op'] = comp_op
                mon['threshold_value'] = threshold_value
                mon['threshold_unit'] = threshold_unit
                mon['type'] = 'DistanceMonitor'
                mon['distance'] = {}

                # get distance IRI
                distance_iri = g.value(monitor, MONITOR["distance"])

                if (distance_iri, RDF.type, DISTCOORD["DistanceCoordinate"]) in g:

                    dist = g.value(distance_iri, DISTCOORD["of"])
                    unit = g.value(distance_iri, QUDT["unit"])

                    query = f"""
                        SELECT DISTINCT ?coord ?x ?y ?z ?unit ?asSeenBy ?of ?wrt
                        WHERE {{
                            ?dist a distance:EuclideanDistance ;
                                a distance:PointToPointDistance ;
                                distance:between-entities ?p1, ?p2 .
                            FILTER(?p1 != ?p2)
                            ?p1 a point:Point .
                            ?p2 a point:Point .
                            ?pos a spatrel:Position ;
                                spatrel:of ?of ;
                                spatrel:wrt ?wrt ;
                                ^coord:of-position ?coord .
                            ?coord a coord:PositionCoordinate, coord:VectorXYZ .

                            {{
                                ?pos spatrel:of | spatrel:wrt ?p2 .
                            }} UNION {{
                                ?pos spatrel:of | spatrel:wrt ?p1 .
                            }}

                            ?cood coord:as-seen-by ?asSeenBy .
                            
                            OPTIONAL {{
                                ?coord coord:x ?x ;
                                    coord:y ?y ;
                                    coord:z ?z ;
                                    qudt-schema:unit ?unit .
                            }}
                        }}
                    """

                    r = g.query(query, initBindings={'dist': dist})

                    # save each row in a variable
                    e1 = [i for i in r][0]
                    e2 = [i for i in r][1]
                    
                    if e1[5] != e2[5]:
                        print('Warning: as-seen-by frames are not same')
                        raise NotImplementedError('Handler for runtime data fetching not implemented yet')
                    
                    if e1[7] != e2[7]:
                        print('Warning: wrt points are not same')
                        raise NotImplementedError('Handler for runtime data fetching not implemented yet')
                    
                    xyz1 = [float(i) for i in e1[1:4]]
                    xyz2 = [float(i) for i in e2[1:4]]

                    mon['distance']['type'] = 'PointToPointDistance'

                    mon['distance']['xyz1'] = xyz1
                    mon['distance']['xyz2'] = xyz2

                    
                else:
                    raise NotImplementedError('Distance monitor not implemented yet')
            else:
                raise NotImplementedError('Monitor not implemented yet')
            
            mons.append(mon)
        
        return mons

if __name__ == '__main__':
    # get -d flag from command line
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true', help='Print debug information')
    args = parser.parse_args()

    # create a new instance of the class
    convert = Convert(args.debug)
    convert.main()