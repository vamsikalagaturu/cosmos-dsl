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
    
    def print_graph(self):
        g = self.parse_models(convert.model_names)

        # print the graph
        print(g.serialize(format='turtle'))

        print("-"*20)

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

        mons = self.test()

        print(mons)

        result = template.render({
            "monitors": mons,
        })

        self.utils.write_to_file(result, template.name)

    def get_mons(self) -> dict:
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

        mons = []

        # get monitors
        for monitor in g.subjects(RDF.type, MONITOR["Monitor"]):
            mon = {}
            # if monitor is a distance monitor
            if (monitor, RDF.type, MONITOR["MonitorDistance"]) in g:
                # get comparison operator
                comp_op = str(g.value(monitor, MONITOR["comparision_operator"]))
                # get threshold
                threshold = float(g.value(monitor, MONITOR["threshold"]))

                mon['comp_op'] = comp_op
                mon['threshold'] = threshold
                mon['type'] = 'MonitorDistance'
                mon['distance'] = {}

                # get distance IRI
                distance_iri = g.value(monitor, MONITOR["distance"])

                if (distance_iri, RDF.type, DIST["CoordPointToPoint"]) in g:
                    
                    mon['distance']['type'] = 'CoordPointToPoint'

                    from_point = g.value(distance_iri, DIST["from-point"])
                    to_point= g.value(distance_iri, DIST["to-point"])

                    # get spatrel:Position relations for both the points
                    query = f"""
                        SELECT ?spatrel
                        WHERE {{
                            {{
                                ?spatrel rdf:type spatrel:Position .
                                ?spatrel ?p rob:{rdflib.URIRef(from_point).fragment} .
                            }}
                            UNION
                            {{
                                ?spatrel rdf:type spatrel:Position .
                                ?spatrel ?p rob:{rdflib.URIRef(to_point).fragment} .
                            }}
                        }}
                    """

                    r = g.query(query)

                    # get the spatrel's
                    spatrels = [rdflib.URIRef(i[0]) for i in r]

                    # if r does not have 2 results, print warning
                    if len(r) == 2:

                        # check if the coord has x, y, z values
                        query = f"""
                            SELECT ?x ?y ?z
                            WHERE {{
                                ?coord rdf:type coord:PositionCoordinate .
                                ?coord ?c rob:{spatrels[0].fragment}  .
                                ?coord coord:x ?x .
                                ?coord coord:y ?y .
                                ?coord coord:z ?z .
                            }}
                        """

                        rc1 = g.query(query)

                        if not rc1:
                            print(f'Warning: The spatrel {spatrels[0]} does not have a coord')
                            raise NotImplementedError('Handler for runtime data fetching not implemented yet')
                        
                        query = f"""
                            SELECT ?x ?y ?z
                            WHERE {{
                                ?coord rdf:type coord:PositionCoordinate .
                                ?coord ?c rob:{spatrels[1].fragment}  .
                                ?coord coord:x ?x .
                                ?coord coord:y ?y .
                                ?coord coord:z ?z .
                            }}
                        """

                        rc2 = g.query(query)

                        if not rc2:
                            print(f'Warning: The spatrel {spatrels[1]} does not have a coord')
                            raise NotImplementedError('Handler for runtime data fetching not implemented yet')

                        # get the coordinates
                        coord1 = [i for i in rc1][0]
                        coord2 = [i for i in rc2][0]

                        # get the x, y, z values
                        xyz1 = [float(i) for i in coord1]
                        xyz2 = [float(i) for i in coord2]

                        mon['distance']['xyz1'] = xyz1
                        mon['distance']['xyz2'] = xyz2

                    else:
                        print(f'Warning: The points does not have spatrel:Position relations')
                else:
                    raise NotImplementedError('Distance monitor not implemented yet')
            else:
                raise NotImplementedError('Monitor not implemented yet')
            
            mons.append(mon)
        
        return mons

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

        mons = []

        # get monitors
        for monitor in g.subjects(RDF.type, MONITOR["Monitor"]):
            mon = {}
            # if monitor is a distance monitor
            if (monitor, RDF.type, MONITOR["DistanceMonitor"]) in g:
                # get comparison operator
                comp_op = str(g.value(monitor, MONITOR["comparision_operator"]))
                # get threshold
                threshold = float(g.value(monitor, MONITOR["threshold"]))

                mon['comp_op'] = comp_op
                mon['threshold'] = threshold
                mon['type'] = 'DistanceMonitor'
                mon['distance'] = {}

                # get distance IRI
                distance_iri = g.value(monitor, MONITOR["distance"])

                if (distance_iri, RDF.type, DIST["PointToPointDistance"]) in g:
                    
                    mon['distance']['type'] = 'PointToPointDistance'

                    query = f"""
                        SELECT DISTINCT ?coord ?x ?y ?z ?unit ?asSeenBy ?of ?wrt
                        WHERE {{
                            ?dist a distance:EuclideanDistance ;
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

                    r = g.query(query, initBindings={'dist': distance_iri})

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

                    mon['distance']['xyz1'] = xyz1
                    mon['distance']['xyz2'] = xyz2

                    
                else:
                    raise NotImplementedError('Distance monitor not implemented yet')
            else:
                raise NotImplementedError('Monitor not implemented yet')
            
            mons.append(mon)
        
        return mons

if __name__ == '__main__':
    convert = Convert()
    convert.main()
    # convert.print_graph()
    # convert.test()