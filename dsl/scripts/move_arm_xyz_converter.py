#!/usr/bin/env python3

import os
import sys
import argparse
import json
import rdflib
from rdflib.namespace import RDF
from rdflib import Graph, URIRef
from rdflib.collection import Collection

from jinja2 import Environment, FileSystemLoader

from utils import Utils
from query_utils import QueryUtils

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class Convert:
    def __init__(self, debug=True):
        self.debug = debug

        self.utils = Utils()

        # relative paths to models
        root_path = os.path.join(os.path.dirname(__file__), '..')
        self.models = f'{root_path}/models/tasks/xyz_motion/'

        self.templates_env = Environment(
            loader=FileSystemLoader(f'{root_path}/templates'))

        # read the model names from the models folder
        self.model_names = []
        for file in os.listdir(self.models):
            if file.endswith(".jsonld"):
                self.model_names.append(file)

    def parse_models(self, model_names: list) -> rdflib.ConjunctiveGraph:
        # Create an RDF graph and parse the monitor instance file
        g = rdflib.ConjunctiveGraph()
        for model_name in model_names:
            g.parse(self.models + model_name, format='json-ld')

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

        # template = self.templates_env.get_template(
        #     'src/move_xyz_template.cpp.jinja')

        data = self.test(g)

        if True or self.debug:
            print(json.dumps(data, indent=2))
        
        ns = "http://example.com/rob#"

        # result = template.render({
        #     "data": data,
        #     "ns": ROB+"/rob#"
        # })

        # self.utils.write_to_file(result, template.name)

    def update_coords_data(self, cond, big_data):
        coord = cond['constraint']['coord']
                        
        if coord not in big_data['coords']:
            coord_data = self.query_utils.get_coord_info(coord)
            big_data['coords'][coord] = coord_data

            if coord_data['type'] == 'DistanceCoordinate' or \
                coord_data['type'] == 'VelocityCoordinate':
                
                f1_coord_data = self.query_utils.get_coord_info(coord_data['f1_coord'])
                f2_coord_data = self.query_utils.get_coord_info(coord_data['f2_coord'])

                big_data['coords'][coord_data['f1_coord']] = f1_coord_data
                big_data['coords'][coord_data['f2_coord']] = f2_coord_data

        return big_data

    def test(self, g: rdflib.ConjunctiveGraph):

        ROB = "http://example.com"
        POINT = rdflib.Namespace(ROB + "/point#")
        FRAME = rdflib.Namespace(ROB + "/frame#")
        SPATREL = rdflib.Namespace(ROB + "/spatial_relations#")
        COORD = rdflib.Namespace(ROB + "/coordinate#")
        DIST = rdflib.Namespace(ROB + "/distance#")
        DISTCOORD = rdflib.Namespace(ROB + "/dist_coord#")
        MONITOR = rdflib.Namespace(ROB + "/monitor#")
        CONSTRAINT = rdflib.Namespace(ROB + "/constraint-ns#")
        THRESHOLD = rdflib.Namespace(ROB + "/threshold#")
        QUDT = rdflib.Namespace("http://qudt.org/schema/qudt/")
        BASETASK = rdflib.Namespace(ROB + "/base_task#")
        MoveArmXYZ = rdflib.Namespace(ROB + "/move_arm_xyz#")
        MOTIONSPEC = rdflib.Namespace(ROB + "/move_arm_xyz#")
        CASCADEDCONTRL = rdflib.Namespace(ROB + "/cascaded_controller#")
        PIDCONTROLLER = rdflib.Namespace(ROB + "/pid_controller#")
        VERESHSOLVER = rdflib.Namespace(ROB + "/vereshchagin_solver#")
        KINEMATICS = rdflib.Namespace(ROB + "/kinematics#")
        MAPPINGS = rdflib.Namespace(ROB + "/mappings#")

        self.query_utils = QueryUtils(g)

        big_data = {}
        big_data['coords'] = {}
        big_data['tasks'] = {}

        # get monitors
        for task_spec in g.subjects(RDF.type, BASETASK["BaseTask"]):
            data = {}

            # get task_spec data
            motion_specs_iris = g.objects(
                task_spec, BASETASK["motion-specifications"])

            # get motion specifications data
            motion_specs = {}

            for motion_spec_iri in motion_specs_iris:
                
                if (motion_spec_iri, RDF.type, MOTIONSPEC["MoveArmXYZ"]) in g:

                    motion_spec = {}
                    # get the motion-specification node
                    mappings_iri = g.value(
                        motion_spec_iri, MOTIONSPEC["mappings"])
                    pre_conditions = g.objects(motion_spec_iri, MOTIONSPEC["pre-conditions"])
                    per_conditions = g.objects(motion_spec_iri, MOTIONSPEC["per-conditions"])
                    post_conditions = g.objects(motion_spec_iri, MOTIONSPEC["post-conditions"])

                    pre_conditions_d = []
                    for pre_condition in pre_conditions:

                        pcr = self.query_utils.get_pre_post_condition_info(pre_condition, {'constraint': pre_condition})
                        big_data = self.update_coords_data(pcr, big_data)
                        pre_conditions_d.append(pcr)

                    motion_spec['pre_conditions'] = pre_conditions_d

                    post_conditions_d = []
                    for post_condition in post_conditions:
                        
                        pcr = self.query_utils.get_pre_post_condition_info(post_condition, {'constraint': post_condition})
                        post_conditions_d.append(pcr)
                        big_data = self.update_coords_data(pcr, big_data)

                    motion_spec['post_conditions'] = post_conditions_d

                    per_conditions_d = []
                    for per_condition in per_conditions:

                        pcr = self.query_utils.get_per_condition_info(per_condition, {'constraint': per_condition})
                        per_conditions_d.append(pcr)
                        big_data = self.update_coords_data(pcr, big_data)

                    motion_spec['per_conditions'] = per_conditions_d

                    # get the mappings data
                    mapping_info = self.query_utils.get_mappings_info(mappings_iri)
                    motion_spec['mappings'] = mapping_info

                    motion_specs[motion_spec_iri] = motion_spec
                    
            data['motion_specs'] = motion_specs
            big_data['tasks'][task_spec] = data

        return big_data


if __name__ == '__main__':
    # get -d flag from command line
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Print debug information')
    args = parser.parse_args()

    # create a new instance of the class
    convert = Convert(args.debug)
    convert.main()
    # convert.print_graph()
