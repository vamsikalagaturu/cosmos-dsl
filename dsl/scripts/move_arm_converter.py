#!/usr/bin/env python3

import os
import sys
import argparse
import json
import rdflib
from rdflib.namespace import RDF

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
        # self.models = f'{root_path}/models/tasks/xyz_motion/'
        self.models = f'{root_path}/models/tasks/partial_spec/'

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

        template = self.templates_env.get_template(
            'src/move_arm_template.cpp.jinja2')

        data = self.test(g)

        if True or self.debug:
            print(json.dumps(data, indent=2))

        ns = "http://example.com/rob#"

        result = template.render({
            "motion_specs": data["tasks"]["move_arm_xyz_task"]["motion_specs"],
            "coords": data["coords"],
            "constraints": data["constraints"],
            "controllers": data["controllers"],
            "ns": ns
        })

        self.utils.write_to_file(result, template.name)

    def update_coords_data(self, cond, big_data):
        coord = cond['coord']

        if coord not in big_data['coords']:
            coord_data = self.query_utils.get_coord_info(coord)

            big_data['coords'][coord] = coord_data

            if coord_data['type'] == 'DistanceCoordinate' or \
                    coord_data['type'] == 'VelocityCoordinate':

                f1_coord_data = self.query_utils.get_coord_info(
                    coord_data
                    ['f1_coord'])
                f2_coord_data = self.query_utils.get_coord_info(
                    coord_data
                    ['f2_coord'])

                big_data['coords'][coord_data['f1_coord']] = f1_coord_data
                big_data['coords'][coord_data['f2_coord']] = f2_coord_data

        return big_data

    def test(self, g: rdflib.ConjunctiveGraph):

        ROB = "http://example.com"
        BASETASK = rdflib.Namespace(ROB + "/base_task#")
        MOTIONSPEC = rdflib.Namespace(ROB + "/move_arm#")

        self.query_utils = QueryUtils(g)

        big_data = {}
        big_data['coords'] = {}
        big_data['constraints'] = {}
        big_data['controllers'] = {}
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

                if (motion_spec_iri, RDF.type, MOTIONSPEC["MoveArm"]) in g:

                    motion_spec = {}

                    # get the motion-specification node
                    mappings_iri = g.value(
                        motion_spec_iri, MOTIONSPEC["mappings"])

                    # get the mappings data
                    mapping_info = self.query_utils.get_mappings_info(
                        mappings_iri)
                    motion_spec['mappings'] = mapping_info

                    pre_conditions = g.objects(
                        motion_spec_iri, MOTIONSPEC["pre-conditions"])
                    per_conditions = g.objects(
                        motion_spec_iri, MOTIONSPEC["per-conditions"])
                    post_conditions = g.objects(
                        motion_spec_iri, MOTIONSPEC["post-conditions"])

                    pre_conditions_d = []
                    for pre_condition in pre_conditions:

                        pcr, ci = self.query_utils.get_pre_post_condition_info(
                            pre_condition, {'constraint': pre_condition})
                        big_data["constraints"][pcr['constraint']
                                                ] = ci[pcr['constraint']]
                        big_data = self.update_coords_data(
                            ci[pcr['constraint']], big_data)
                        pre_conditions_d.append(pcr)

                    motion_spec['pre_conditions'] = pre_conditions_d

                    post_conditions_d = []
                    for post_condition in post_conditions:

                        pcr, ci = self.query_utils.get_pre_post_condition_info(
                            post_condition, {'constraint': post_condition})
                        post_conditions_d.append(pcr)
                        big_data["constraints"][pcr['constraint']
                                                ] = ci[pcr['constraint']]
                        big_data = self.update_coords_data(
                            ci[pcr['constraint']], big_data)

                    motion_spec['post_conditions'] = post_conditions_d

                    alphas = []

                    per_conditions_d = []
                    for per_condition in per_conditions:

                        pcr, ci = self.query_utils.get_per_condition_info(
                            per_condition, {'constraint': per_condition})
                        per_conditions_d.append(pcr)
                        big_data["constraints"][pcr['constraint']
                                                ] = ci[pcr['constraint']]
                        big_data = self.update_coords_data(
                            ci[pcr['constraint']], big_data)
                        cont_info = self.query_utils.get_pid_controller_info(
                            pcr['controller'])
                        big_data["controllers"][pcr['controller']] = cont_info
                        alpha = self.query_utils.get_alpha(pcr['constraint'])
                        if isinstance(alpha[0], list):
                            for a in alpha:
                                alphas.append(a)
                        else:
                            alphas.append(alpha)

                    motion_spec['per_conditions'] = per_conditions_d

                    # construct alpha, beta and nc
                    alpha, beta, nc = self.query_utils.construct_abc(alphas)

                    motion_spec['alpha_beta_nc'] = {
                        'alpha': alpha,
                        'beta': beta,
                        'nc': nc
                    }

                    motion_specs[str(motion_spec_iri).replace(
                        ROB + "/rob#", '')] = motion_spec

            data['motion_specs'] = motion_specs
            big_data['tasks'][str(task_spec).replace(ROB+"/rob#", '')] = data

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
