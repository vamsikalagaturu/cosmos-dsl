"""
Author: Vamsi Kalagaturu

Description:
This script converts the motion specifications to C++ code.
"""

import os
import argparse
import json
import rdflib
from rdflib.namespace import RDF
from rdflib.collection import Collection

from jinja2 import Environment, FileSystemLoader

from modelling_tools.utils.utils import Utils
from modelling_tools.utils.query_utils import QueryUtils


class Convert:
    def __init__(self, debug=True, no_render=False, debug_graph=False):
        self.debug = debug
        self.no_render = no_render
        self.debug_graph = debug_graph

        # get the package root path

        root_path = os.path.join(os.path.dirname(__file__), '../../')
        ws_path = os.path.join(os.path.dirname(__file__), '../../..')

        self.utils = Utils(ws_path)

        # relative paths to models
        self.models = f'{root_path}/cosmos_dsl/models/tasks/'

        self.templates_env = Environment(
            loader=FileSystemLoader(f'{root_path}/modelling_tools/templates'))

        # read the model names from the models folder
        self.model_names = []
        for file in os.listdir(self.models):
            if file.endswith(".jsonld"):
                self.model_names.append(file)

    def parse_models(self, model_names: list) -> rdflib.ConjunctiveGraph:
        # Create an RDF graph
        g = rdflib.ConjunctiveGraph()
        for model_name in model_names:
            g.parse(self.models + model_name, format='json-ld')

        return g

    def main(self):

        print("Parsing models...")
        g = self.parse_models(convert.model_names)

        if self.debug_graph:
            print(g.serialize(format='turtle'))
            print("-"*20)

        data = self._get_tasks(g)

        if self.debug:
            print(json.dumps(data, indent=2))
            print("-"*20)

        if not self.no_render:

            template = self.templates_env.get_template(
                'src/move_arm_template.cpp.jinja2')

            print("Rendering templates...")
            for task in data["tasks"]:
                result = template.render({
                    "task_spec": data["tasks"][task],
                    "coords": data["coords"],
                    "constraints": data["constraints"],
                    "controllers": data["controllers"],
                    "task_name": task
                })

                self.utils.write_to_file(result, task_name=task)

    def _update_coords_data(self, cond, big_data):
        ns = "http://example.com/rob#"

        coord = cond['coord']

        if coord not in big_data['coords']:
            coord_data = self.query_utils.get_coord_info(coord)

            assert coord_data, f"coord_data is empty for {coord}"

            big_data['coords'][coord] = coord_data

            if coord_data['type'] == 'DistanceCoordinate':
                f1_coord_data = self.query_utils.get_coord_info(
                    coord_data
                    ['f1_coord'])
                f2_coord_data = self.query_utils.get_coord_info(
                    coord_data
                    ['f2_coord'])

                big_data['coords'][coord_data['f1_coord']] = f1_coord_data
                big_data['coords'][coord_data['f2_coord']] = f2_coord_data

            elif coord_data['type'] == 'VelocityCoordinate':
                of_coord_data = self.query_utils.get_coord_info(
                    coord_data['of_coord'])

                wrt_coord_data = self.query_utils.get_coord_info(
                    coord_data['wrt_coord'])

                asb_coord_data = self.query_utils.get_coord_info(
                    coord_data['asb_coord'])

                big_data['coords'][coord_data['of_coord']] = of_coord_data
                big_data['coords'][coord_data['wrt_coord']] = wrt_coord_data
                big_data['coords'][coord_data['asb_coord']] = asb_coord_data

                big_data['coords'][coord_data['of_coord']+'_twist'] = {
                    'type': ['TwistCoordinate']}

            elif coord_data['type'] == 'WrenchCoordinate':
                if coord_data['ab_coord'].replace(
                        ns, '') not in big_data['coords']:
                    ab_coord_data = self.query_utils.get_coord_info(
                        coord_data['ab_coord'])
                    big_data['coords'][coord_data['ab_coord']] = ab_coord_data

                # if coord_data['at_coord'].replace(ns, '') not in big_data['coords']:
                #     at_coord_data = self.query_utils.get_coord_info(
                #         coord_data['at_coord'])
                #     big_data['coords'][coord_data['at_coord']] = at_coord_data

                if coord_data['asb_coord'].replace(
                        ns, '') not in big_data['coords']:
                    asb_coord_data = self.query_utils.get_coord_info(
                        coord_data['asb_coord'])
                    big_data['coords'][coord_data['asb_coord']] = asb_coord_data

            else:
                raise Exception(f"Unknown coordinate type {coord_data['type']}")

        return big_data

    def _get_tasks(self, g: rdflib.ConjunctiveGraph):

        ROB = "http://example.com"
        BASETASK = rdflib.Namespace(ROB + "/base_task#")
        MOTIONSPEC = rdflib.Namespace(ROB + "/move_arm#")

        self.query_utils = QueryUtils(g)

        big_data = {}
        big_data['coords'] = {}
        big_data['constraints'] = {}
        big_data['controllers'] = {}
        big_data['tasks'] = {}

        # get task specs
        for task_spec in g.subjects(RDF.type, BASETASK["BaseTask"]):
            data = {}

            # check if q-init is present
            q_init_iri = g.value(task_spec, BASETASK["q-init"])

            if q_init_iri:
                qi = Collection(g, q_init_iri)
                data["q_init"] = [float(q) for q in qi]
            else:
                data["q_init"] = [0.0, 0.0, 0.0, 1.57, 0.0, 1.57, 0.0]

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
                        big_data = self._update_coords_data(
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
                        big_data = self._update_coords_data(
                            ci[pcr['constraint']], big_data)

                    motion_spec['post_conditions'] = post_conditions_d

                    alphas = []
                    per_conditions_d = []
                    for per_condition in per_conditions:
                        pcr, ci = self.query_utils.get_per_condition_info(
                            per_condition)
                        per_conditions_d.append(pcr)

                        big_data["constraints"][pcr['constraint']
                                                ] = ci[pcr['constraint']]

                        big_data = self._update_coords_data(
                            ci[pcr['constraint']], big_data)

                        if pcr['type'] != 'ForceConstraint':
                            cont_info = self.query_utils.get_pid_controller_info(
                                pcr['controller'])
                            big_data["controllers"][
                                pcr['controller']] = cont_info

                        alpha = self.query_utils.get_alpha(pcr['constraint'])

                        if alpha is None:
                            print(f"Alpha is None for {pcr['constraint']}")
                        elif isinstance(alpha[0], list):
                            for a in alpha:
                                alphas.append(a)
                        elif isinstance(alpha[0], float):
                            alphas.append(alpha)
                        else:
                            raise Exception(
                                f"Unknown alpha type {type(alpha)}")

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
                else:
                    raise Exception(
                        f"Motion specification {motion_spec_iri} is not supported")

            data['motion_specs'] = motion_specs
            big_data['tasks'][str(task_spec).replace(ROB+"/rob#", '')] = data

        return big_data


if __name__ == '__main__':
    # get -d flag from command line
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Print debug information')
    parser.add_argument('-dg', '--debug-graph', action='store_true',
                        help='Print the RDF graph')
    parser.add_argument('-nr', '--no-render', action='store_true',
                        help='Do not render the result to a file')
    args = parser.parse_args()

    # create a new instance of the class
    convert = Convert(args.debug, args.no_render, args.debug_graph)
    convert.main()
