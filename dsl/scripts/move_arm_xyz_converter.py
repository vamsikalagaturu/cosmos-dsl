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

        data = self.test()

        if self.debug:
            print(data)

        # result = template.render({
        #     "monitors": mons,
        #     "ns": ROB+"/rob#"
        # })

        # self.utils.write_to_file(result, template.name)

    def test(self):
        g = self.parse_models(convert.model_names)

        ROB = "http://example.com"
        POINT = rdflib.Namespace(ROB + "/point#")
        FRAME = rdflib.Namespace(ROB + "/frame#")
        SPATREL = rdflib.Namespace(ROB + "/spatial_relations#")
        COORD = rdflib.Namespace(ROB + "/coordinate#")
        DIST = rdflib.Namespace(ROB + "/distance#")
        DISTCOORD = rdflib.Namespace(ROB + "/dist_coord#")
        MONITOR = rdflib.Namespace(ROB + "/monitor#")
        CONSTRAINT = rdflib.Namespace(ROB + "/constraint#")
        THRESHOLD = rdflib.Namespace(ROB + "/threshold#")
        QUDT = rdflib.Namespace("http://qudt.org/schema/qudt/")
        BASETASK = rdflib.Namespace(ROB + "/base_task#")
        MoveArmXYZ = rdflib.Namespace(ROB + "/move_arm_xyz#")
        DEBUGCONFIG = rdflib.Namespace(ROB + "/debug_config#")
        ROBOTCONFIG = rdflib.Namespace(ROB + "/robot_config#")
        MOTIONSPEC = rdflib.Namespace(ROB + "/move_arm_xyz#")
        CASCADEDCONTRL = rdflib.Namespace(ROB + "/cascaded_controller#")
        PIDCONTROLLER = rdflib.Namespace(ROB + "/pid_controller#")
        VERESHSOLVER = rdflib.Namespace(ROB + "/vereshchagin_solver#")

        big_data = []

        # get monitors
        for task_spec in g.subjects(RDF.type, BASETASK["BaseTask"]):
            data = {}

            # get task_spec data
            debug_config_iri = g.value(task_spec, BASETASK["debug-config"])
            robot_config_iri = g.value(task_spec, BASETASK["robot-config"])
            motion_specs_iris = g.objects(
                task_spec, BASETASK["motion-specifications"])

            # get debug config data
            debug_config = {}
            if (debug_config_iri, RDF.type, DEBUGCONFIG["DebugConfig"]) in g:
                debug_config['log-terminal'] = g.value(
                    debug_config_iri, DEBUGCONFIG["log-terminal"]).toPython()
                debug_config['log-file'] = g.value(
                    debug_config_iri, DEBUGCONFIG["log-file"]).toPython()
                debug_config['plot-data'] = g.value(
                    debug_config_iri, DEBUGCONFIG["plot-data"]).toPython()
                debug_config['save-data'] = g.value(
                    debug_config_iri, DEBUGCONFIG["save-data"]).toPython()

            data['debug_config'] = debug_config

            # get robot config data
            robot_config = {}
            if (robot_config_iri, RDF.type, ROBOTCONFIG["RobotConfig"]) in g:
                robot_config['urdf-name'] = str(
                    g.value(robot_config_iri, ROBOTCONFIG["urdf-name"]))
                # get the inital-state node iri
                initial_state = g.value(
                    robot_config_iri, ROBOTCONFIG["initial-state"])
                # get the initial-state node data
                # loop through the initial-state list node and get the data
                robot_config['initial-state'] = [jv.toPython()
                                                 for jv in Collection(g, initial_state)]

            data['robot_config'] = robot_config

            # get motion specifications data
            motion_specs = {}

            for motion_spec_iri in motion_specs_iris:
                motion_spec = {}
                # get the motion-specification node
                control_frame_coord = g.value(
                    motion_spec_iri, MOTIONSPEC["control-frame-coord"])
                target_frame_coord = g.value(
                    motion_spec_iri, MOTIONSPEC["target-frame-coord"])
                time_step = g.value(motion_spec_iri, MOTIONSPEC["time-step"])
                controller = g.value(motion_spec_iri, MOTIONSPEC["controller"])
                solver = g.value(motion_spec_iri, MOTIONSPEC["solver"])
                iterations = g.value(motion_spec_iri, MOTIONSPEC["iterations"])
                monitors = g.objects(motion_spec_iri, MOTIONSPEC["monitors"])

                # get controller data
                controller_data = {}
                if (controller, RDF.type, CASCADEDCONTRL["CascadedController"]) in g:

                    controller_data['type'] = ["CascadedController", "PositionVelocityController"]

                    controller_data['data'] = {}

                    # get position controller iri
                    position_controller = g.value(
                        controller, CASCADEDCONTRL["position-controller"])
                    # get velocity controller iri
                    velocity_controller = g.value(
                        controller, CASCADEDCONTRL["velocity-controller"])

                    for controller_iri in [position_controller, velocity_controller]:
                        if (controller_iri, RDF.type, PIDCONTROLLER["PIDController"]) in g:
                            d = {}
                            d['name'] = controller_iri
                            d['type'] = "PIDController"

                            # get the proportional gain
                            d['p-gain'] = g.value(
                                controller_iri, PIDCONTROLLER["p-gain"]).toPython()
                            # get the integral gain
                            d['i-gain'] = g.value(
                                controller_iri, PIDCONTROLLER["i-gain"]).toPython()
                            # get the derivative gain
                            d['d-gain'] = g.value(
                                controller_iri, PIDCONTROLLER["d-gain"]).toPython()
                            # time step
                            d['time-step'] = g.value(
                                controller_iri, PIDCONTROLLER["time-step"]).toPython()
                            
                            controller_data['data'][controller_iri] = d

                motion_spec['controller'] = controller_data

                # get solver data
                solver_data = {}
                if (solver, RDF.type, VERESHSOLVER["VereshchaginSolver"]) in g:
                    solver_data['type'] = "VereshchaginSolver"
                    solver_data['data'] = {}
                    # get the alpha-constraints and beta-constraints if any
                    alpha_constraints = g.value(
                        solver, VERESHSOLVER["alpha-constraint"])
                    beta_constraints = g.value(
                        solver, VERESHSOLVER["beta-constraint"])
                    
                    if alpha_constraints is not None:
                        solver_data['data']['alpha-constraint'] = [jv.toPython() for jv in Collection(g, alpha_constraints)]
                    if beta_constraints is not None:
                        solver_data['data']['beta-constraint'] = [jv.toPython() for jv in Collection(g, beta_constraints)]
                    
                motion_spec['solver'] = solver_data

                # get control-frame-coord data
                control_frame_coord_data = {}
                if (control_frame_coord, RDF.type, COORD["FrameCoordinate"]) in g:
                    control_frame_coord_data['type'] = "FrameCoordinate"
                    control_frame_coord_data['of-frame'] = g.value(
                        control_frame_coord, COORD["of-frame"])
                    control_frame_coord_data['unit'] = g.value(
                        control_frame_coord, QUDT["unit"])
                    
                # get target-frame-coord data
                target_frame_coord_data = {}
                if (target_frame_coord, RDF.type, COORD["FrameCoordinate"]) in g:
                    target_frame_coord_data['type'] = "FrameCoordinate"
                    target_frame_coord_data['of-frame'] = g.value(
                        target_frame_coord, COORD["of-frame"])
                    # TODO: check if of-frame is of type frame and available in g
                    target_frame_coord_data['unit'] = g.value(
                        target_frame_coord, QUDT["unit"])
                    
                    # get x, y, z values if any
                    x = g.value(target_frame_coord, COORD["x"]).toPython()
                    y = g.value(target_frame_coord, COORD["y"]).toPython()
                    z = g.value(target_frame_coord, COORD["z"]).toPython()

                    if x is not None and y is not None and z is not None:
                        target_frame_coord_data['position'] = [x, y, z]

                # get monitors data
                monitors_data = []
                for monitor in monitors:
                    if (monitor, RDF.type, MONITOR["Monitor"]) in g:
                        constraint = g.value(monitor, MONITOR["constraint"])
                        if (constraint, RDF.type, CONSTRAINT["Constraint"]) in g \
                            and (constraint, RDF.type, CONSTRAINT["DistanceConstraint"]) in g:

                            monitor_data = {}
                            monitor_data['type'] = "DistanceConstraint"
                            monitor_data['data'] = {}
                            monitor_data['data']['operator'] = g.value(
                                constraint, CONSTRAINT["operator"])
                            threshold = g.value(
                                constraint, CONSTRAINT["threshold"])
                            if (threshold, RDF.type, THRESHOLD["Threshold"]) in g:
                                monitor_data['data']['threshold_value'] = \
                                    g.value(threshold, THRESHOLD["threshold-value"]).toPython()
                            dist_coord = g.value(
                                constraint, CONSTRAINT["distance-coord"])
                            
                            if (dist_coord, RDF.type, DISTCOORD["DistanceCoordinate"]) in g:
                                dist_coord_data = {}
                                dist_coord_data['type'] = "DistanceCoordinate"
                                dist_coord_data['unit'] = g.value(
                                    dist_coord, QUDT["unit"])
                                dist_coord_of = g.value(
                                    dist_coord, DISTCOORD["of"])
                                if (dist_coord_of, RDF.type, DIST["EuclideanDistance"]) in g \
                                    and (dist_coord_of, RDF.type, DIST["FramePointToPointDistance"]) in g:

                                    dist_coord_data['between-entities'] = list(g.objects(
                                        dist_coord_of, DIST["between-entities"]))
                                
                                monitor_data['data']['distance-coord'] = dist_coord_data
                            
                            monitors_data.append(monitor_data)
                
                motion_spec['control-frame-coord'] = control_frame_coord_data
                motion_spec['target-frame-coord'] = target_frame_coord_data
                motion_spec['time-step'] = time_step.toPython()
                motion_spec['iterations'] = iterations.toPython()
                motion_spec['monitors'] = monitors_data

                motion_specs[motion_spec_iri] = motion_spec

            data['motion_specs'] = motion_specs

            # print dictionary pretty
            print(json.dumps(data, indent=4))

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
