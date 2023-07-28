from rdflib import RDF, Graph, URIRef
from rdflib.collection import Collection
import rdflib


class QueryUtils:

    ROB = "http://example.com"
    KINEMATICS = rdflib.Namespace(ROB + "/kinematics#")
    COORD = rdflib.Namespace(ROB + "/coordinate#")
    CONSTRAINT = rdflib.Namespace(ROB + "/constraint#")
    MAPPINGS = rdflib.Namespace(ROB + "/mappings#")
    CONTROLLERMAPPING = rdflib.Namespace(ROB + "/controller_mapping#")
    CONTROLLERIO = rdflib.Namespace(ROB + "/controller_io#")

    def __init__(self, graph: Graph):
        self.graph = graph
        self.ns = QueryUtils.ROB + "/rob#"

    def get_pre_post_condition_info(self, pre_condition, init_bindings=None):
        """
        Returns the pre-condition of a given constraint.
        """
        query = f"""
            SELECT DISTINCT ?monitor ?constraint ?coord
            WHERE {{
                ?monitor a monitor:Monitor ;
                    monitor:constraint ?constraint .
                FILTER (?constraint = <{pre_condition}>)
                ?constraint a constraint-ns:Constraint ;
                    constraint-ns:coord ?coord .
            }}
        """

        qres = self.graph.query(query, initBindings=init_bindings)

        pre_condition_info = {}

        for row in qres:
            ci = self.get_constraint_info(init_bindings={'constraint': row[1]})
            pre_condition_info['monitor'] = str(row[0]).replace(self.ns, '')
            pre_condition_info['constraint'] = str(row[1]).replace(self.ns, '')

        return pre_condition_info, ci

    def get_per_condition_info(self, per_condition, init_bindings=None):
        """
        Returns the per-condition of a given constraint.
        """
        query = f"""
            SELECT DISTINCT ?controller ?constraint ?coord
            WHERE {{
                ?controller a pidController:PIDController ;
                    pidController:constraint ?constraint .
                FILTER (?constraint = <{per_condition}>)
                ?constraint a constraint-ns:Constraint ;
                    constraint-ns:coord ?coord .
            }}
        """

        # TODO: modify the init_bindings and percondition. its wrong way

        qres = self.graph.query(query, initBindings=init_bindings)

        per_condition_info = {}

        for row in qres:
            ci = self.get_constraint_info(init_bindings={'constraint': row[1]})
            per_condition_info['controller'] = str(row[0]).replace(self.ns, '')
            per_condition_info['constraint'] = str(row[1]).replace(self.ns, '')

        return per_condition_info, ci

    def get_constraint_info(self, init_bindings=None):
        """
        Returns a dictionary of constraint information for a given set of bindings.
        """

        query = f"""
            SELECT ?constraint ?operator ?tv ?tu ?coord
            WHERE {{
                ?constraint a constraint-ns:Constraint ;
                    constraint-ns:operator ?operator ;
                    constraint-ns:coord ?coord ;
                    constraint-ns:threshold ?threshold .
                ?threshold a thresh:Threshold ;
                    thresh:threshold-value ?tv ;
                    qudt-schema:unit ?tu .
            }}
        """

        qres = self.graph.query(query, initBindings=init_bindings)

        constraint = {}

        for row in qres:
            constraint_info = {}
            constraint_info['operator'] = str(row[1]).replace(self.ns, '')
            constraint_info['thresh_val'] = float(row[2])
            constraint_info['thresh_unit'] = str(row[3])
            constraint_info['coord'] = str(row[4]).replace(self.ns, '')

            constraint[str(row[0]).replace(self.ns, '')] = constraint_info

        return constraint

    def get_coord_info(self, coord, init_bindings=None):
        """
        Returns a dictionary of coordinate information for a given set of bindings.
        """
        coord_info = {}

        if (self.ns not in str(coord)):
            coord = URIRef(self.ns + coord)

        # check coord type is distance coordinate
        if (coord, RDF.type, QueryUtils.COORD["DistanceCoordinate"]) in self.graph:

            query = f"""
                SELECT ?dist_coord ?du ?qk ?f1_coord ?f2_coord
                WHERE {{
                    ?dist_coord a coord:DistanceCoordinate ;
                        coord:of-distance ?dist ;
                        qudt-schema:unit ?du .
                    ?dist a kinematics:EuclideanDistance ;
                        a kinematics:FramePointToPointDistance ;
                        qudt-schema:hasQuantityKind ?qk ;
                        kinematics:between-entities ?f1, ?f2 .
                    FILTER (?f1 != ?f2)
                    ?f1 a frame:Frame .
                    ?f2 a frame:Frame .
                    ?f1_coord a coord:FrameCoordinate ;
                        a coord:FrameReference ;
                        coord:of-frame ?f1 .
                    ?f2_coord a coord:FrameCoordinate ;
                        a coord:FrameReference ;
                        coord:of-frame ?f2 .
                }}
            """

            if init_bindings is None:
                init_bindings = {'dist_coord': coord}

            qres = self.graph.query(query, initBindings=init_bindings)

            for row in qres:
                coord_info = {
                    'type': 'DistanceCoordinate',
                    'unit': str(row[1]),
                    'quant_kind': row[2],
                    'f1_coord': str(row[3]).replace(self.ns, ''),
                    'f2_coord': str(row[4]).replace(self.ns, '')
                }

        # check coord type is velocity coordinate
        elif (coord, RDF.type, QueryUtils.COORD["VelocityCoordinate"]) in self.graph:
            query = f"""
                SELECT DISTINCT ?vel_coord ?vu ?qk ?f1_coord ?f2_coord ?f3_coord ?vel ?vel_type
                WHERE {{
                    ?vel_coord a coord:VelocityCoordinate ;
                        coord:of-velocity ?vel ;
                        qudt-schema:unit ?vu .
                    {{
                        ?vel a kinematics:CartesianVelocity ;
                            a ?vel_type ;
                            qudt-schema:hasQuantityKind ?qk ;
                            kinematics:of-frame ?f1 ;
                            kinematics:wrt-frame ?f2 ;
                            kinematics:measured-in-frame ?f3 .
                    }}
                    UNION
                    {{
                        ?vel a kinematics:CartesianVelocity ;
                            a ?vel_type ;
                            qudt-schema:hasQuantityKind ?qk ;
                            kinematics:of-frame ?f1 ;
                            kinematics:wrt-frame ?f2 ;
                            kinematics:measured-in-frame ?f3 .
                    }}
                    VALUES (?vel_type) {{ (kinematics:LinearVelocity) (kinematics:OneDimensionalVelocity) }}
                    FILTER (?f1 != ?f2)
                    ?f1 a frame:Frame .
                    ?f2 a frame:Frame .
                    ?f1_coord a coord:FrameCoordinate ;
                        a coord:FrameReference ;
                        coord:of-frame ?f1 .
                    ?f2_coord a coord:FrameCoordinate ;
                        a coord:FrameReference ;
                        coord:of-frame ?f2 .
                    ?f3_coord a coord:FrameCoordinate ;
                        a coord:FrameReference ;
                        coord:of-frame ?f3 .
                }}
            """

            if init_bindings is None:
                init_bindings = {'vel_coord': coord}

            qres = self.graph.query(query, initBindings=init_bindings)

            for row in qres:
                dim_node = self.graph.value(row[6], QueryUtils.KINEMATICS["dimension"])
                dim = Collection(self.graph, dim_node)
                dim = [float(d) for d in dim]
                coord_info = {
                    'type': 'VelocityCoordinate',
                    'unit': str(row[1]),
                    'quant_kind': row[2],
                    'f1_coord': str(row[3]).replace(self.ns, ''),
                    'f2_coord': str(row[4]).replace(self.ns, ''),
                    'f3_coord': str(row[5]).replace(self.ns, ''),
                    'vel_type': str(row[7]).split('#')[1],
                    'vel_dim': dim
                }

        # check coord type is FrameCoordinate
        elif (coord, RDF.type, QueryUtils.COORD["FrameCoordinate"]) in self.graph:
            query = f"""
                SELECT ?f1 ?f2 ?unit ?x ?y ?z
                WHERE {{
                    ?frame_coord a coord:FrameCoordinate ;
                        a coord:FrameReference ;
                        coord:of-frame ?f1 ;
                        qudt-schema:unit ?unit .
                    ?f1 a frame:Frame .

                    OPTIONAL {{
                        ?frame_coord a coord:FrameReference ;
                            coord:as-seen-by ?f2 .
                    }}

                    OPTIONAL {{
                        ?frame_coord a coord:VectorXYZ ;
                            coord:x ?x ;
                            coord:y ?y ;
                            coord:z ?z ;
                    }}
                }}
            """

            if init_bindings is None:
                init_bindings = {'frame_coord': coord}

            qres = self.graph.query(query, initBindings=init_bindings)

            for row in qres:
                coord_info = {
                    'type': ['FrameCoordinate'],
                    'f1': str(row[0]).replace(self.ns, ''),
                    'f2': str(row[1]).replace(self.ns, ''),
                    'unit': str(row[2])
                }

                if row[3] is not None:
                    coord_info['type'].append('VectorXYZ')
                    coord_info['x'] = float(row[3])
                    coord_info['y'] = float(row[4])
                    coord_info['z'] = float(row[5])

        return coord_info

    def get_mappings_info(self, mappings_iri, init_bindings=None):
        """
        Returns a dictionary of mappings information for a given set of bindings.
        """
        mappings_info = {}

        query = f"""
            SELECT DISTINCT ?interface ?solver
            WHERE {{
                ?mappings a mapping:ControllerSolver ;
                    mapping:interface ?interface ;
                    mapping:solver ?solver .
                ?interface a mapping:AccEnegery .
                ?solver a vereshchaginSolver:VereshchaginSolver .
            }}
        """

        if init_bindings is None:
            init_bindings = {'mappings': mappings_iri}

        qres = self.graph.query(query, initBindings=init_bindings)

        for row in qres:
            mappings_info['interface'] = str(row[0]).replace(self.ns, '')
            mappings_info['solver'] = str(row[1]).replace(self.ns, '')

        solver_inputs = self.graph.objects(
            mappings_iri, QueryUtils.MAPPINGS["solver-input"])

        mappings_info['solver-input'] = []

        for solver_input in solver_inputs:
            if (solver_input, RDF.type, QueryUtils.CONTROLLERIO["ControllerOutput"]):
                mappings_info['solver-input'].append(str(solver_input).replace(self.ns, ''))

        # get controller mappings
        controller_mappings = self.graph.objects(
            mappings_iri, QueryUtils.MAPPINGS["controller-mappings"])

        controller_mappings_data = []

        for controller_mapping_iri in controller_mappings:
            if (controller_mapping_iri, RDF.type, QueryUtils.CONTROLLERMAPPING["ControllerMapping"]):
                controller = self.graph.value(
                    controller_mapping_iri, QueryUtils.CONTROLLERMAPPING
                    ["controller"])
                input_iri = self.graph.value(
                    controller_mapping_iri, QueryUtils.CONTROLLERMAPPING
                    ["input"])
                output_iri = self.graph.value(
                    controller_mapping_iri, QueryUtils.CONTROLLERMAPPING
                    ["output"])

                d = {}

                d['controller'] = str(controller).replace(self.ns, '')

                if (input_iri, RDF.type, QueryUtils.CONTROLLERIO["ControllerInput"]):
                    d['input'] = self.graph.value(
                        input_iri, QueryUtils.CONTROLLERIO["io"]).__str__().replace(self.ns, '')
                    cop_node = self.graph.value(
                        input_iri, QueryUtils.CONTROLLERIO["io-dimension"])
                    cop = Collection(self.graph, cop_node)
                    d['input-dimension'] = [int(d) for d in cop]

                if (output_iri, RDF.type, QueryUtils.CONTROLLERIO["ControllerOutput"]):
                    d['output'] = str(output_iri).replace(self.ns, '')
                    cop_node = self.graph.value(
                        output_iri, QueryUtils.CONTROLLERIO["io-dimension"])
                    cop = Collection(self.graph, cop_node)
                    d['output-dimension'] = [int(d) for d in cop]

                controller_mappings_data.append(d)

        mappings_info['controller-mappings'] = controller_mappings_data

        return mappings_info

    def get_pid_controller_info(self, controller, init_bindings=None):
        """
        Returns a dictionary of PID controller information for a given set of bindings.
        """
        pid_controller_info = {}

        if controller is not None:
            if (self.ns not in str(controller)):
                controller = URIRef(self.ns + controller)

        query = f"""
            SELECT DISTINCT ?kp ?ki ?kd ?dt ?constraint
            WHERE {{
                ?controller a pidController:PIDController ;
                    pidController:p-gain ?kp ;
                    pidController:i-gain ?ki ;
                    pidController:d-gain ?kd ;
                    pidController:time-step ?dt ;
                    pidController:constraint ?constraint .
                ?constraint a constraint-ns:Constraint .
            }}
        """

        if init_bindings is None:
            init_bindings = {'controller': controller}

        qres = self.graph.query(query, initBindings=init_bindings)

        for row in qres:
            pid_controller_info['kp'] = float(row[0])
            pid_controller_info['ki'] = float(row[1])
            pid_controller_info['kd'] = float(row[2])
            pid_controller_info['dt'] = float(row[3])
            pid_controller_info['constraint'] = str(row[4]).replace(self.ns, '')

        return pid_controller_info

    def get_alpha(self, constraint):
        """
        Returns a dictionary of alpha beta data for a given constraint.
        """

        if (self.ns not in str(constraint)):
            constraint = URIRef(self.ns + constraint)

        coord = self.graph.value(constraint,
                                 QueryUtils.CONSTRAINT["coord"])

        if (coord, RDF.type, QueryUtils.COORD["VelocityCoordinate"]) in self.graph:
            of_vel = self.graph.value(coord, QueryUtils.COORD["of-velocity"])

            if (of_vel, RDF.type, QueryUtils.KINEMATICS["LinearVelocity"]) in self.graph:
                alpha = [[1, 0, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0, 0],
                         [0, 0, 1, 0, 0, 0]]

                return alpha

            elif (of_vel, RDF.type, QueryUtils.KINEMATICS["OneDimensionalVelocity"]) in self.graph:
                dimension_node = self.graph.value(
                    of_vel, QueryUtils.KINEMATICS["dimension"])
                
                # TODO: if dimension is not present, throw an error
                # calculate alpha based on of-frame and wrt-frame in cpp
                # check if dimension is present
                if dimension_node is None:
                    raise Exception("Dimension is not present")
                
                dimension = Collection(self.graph, dimension_node)

                alpha = [float(d) for d in dimension]

                return alpha
        elif (coord, RDF.type, QueryUtils.COORD["DistanceCoordinate"]) in self.graph:
            of_dist = self.graph.value(coord, QueryUtils.COORD["of-distance"])

            if (of_dist, RDF.type, QueryUtils.KINEMATICS["EuclideanDistance"]) in self.graph:
                alpha = [[1, 0, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0, 0],
                         [0, 0, 1, 0, 0, 0]]

                return alpha
            
            else:
                raise Exception("Distance type is not supported")

    def construct_abc(self, alpha_in):
        """
        Construct the alpha beta matrices.
        """
        alpha = []
        beta = []
        nc = 0

        # if alpha is not a list of lists
        if isinstance(alpha_in[0], list):
            for a in alpha_in:
                # if a is not in alpha, add it
                if a not in alpha:
                    alpha.append(a)
        elif isinstance(alpha_in[0], float):
            alpha.append(alpha_in)

        # sort alpha
        alpha.sort(reverse=True)

        if not isinstance(alpha[0], list):
            raise Exception("Alpha is not a list of lists")

        nc = len(alpha)

        beta = [0.0 for i in range(nc)]

        # construct beta from alpha based on apperance of non-zero elements
        for i, a in enumerate(alpha):
            if a[2] != 0:
                beta[i] = 9.81

        return alpha, beta, nc
