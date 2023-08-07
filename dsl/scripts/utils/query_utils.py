import time
from rdflib import RDF, Graph, URIRef
from rdflib.collection import Collection
import rdflib
import numpy as np


class QueryUtils:

    ROB = "http://example.com"
    KINEMATICS = rdflib.Namespace(ROB + "/kinematics#")
    DYNAMICS = rdflib.Namespace(ROB + "/dynamics#")
    COORD = rdflib.Namespace(ROB + "/coordinate#")
    CONSTRAINT = rdflib.Namespace(ROB + "/constraint#")
    MAPPINGS = rdflib.Namespace(ROB + "/mappings#")
    CONTROLLERMAPPING = rdflib.Namespace(ROB + "/controller_mapping#")
    CONTROLLERIO = rdflib.Namespace(ROB + "/controller_io#")
    QUDT = rdflib.Namespace("http://qudt.org/schema/qudt#")

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

        qb = qres.bindings

        assert len(
            qb) > 0, f'Pre-condition {pre_condition} is not properly defined'

        pre_condition_info = {}

        ci = self.get_constraint_info(
            init_bindings={'constraint': qb[0]['constraint']})
        pre_condition_info['monitor'] = str(
            qb[0]['monitor']).replace(
            self.ns, '')
        pre_condition_info['constraint'] = str(
            qb[0]['constraint']).replace(
            self.ns, '')

        return pre_condition_info, ci

    def get_per_condition_info(self, per_condition, init_bindings=None):
        """
        Returns the per-condition of a given constraint.
        """

        per_condition_info = {}
        ci = None

        qb = None

        # check constraint type
        if (per_condition, RDF.type, QueryUtils.CONSTRAINT["ForceConstraint"]) in self.graph:

            per_condition_info['type'] = 'ForceConstraint'

            query = f"""
                SELECT DISTINCT ?constraint ?coord
                WHERE {{
                    ?constraint a constraint-ns:Constraint ;
                        constraint-ns:coord ?coord .
                }}
                LIMIT 1
            """

            if init_bindings is None:
                init_bindings = {'constraint': per_condition}

            qres = self.graph.query(query, initBindings=init_bindings)

            qb = qres.bindings

            assert len(
                qb) > 0, f'Per-condition {per_condition} is not properly defined'

        else:
            per_condition_info['type'] = 'OTHER'

            query = f"""
                SELECT DISTINCT ?controller ?constraint ?coord
                WHERE {{
                    ?controller a pidController:PIDController ;
                        pidController:constraint ?constraint .
                    FILTER (?constraint = ?per_condition)
                    ?constraint a constraint-ns:Constraint ;
                        constraint-ns:coord ?coord .
                }}
            """

            if init_bindings is None:
                init_bindings = {'per_condition': per_condition}

            qres = self.graph.query(query, initBindings=init_bindings)

            qb = qres.bindings

            assert len(
                qb) > 0, f'Per-condition {per_condition} is not properly defined'

            per_condition_info['controller'] = str(
                qb[0]['controller']).replace(
                self.ns, '')

        ci = self.get_constraint_info(
            init_bindings={'constraint': qb[0]['constraint']})

        con_name = str(qb[0]['constraint']).replace(self.ns, '')

        assert ci[con_name]['operator'] == "eq", f"Operator {ci[con_name]['operator']} is not supported for per-condition"

        per_condition_info['constraint'] = str(
            qb[0]['constraint']).replace(
            self.ns, '')

        return per_condition_info, ci

    def get_constraint_info(self, init_bindings=None):
        """
        Returns a dictionary of constraint information for a given set of bindings.
        """

        query = f"""
            SELECT ?operator ?tv ?tu ?coord
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

        assert init_bindings is not None, "init_bindings cannot be None"

        qres = self.graph.query(query, initBindings=init_bindings)

        qb = qres.bindings

        constriant_name = str(qb[0]['constraint']).replace(self.ns, '')

        assert len(
            qb) > 0, f'Constraint {constriant_name} is not properly defined'

        constraint = {}

        constraint_info = {}
        constraint_info['operator'] = str(qb[0]['operator']).split('#')[1]
        constraint_info['thresh_val'] = float(qb[0]['tv'])
        constraint_info['thresh_unit'] = str(qb[0]['tu']).split('/')[-1]
        constraint_info['coord'] = str(qb[0]['coord']).replace(self.ns, '')

        constraint[constriant_name] = constraint_info

        return constraint

    def get_coord_info(self, coord, init_bindings=None):
        """
        Returns a dictionary of coordinate information for a given set of bindings.
        """

        assert coord is not None or init_bindings is not None, "Either coord or init_bindings should be provided"

        coord_info = {}

        if (self.ns not in str(coord)):
            coord = URIRef(self.ns + coord)

        # check coord type is distance coordinate
        if (coord, RDF.type, QueryUtils.COORD["DistanceCoordinate"]) in self.graph:

            query = f"""
                SELECT ?du ?qk ?f1_coord ?f2_coord ?dist_type_1 ?dist_type_2
                WHERE {{
                    ?dist_coord a coord:DistanceCoordinate ;
                        coord:of-distance ?dist ;
                        qudt-schema:unit ?du .
                    ?dist a kinematics:EuclideanDistance ;
                        a kinematics:FrameToFrameDistance ;
                        a ?type_1 ;
                        a ?type_2 ;
                        qudt-schema:hasQuantityKind ?qk ;
                        kinematics:between-entities ?f1, ?f2 .
                    VALUES ?type_1 {{ kinematics:LinearDistance kinematics:AngularDistance }}
                    VALUES ?type_2 {{ kinematics:1D kinematics:2D kinematics:3D }}
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

            qb = qres.bindings

            assert len(
                qb) > 0, f"Distance coord {coord} is not properly defined"

            coord_info = {
                'type': 'DistanceCoordinate',
                'unit': str(qb[0]['du']).split('/')[-1],
                'quant_kind': str(qb[0]['qk']),
                'f1_coord': str(qb[0]['f1_coord']).replace(self.ns, ''),
                'f2_coord': str(qb[0]['f2_coord']).replace(self.ns, '')
            }

        # check coord type is velocity coordinate
        elif (coord, RDF.type, QueryUtils.COORD["VelocityCoordinate"]) in self.graph:

            vel = self.graph.value(coord, QueryUtils.COORD["of-velocity"])

            # check velocity type: CartesianVelocity, OneDimensionalVelocity, TwoDimensionalVelocity
            if (vel, RDF.type, QueryUtils.KINEMATICS["CartesianVelocity"]) in self.graph:
                # check velocity type: LinearVelocity, AngularVelocity
                vel_type = ['CartesianVelocity']
                if (vel, RDF.type, QueryUtils.KINEMATICS["LinearVelocity"]) in self.graph:
                    vel_type.append('LinearVelocity')
                elif (vel, RDF.type, QueryUtils.KINEMATICS["AngularVelocity"]) in self.graph:
                    vel_type.append('AngularVelocity')
                else:
                    raise Exception("Velocity type is not supported")

                of_frame = self.graph.value(
                    vel, QueryUtils.KINEMATICS["of-frame"])
                wrt_frame = self.graph.value(
                    vel, QueryUtils.KINEMATICS["wrt-frame"])

                query = f"""
                    SELECT DISTINCT ?of_coord ?wrt_coord
                    WHERE {{
                        ?of_coord a coord:FrameCoordinate ;
                            a coord:FrameReference ;
                            coord:of-frame ?of_frame .
                        ?wrt_coord a coord:FrameCoordinate ;
                            a coord:FrameReference ;
                            coord:of-frame ?wrt_frame .
                    }}
                """

                init_bindings = {'of_frame': of_frame, 'wrt_frame': wrt_frame}

                qres = self.graph.query(query, initBindings=init_bindings)

                qb = qres.bindings

                assert len(qb[0]) > 0, "Velocity corrds are not properly defined"

                of_coord = qb[0]['of_coord']
                wrt_coord = qb[0]['wrt_coord']

                qk = self.graph.value(vel, QueryUtils.QUDT["hasQuantityKind"])

                # check if of-frame and wrt-frame are different
                if of_frame == wrt_frame:
                    raise Exception("of-frame and wrt-frame cannot be the same")

                query = f"""
                    SELECT DISTINCT ?asb_coord ?vu ?v_x ?v_y ?v_z
                    WHERE {{
                        ?vel_coord a coord:VelocityCoordinate ;
                            a coord:VectorXYZ ;
                            coord:of-velocity ?vel ;
                            coord:as-seen-by ?f3 ;
                            qudt-schema:unit ?vu ;
                            coord:x ?v_x ;
                            coord:y ?v_y ;
                            coord:z ?v_z .
                        
                        ?f3 a frame:Frame .

                        ?asb_coord a coord:FrameCoordinate ;
                            a coord:FrameReference ;
                            coord:of-frame ?f3 .
                    }}
                """

                init_bindings = {'vel': vel, 'vel_coord': coord}

                qres = self.graph.query(query, initBindings=init_bindings)

                vals = qres.bindings

                assert len(
                    vals) > 0, f"Velocity coord {coord} is not properly defined"

                vu = vals[0]['vu']
                asb_coord = vals[0]['asb_coord']

                coord_info = {
                    'type': 'VelocityCoordinate',
                    'of_coord': str(of_coord).replace(self.ns, ''),
                    'wrt_coord': str(wrt_coord).replace(self.ns, ''),
                    'asb_coord': str(asb_coord).replace(self.ns, ''),
                    'vel_sp': [float(vals[0]['v_x']), float(vals[0]['v_y']), float(vals[0]['v_z'])],
                    'unit': str(vu).split('/')[-1],
                    'vel_type': vel_type
                }

            elif (vel, RDF.type, QueryUtils.KINEMATICS["OneDimensionalVelocity"]) in self.graph:
                query = f"""
                    SELECT DISTINCT ?speed ?direction ?vel_type ?vel_type2
                    WHERE {{
                        ?vel a kinematics:OneDimensionalVelocity ;
                            a ?vel_type ;
                            a ?vel_type2 ;
                            kinematics:speed ?speed ;
                            kinematics:direction ?direction .

                        VALUES ?vel_type {{ kinematics:VelocityBoundUnitDirectionSpeed kinematics:VelocityReferencePointDirectionSpeed }}
                        VALUES ?vel_type2 {{ kinematics:LinearVelocity kinematics:AngularVelocity }}
                    }}
                """

                if init_bindings is None:
                    init_bindings = {'vel': vel}

                qres = self.graph.query(query, initBindings=init_bindings)

                vals = qres.bindings

                assert len(vals) > 0, "Velocity type is not supported"

                speed = vals[0]['speed']
                direction = vals[0]['direction']
                vel_type = vals[0]['vel_type']
                vel_type2 = vals[0]['vel_type2'].split('#')[1]

                if "VelocityBoundUnitDirectionSpeed" in vel_type:
                    query = f"""
                        SELECT DISTINCT ?f1 ?f3 ?f1_coord ?f2_coord ?f4_coord ?vel_sp ?vu ?du ?dx ?dy ?dz
                        WHERE {{
                            ?speed a kinematics:Speed ;
                                kinematics:of-frame ?f1 ;
                                kinematics:wrt-frame ?f2 ;
                                qudt-schema:hasQuantityKind ?qks .

                            ?direction a kinematics:DirectionVector ;
                                a kinematics:BoundVector ;
                                qudt-schema:hasQuantityKind ?qkd ;
                                kinematics:start ?f3 .

                            FILTER (?f1 != ?f2 && ?f1 = ?f3)

                            ?f1 a frame:Frame .
                            ?f2 a frame:Frame .

                            ?f1_coord a coord:FrameCoordinate ;
                                a coord:FrameReference ;
                                coord:of-frame ?f1 .
                            ?f2_coord a coord:FrameCoordinate ;
                                a coord:FrameReference ;
                                coord:of-frame ?f2 .
                            
                            ?speed_coord a coord:SpeedCoordinate ;
                                coord:of-speed ?speed ;
                                qudt-schema:unit ?vu ;
                                qudt-schema:value ?vel_sp .

                            ?direction_coord a coord:VectorCoordinate ;
                                a coord:VectorXYZ ;
                                coord:of-direction ?direction ;
                                coord:as-seen-by ?f4 ;
                                qudt-schema:unit ?du ;
                                coord:x ?dx ;
                                coord:y ?dy ;
                                coord:z ?dz .
                                
                            ?f4 a frame:Frame .

                            ?f4_coord a coord:FrameCoordinate ;
                                a coord:FrameReference ;
                                coord:of-frame ?f4 .
                        }}
                        LIMIT 1
                    """

                    init_bindings = {'speed': speed, 'direction': direction}

                    qres = self.graph.query(query, initBindings=init_bindings)

                    qb = qres.bindings

                    assert len(qb[0]) > 0, "Velocity type is not supported"

                    # TODO: dir unit is not used for now. write in report

                    dir = [
                        float(qb[0]['dx']),
                        float(qb[0]['dy']),
                        float(qb[0]['dz'])]

                    dir = [1 if d != 0 else np.inf for d in dir]

                    coord_info = {
                        'type': 'VelocityCoordinate',
                        'of_coord': str(qb[0]['f1_coord']).replace(self.ns, ''),
                        'wrt_coord': str(qb[0]['f2_coord']).replace(self.ns, ''),
                        'asb_coord': str(qb[0]['f4_coord']).replace(self.ns, ''),
                        'vel_sp': np.multiply(float(qb[0]['vel_sp']), dir).tolist(),
                        'unit': str(qb[0]['vu']).split('/')[-1],
                        'vel_type': ['OneDimensionalVelocity', 'VelocityBoundUnitDirectionSpeed'] + [vel_type2]
                    }

                else:
                    raise Exception("Velocity type is not supported")

            elif (vel, RDF.type, QueryUtils.KINEMATICS["TwoDimensionalVelocity"]) in self.graph:
                raise Exception("TwoDimensionalVelocity is not supported")

        # check coord type is FrameCoordinate
        elif (coord, RDF.type, QueryUtils.COORD["FrameCoordinate"]) in self.graph:
            c_type = ['FrameCoordinate']

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

            qb = qres.bindings

            assert len(
                qb) > 0, f'FrameCoordinate {coord} is not properly defined'

            coord_info = {
                'of': str(qb[0]['f1']).replace(self.ns, ''),
                'asb': str(qb[0]['f2']).replace(self.ns, '')
                if 'f2' in qb[0].keys() else None,
                'unit': str(qb[0]['unit']).split('/')[-1]}

            # check if qb[0]['x'] is in keys
            if 'x' in qb[0].keys():
                c_type.append('VectorXYZ')
                coord_info['x'] = float(qb[0]['x'])
                coord_info['y'] = float(qb[0]['y'])
                coord_info['z'] = float(qb[0]['z'])

            coord_info['type'] = c_type

        # check coord type is WrenchCoordinate
        elif (coord, RDF.type, QueryUtils.COORD["WrenchCoordinate"]) in self.graph:

            of = self.graph.value(coord, QueryUtils.COORD["of-wrench"])

            if (of, RDF.type, QueryUtils.DYNAMICS["Force"]) in self.graph:

                query = f"""
                    SELECT ?ab_coord ?at_coord ?asb_coord ?unit ?fx ?fy ?fz ?qk
                    WHERE {{
                        ?force_coord a coord:WrenchCoordinate ;
                            a coord:VectorXYZ ;
                            coord:of-wrench ?of ;
                            coord:as-seen-by ?asb ;
                            qudt-schema:unit ?unit ;
                            coord:x ?fx ;
                            coord:y ?fy ;
                            coord:z ?fz .

                        ?asb a frame:Frame .

                        ?asb_coord a coord:FrameCoordinate ;
                            a coord:FrameReference ;
                            coord:of-frame ?asb .

                        ?of a dynamics:Force ;
                            a dynamics:ContactForce ;
                            dynamics:applied-by ?ab ;
                            dynamics:applied-to ?at ;
                            qudt-schema:hasQuantityKind ?qk .

                        FILTER (?ab != ?at && ?at != ?asb)

                        ?ab a frame:Frame .
                        ?at a frame:Frame .
                        
                        ?ab_coord a coord:FrameCoordinate ;
                            a coord:FrameReference ;
                            coord:of-frame ?ab .
                    }}
                """

                if init_bindings is None:
                    init_bindings = {'force_coord': coord, 'of': of}

                qres = self.graph.query(query, initBindings=init_bindings)

                qb = qres.bindings

                assert len(
                    qb) > 0, f'WrenchCoordinate {coord} is not properly defined'

                coord_info = {
                    'type': 'WrenchCoordinate',
                    'ab_coord': str(qb[0]['ab_coord']).replace(self.ns, ''),
                    'asb_coord': str(qb[0]['asb_coord']).replace(self.ns, ''),
                    'unit': str(qb[0]['unit']).split('/')[-1],
                    'fx': float(qb[0]['fx']),
                    'fy': float(qb[0]['fy']),
                    'fz': float(qb[0]['fz']),
                    'quant_kind': str(qb[0]['qk']).split('/')[-1]
                }

            elif (of, RDF.type, QueryUtils.DYNAMICS["Torque"]) in self.graph:
                raise Exception("Torque is not supported")

            elif (of, RDF.type, QueryUtils.DYNAMICS["Wrench"]) in self.graph:
                raise Exception("Wrench is not supported")

            else:
                raise Exception(f"Unknown Wrench type for {coord}")

        # check coord type is TorqueCoordinate
        elif (coord, RDF.type, QueryUtils.COORD["TorqueCoordinate"]) in self.graph:
            pass

        else:
            raise Exception("Coordinate type is not supported")

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
                ?solver a vereshchaginSolver:VereshchaginSolver .
            }}
        """

        if init_bindings is None:
            init_bindings = {'mappings': mappings_iri}

        qres = self.graph.query(query, initBindings=init_bindings)

        qb = qres.bindings

        assert len(qb[0]) > 0, "Mappings is not properly defined"

        mappings_info['interface'] = []

        if (qb[0]['interface'], RDF.type, QueryUtils.MAPPINGS["AccEnegery"]) in self.graph:
            mappings_info['interface'].append("AccEnegery")
        elif (qb[0]['interface'], RDF.type, QueryUtils.MAPPINGS["ExtWrench"]) in self.graph:
            mappings_info['interface'].append("ExtWrench")
        else:
            raise Exception("Interface type is not supported")

        mappings_info['solver'] = str(qb[0]['solver']).replace(self.ns, '')

        if "AccEnegery" in mappings_info['interface']:

            solver_inputs = self.graph.objects(
                mappings_iri, QueryUtils.MAPPINGS["solver-input"])

            mappings_info['solver-input'] = []

            for solver_input in solver_inputs:
                if (solver_input, RDF.type, QueryUtils.CONTROLLERIO["ControllerOutput"]):
                    mappings_info['solver-input'].append(
                        str(solver_input).replace(self.ns, ''))

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

        elif "ExtWrench" in mappings_info['interface']:
            pass
        else:
            raise Exception("Interface type is not supported")

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
            alpha = np.zeros((6, 6))
            dir = None

            if (of_vel, RDF.type, QueryUtils.KINEMATICS["CartesianVelocity"]) in self.graph:
                dir = [1, 1, 1]

            elif (of_vel, RDF.type, QueryUtils.KINEMATICS["VelocityBoundUnitDirectionSpeed"]) in self.graph:

                direction = self.graph.value(
                    of_vel, QueryUtils.KINEMATICS["direction"])

                query = f"""
                    SELECT DISTINCT ?dx ?dy ?dz ?dir_type
                    WHERE {{
                        ?direction_coord a coord:VectorCoordinate ;
                            a coord:VectorXYZ ;
                            coord:of-direction ?direction ;
                            coord:x ?dx ;
                            coord:y ?dy ;
                            coord:z ?dz .
                        OPTIONAL {{
                            ?direction a kinematics:BoundVector ;
                                a ?dir_type .
                            VALUES ?dir_type {{ kinematics:UnitLength }}
                        }}
                    }}
                """

                init_bindings = {'direction': direction}

                qres = self.graph.query(query, initBindings=init_bindings)

                qb = qres.bindings

                assert len(qb[0]) > 0, "Direction type is not supported"

                dir = {}

                dir["dir_vec"] = [
                    float(qb[0]['dx']),
                    float(qb[0]['dy']),
                    float(qb[0]['dz'])]

                assert None not in dir["dir_vec"], "Direction vector is invalid"

                if qb[0]['dir_type'] is not None:
                    dir["dir_type"] = str(qb[0]['dir_type']).split('#')[1]
                else:
                    dir["dir_type"] = None

                # TODO: calculate alpha based on of-frame and wrt-frame in cpp

                dir_vec = dir["dir_vec"]

                if "UnitLength" in dir["dir_type"]:
                    if np.linalg.norm(dir_vec) > 1:
                        raise Exception("Direction vector is not a unit vector")
                    else:
                        dir = dir_vec
                else:
                    raise Exception("Direction vector type is not supported")
            else:
                raise Exception("Velocity type is not supported")

            if (of_vel, RDF.type, QueryUtils.KINEMATICS["LinearVelocity"]) in self.graph:
                dir.extend([0, 0, 0])
            elif (of_vel, RDF.type, QueryUtils.KINEMATICS["AngularVelocity"]) in self.graph:
                dir = [0, 0, 0] + dir
            else:
                raise Exception("Velocity type is not supported")

            np.fill_diagonal(alpha, dir)

            # remove zero rows
            alpha = alpha[~np.all(alpha == 0, axis=1)]

            return alpha.tolist()

        elif (coord, RDF.type, QueryUtils.COORD["DistanceCoordinate"]) in self.graph:
            of_dist = self.graph.value(coord, QueryUtils.COORD["of-distance"])

            query = f"""
                SELECT ?dist_type_1 ?dist_type_2 ?dist_type3
                WHERE {{
                    ?dist a kinematics:EuclideanDistance ;
                        a kinematics:FrameToFrameDistance ;
                        a ?dist_type_1 ;
                        a ?dist_type_2 .
                    VALUES ?dist_type_1 {{ kinematics:LinearDistance kinematics:AngularDistance }}
                    VALUES ?dist_type_2 {{ kinematics:1D kinematics:2D kinematics:3D }}

                }}
            """

            init_bindings = {'dist': of_dist}

            qres = self.graph.query(query, initBindings=init_bindings)

            qb = qres.bindings

            assert len(qb[0]) > 0, f"Distance {of_dist} is not properly defined"

            dt1 = str(qb[0]['dist_type_1']).split('#')[1]
            dt2 = str(qb[0]['dist_type_2']).split('#')[1]

            non_3d_coord_type = None

            if dt2 != "3D":
                # get the type of coordinate
                query = f"""
                    SELECT ?coord_type
                    WHERE {{
                        OPTIONAL {{
                            ?dist_coord a coord:DistanceCoordinate ;
                                a ?coord_type .

                            VALUES ?coord_type {{ coord:AxisX coord:AxisY coord:AxisZ coord:PlaneXY coord:PlaneYZ coord:PlaneZX }}
                        }}
                    }}
                """

                init_bindings = {'dist_coord': coord}

                qres = self.graph.query(query, initBindings=init_bindings)

                qb = qres.bindings

                assert len(qb[0]) > 0, "Coordinate type is not supported"
                
                if qb[0]['coord_type'] is not None:
                    non_3d_coord_type = str(qb[0]['coord_type']).split('#')[1]

            alpha = np.zeros((6, 6))

            coord_vectors = {
                "AxisX": [1, 0, 0],
                "AxisY": [0, 1, 0],
                "AxisZ": [0, 0, 1],
                "PlaneXY": [1, 1, 0],
                "PlaneYZ": [0, 1, 1],
                "PlaneZX": [1, 1, 1],
            }

            if dt2 == "1D" or dt2 == "2D":
                assert non_3d_coord_type is not None, "Coordinate type is not supported"
                if dt1 == "LinearDistance":
                    vec = coord_vectors[non_3d_coord_type]
                    vec.extend([0, 0, 0])
                    np.fill_diagonal(alpha, vec)
                elif dt1 == "AngularDistance":
                    vec = coord_vectors[non_3d_coord_type]
                    vec = [0, 0, 0] + vec
                    np.fill_diagonal(alpha, vec)
            elif dt2 == "3D":
                if dt1 == "LinearDistance":
                    np.fill_diagonal(alpha, [1, 1, 1, 0, 0, 0])
                elif dt1 == "AngularDistance":
                    np.fill_diagonal(alpha, [0, 0, 0, 1, 1, 1])
            else:
                raise Exception(f"Distance type {dt2} is not supported")

            # remove zero rows
            alpha = alpha[~np.all(alpha == 0, axis=1)]

            return alpha.tolist()

        elif (coord, RDF.type, QueryUtils.COORD["WrenchCoordinate"]) in self.graph:
            return None

        else:
            raise Exception("Coordinate type is not supported")

    def construct_abc(self, alpha_in):
        """
        Construct the alpha beta matrices.
        """
        alpha = []
        beta = []
        nc = 0

        # if alpha is not a list of lists
        if len(alpha_in) == 0:
            return alpha, beta, nc
        elif isinstance(alpha_in[0], list):
            for a in alpha_in:
                # if a is not in alpha, add it
                if a not in alpha:
                    alpha.append(a)
        elif isinstance(alpha_in[0], float):
            alpha.append(alpha_in)

        # sort alpha
        alpha.sort(reverse=True)

        assert isinstance(alpha[0], list), "Alpha is not a list of lists"

        nc = len(alpha)

        beta = [0.0 for i in range(nc)]

        # construct beta from alpha based on apperance of non-zero elements
        for i, a in enumerate(alpha):
            if a[2] != 0:
                beta[i] = 9.81

        return alpha, beta, nc
