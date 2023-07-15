from rdflib import RDF, Graph
import rdflib


class QueryUtils:

    ROB = "http://example.com"
    COORD = rdflib.Namespace(ROB + "/coordinate#")
    MAPPINGS = rdflib.Namespace(ROB + "/mappings#")

    def __init__(self, graph: Graph):
        self.graph = graph

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
            pre_condition_info['monitor'] = row[0]
            pre_condition_info['constraint'] = ci

        return pre_condition_info
    
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
        
        qres = self.graph.query(query, initBindings=init_bindings)

        per_condition_info = {}

        for row in qres:
            ci = self.get_constraint_info(init_bindings={'constraint': row[1]})
            per_condition_info['controller'] = row[0]
            per_condition_info['constraint'] = ci

        return per_condition_info
    
    def get_constraint_info(self, init_bindings=None):
        """
        Returns a dictionary of constraint information for a given set of bindings.
        """
        constraint_info = {}
        
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

        for row in qres:
            constraint_info['constraint'] = row[0]
            constraint_info['operator'] = row[1]
            constraint_info['thresh_val'] = row[2]
            constraint_info['thresh_unit'] = row[3]
            constraint_info['coord'] = row[4]
        
        return constraint_info
    
    def get_coord_info(self, coord, init_bindings=None):
        """
        Returns a dictionary of coordinate information for a given set of bindings.
        """
        coord_info = {}
        
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
                    'type' : 'DistanceCoordinate',
                    'unit' : row[1],
                    'quant_kind' : row[2],
                    'f1_coord' : row[3],
                    'f2_coord' : row[4]
                }

        # check coord type is velocity coordinate
        elif (coord, RDF.type, QueryUtils.COORD["VelocityCoordinate"]) in self.graph:
            query = f"""
                SELECT ?vel_coord ?vu ?qk ?f1_coord ?f2_coord
                WHERE {{
                    ?vel_coord a coord:VelocityCoordinate ;
                        coord:of-velocity ?vel ;
                        qudt-schema:unit ?vu .
                    ?vel a kinematics:CartesianVelocity ;
                        a kinematics:LinearVelocity ;
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
                init_bindings = {'vel_coord': coord}
            
            qres = self.graph.query(query, initBindings=init_bindings)

            for row in qres:
                coord_info = {
                    'type' : 'VelocityCoordinate',
                    'unit' : row[1],
                    'quant_kind' : row[2],
                    'f1_coord' : row[3],
                    'f2_coord' : row[4]
                }
            
        # check coord type is FrameCoordinate
        elif (coord, RDF.type, QueryUtils.COORD["FrameCoordinate"]) in self.graph:
            query = f"""
                SELECT ?frame_coord ?f1 ?unit ?x ?y ?z
                WHERE {{
                    ?frame_coord a coord:FrameCoordinate ;
                        a coord:FrameReference ;
                        coord:of-frame ?f1 ;
                        qudt-schema:unit ?unit .
                    ?f1 a frame:Frame .

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
                    'type' : 'FrameCoordinate',
                    'f1' : row[1],
                    'unit' : row[2],
                    'x' : row[3],
                    'y' : row[4],
                    'z' : row[5],
                }

        return coord_info
    
    def get_mappings_info(self, mappings_iri, init_bindings=None):
        """
        Returns a dictionary of mappings information for a given set of bindings.
        """
        mappings_info = {}
        
        query = f"""
            SELECT DISTINCT ?interface ?solver ?controller ?controller_input
            WHERE {{
                ?mappings a mapping:ControllerSolver ;
                    mapping:interface ?interface ;
                    mapping:solver ?solver ;
                    mapping:controller ?controller ;
                    mapping:controller-input ?controller_input ;
                    mapping:controller-input-dimension ?cid .
                ?interface a mapping:AccEnegery .
                ?solver a vereshchaginSolver:VereshchaginSolver .
                ?controller a cascadedController:CascadedController .
                ?controller_input a coord:FrameCoordinate .
            }}
        """

        if init_bindings is None:
            init_bindings = {'mappings': mappings_iri}
        
        qres = self.graph.query(query, initBindings=init_bindings)

        for row in qres:
            mappings_info['interface'] = row[0]
            mappings_info['solver'] = row[1]
            mappings_info['controller'] = self.get_controller_info(init_bindings={'controller': row[2]})
            mappings_info['controller-input'] = row[3]

        cip = self.graph.objects(mappings_iri, QueryUtils.MAPPINGS["controller-input-dimension"])
        cop = self.graph.objects(mappings_iri, QueryUtils.MAPPINGS["controller-output-dimension"])

        mappings_info['controller-input-dimension'] = list(cip)
        mappings_info['controller-output-dimension'] = list(cop)
        
        return mappings_info


    def get_controller_info(self, init_bindings=None):
        """
        Returns a dictionary of controller information for a given set of bindings.
        """
        controller_info = {}
        
        query = f"""
            SELECT DISTINCT ?controller ?pc ?vc
            WHERE {{
                ?controller a cascadedController:CascadedController ;
                    cascadedController:position-controller ?pc ;
                    cascadedController:velocity-controller ?vc .
                ?pc a pidController:PIDController .
                ?vc a pidController:PIDController .
            }}
        """
        
        qres = self.graph.query(query, initBindings=init_bindings)

        for row in qres:
            controller_info['controller'] = row[0]
            controller_info['controller_type'] = 'CascadedController'
            controller_info['pos-c'] = self.get_pid_controller_info(init_bindings={'controller': row[1]})
            controller_info['vel-c'] = self.get_pid_controller_info(init_bindings={'controller': row[2]})
        
        return controller_info
    
    def get_pid_controller_info(self, init_bindings=None):
        """
        Returns a dictionary of PID controller information for a given set of bindings.
        """
        pid_controller_info = {}
        
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
        
        qres = self.graph.query(query, initBindings=init_bindings)

        for row in qres:
            pid_controller_info['kp'] = row[0]
            pid_controller_info['ki'] = row[1]
            pid_controller_info['kd'] = row[2]
            pid_controller_info['dt'] = row[3]
            pid_controller_info['constraint'] = row[4]
        
        return pid_controller_info
    


