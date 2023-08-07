#include "arm_actions/arm_actions.hpp"

int main()
{
  // initialize config
  bool _debug = true;
  ENV _env = ENV::SIM;

  // get current file path
  std::filesystem::path path = __FILE__;

  std::string log_path = path.parent_path().parent_path().parent_path().string() + "/logs/runs/";

  std::string data_path = path.parent_path().parent_path().parent_path().string() + "/logs/data/";

  // initialize logger
  std::shared_ptr<Logger> logger = std::make_shared<Logger>(true, false, log_path);

  // initialize plotter
  std::shared_ptr<GNUPlotter> plotter = std::make_shared<GNUPlotter>(data_path, true, false);

  // initialize utils
  std::shared_ptr<Utils> utils = std::make_shared<Utils>(logger);

  // initialize solver utils
  std::shared_ptr<SolverUtils> solver_utils = std::make_shared<SolverUtils>(logger);

  // initialize tf utils
  std::shared_ptr<TfUtils> tf_utils = std::make_shared<TfUtils>(logger);

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "gen3_robotiq_2f_85.urdf").string();

  // set the base and tool links
  std::string base_link = "base_link";
  std::string tool_link = "bracelet_link";

  // initialize the kinova mediator to control the robot
  std::shared_ptr<kinova_mediator> km = std::make_shared<kinova_mediator>();

  auto move_arm_forward_no_z_cons = [&]()
  {
    // initialize the chain
    // define the robot chain
    KDL::Chain robot_chain;

    // set the solver parameters
    KDL::JntArray q;                // input joint positions
    KDL::JntArray qd;               // input joint velocities
    KDL::JntArray qdd;              // output joint accelerations
    KDL::JntArray joint_torques_m;  // measured joint torques
    KDL::Wrench f_tool_m;           // measured tool wrench

    std::vector<double> initial_joint_angles;

    // define the initial joint angles
    if (_env == ENV::SIM)
    {
      initial_joint_angles = {0.0, 0.26, 0.0, 2.26, 0.0, -0.95, -1.57};
    }
    else if (_env == ENV::ROB)
    {
      logger->logWarning("Real robot mode");
      km->initialize(0, 0, 0.0);
      km->set_control_mode(2);  // 0 is position, 1 is velocity, 2 is torque
      km->get_robot_state(q, qd, joint_torques_m, f_tool_m);
    }
    else
      logger->logError("Invalid mode");

    // initialize the robot
    int r = utils->initialize_robot_urdf(robot_urdf, robot_chain, base_link, tool_link, q,
                                    initial_joint_angles, _env);

    if (r != 0)
    {
      logger->logError("Failed to initialize robot");
      return -1;
    }

    // number of joints
    int n_joints = robot_chain.getNrOfJoints();

    // number of segments
    int n_segments = robot_chain.getNrOfSegments();

    KDL::JntArray ff_tau;             // input feedforward torques
    KDL::JntArray constraint_tau;     // output constraint torques
    KDL::Wrenches f_ext(n_segments);  // external forces at each segment

    // set chain for the tf utils
    tf_utils->setChain(&robot_chain);

    // coords
    std::string bracelet_link = "bracelet_link";
    KDL::Frame bracelet_link_coord_frame;
    if (utils->checkLinkInChain(robot_chain, bracelet_link))
    {
      int seg_n = utils->getLinkIdFromChain(robot_chain, bracelet_link);
      bracelet_link_coord_frame = solver_utils->computeFKFrame(&robot_chain, q, seg_n + 1);
    }
    else
    {
      logger->logError("Not implemented");
      return -1;
    }

    // TODO: for now, we assume initial twist is zero
    KDL::Twist bracelet_link_coord_twist;
    std::string target_link_ef = "target_link_ef";
    KDL::Frame target_link_ef_coord_frame;
    target_link_ef_coord_frame.p = {0.2, 0.0, 0.0};
    tf_utils->transform(target_link_ef_coord_frame, &q, CoordinateSystem::EE,
                        CoordinateSystem::BASE);

    // initialize the monitors
    // Pre-conditions

    Monitor pre_monitor_1(Monitor::MonitorType::PRE, logger, "eq", "RAD-PER-SEC", 0.0,
                          KDL::Twist({INFINITY, INFINITY, INFINITY}, {0.0, 0.0, 0.0}));

    Monitor pre_monitor_2(Monitor::MonitorType::PRE, logger, "gt", "M", 0.025,
                          &target_link_ef_coord_frame);

    // Post-conditions

    Monitor post_monitor_1(Monitor::MonitorType::POST, logger, "lt", "M", 0.0025,
                           &target_link_ef_coord_frame);

    // initialize the PID controllers

    PIDController pid_bracelet_link_target_link_ef_xy_dist_controller(20.0, 0.9, 0.0, 0.1);

    // initialize solver params
    // define unit constraint forces for EE
    KDL::Jacobian alpha_unit_forces;

    // beta - accel energy for EE
    KDL::JntArray beta_energy;

    // initialize the solver weights
    int solver_nc = 3;

    std::vector<std::vector<double>> solver_alpha_weights;

    std::vector<double> solver_alpha_col_1 = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    solver_alpha_weights.push_back(solver_alpha_col_1);
    std::vector<double> solver_alpha_col_2 = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    solver_alpha_weights.push_back(solver_alpha_col_2);
    std::vector<double> solver_alpha_col_3 = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    solver_alpha_weights.push_back(solver_alpha_col_3);

    std::vector<double> solver_beta_weights = {0.0, 0.0, 9.81};

    // initialize vereshchagin solver
    KDL::ChainHdSolver_Vereshchagin vereshchagin_solver =
        solver_utils->initializeVereshchaginSolver(
            &robot_chain, solver_nc, solver_alpha_weights, alpha_unit_forces, solver_beta_weights,
            beta_energy, qd, qdd, ff_tau, constraint_tau, f_ext);

    // time step
    double dt = 0.001;

    // initialize the control acceleration
    KDL::JntArray control_accelerations(6);

    // store positions and velocities
    std::vector<KDL::Vector> positions;
    std::vector<KDL::Vector> velocities;

    std::vector<KDL::Twist> twists(n_segments);

    // check if pre-conditions are met
    if (!pre_monitor_1.checkAll(bracelet_link_coord_twist))
    {
      logger->logError("Pre-condition 1 not met");
      return -1;
    }
    if (!pre_monitor_2.checkAny(bracelet_link_coord_frame))
    {
      logger->logError("Pre-condition 2 not met");
      return -1;
    }

    // start loop
    // counter
    int i = 0;
    int break_iteration_ = 500;

    std::vector<KDL::JntArray> q_vec;
    std::vector<KDL::JntArray> qd_vec;
    std::vector<KDL::JntArray> jnt_tau_vec;

    // run the system
    while (true)
    {
      if (_debug)
        logger->logInfo("Iteration: %d", i);

      if (_env == ENV::ROB) km->get_robot_state(q, qd, joint_torques_m, f_tool_m);

      // update the beta energy and controllers

      if (i % 10 == 0)
      {
        int seg_n = utils->getLinkIdFromChain(robot_chain, bracelet_link);

        // get the current tool cartesian position
        KDL::Frame bracelet_link_coord_frame =
            solver_utils->computeFKFrame(&robot_chain, q, seg_n + 1);

        auto cid = std::vector<int>{1, 1, 0, 0, 0, 0};

        KDL::Frame control_frame;

        for (int j = 0; j < cid.size(); j++)
        {
          if (cid[j] == 1)
          {
            control_frame.p(j) = target_link_ef_coord_frame.p(j);
          }
        }

        auto pid_bracelet_link_target_link_ef_xy_controller_io_output =
            pid_bracelet_link_target_link_ef_xy_dist_controller.computeControlSignal_3d(
                bracelet_link_coord_frame.p, control_frame.p);
        auto cod = std::vector<int>{1, 1, 0, 0, 0, 0};
        for (int j = 0; j < cod.size(); j++)
        {
          if (cod[j] == 1)
          {
            control_accelerations(j) = pid_bracelet_link_target_link_ef_xy_controller_io_output(j);
          }
        }
      }

      // update the default beta values with the controller values
      std::vector<double> solver_beta_weights = {0.0, 0.0, 9.81};
      for (int j = 0; j < solver_beta_weights.size(); j++)
      {
        beta_energy(j) = solver_beta_weights[j] + control_accelerations(j);
      }

      if (_debug)
        logger->logInfo("Beta energy: %s", beta_energy);

      // compute the inverse dynamics
      int sr = vereshchagin_solver.CartToJnt(q, qd, qdd, alpha_unit_forces, beta_energy, f_ext,
                                             ff_tau, constraint_tau);
      if (sr < 0)
      {
        logger->logError("KDL: Vereshchagin solver ERROR: %d", sr);
        return -1;
      }
      // update the joint positions and velocities if in simulation
      if (_env == ENV::SIM) solver_utils->updateQandQd(q, qd, &qdd, dt);
      // get the current link cartesian poses
      std::vector<KDL::Frame> frames(n_segments);
      vereshchagin_solver.getLinkCartesianPose(frames);
      // get the current link cartesian velocities
      std::vector<KDL::Twist> twists(n_segments);
      vereshchagin_solver.getLinkCartesianVelocity(twists);

      if (_env == ENV::SIM)
      {
        q_vec.push_back(q);
        qd_vec.push_back(qd);
        jnt_tau_vec.push_back(constraint_tau);
      }

      // send torques to the robot if in real robot mode
      if (_env == ENV::ROB)
      {
        logger->logInfo("Sending torques to robot");
        km->set_joint_torques(constraint_tau);
      }

      // print the joint positions, velocities, aceelerations and constraint torques
      if (_debug)
      {
        logger->logInfo("Joint accelerations: %s", qdd);
        logger->logInfo("Joint torques: %s", constraint_tau);
        logger->logInfo("Joint velocities: %s", qd);
        logger->logInfo("Joint positions: %s", q);
      }

      // check if post-conditions are met
      int seg_n_1 = utils->getLinkIdFromChain(robot_chain, bracelet_link);

      KDL::Frame bracelet_link_coord_frame_1 = frames[seg_n_1];
      positions.push_back(bracelet_link_coord_frame_1.p);
      if (_debug)
        logger->logInfo("bracelet_link_coord_frame_1: %s", bracelet_link_coord_frame_1.p);
      if (_debug)
        logger->logInfo("target_link_ef_coord_frame: %s", target_link_ef_coord_frame.p);

      if (post_monitor_1.checkAll(bracelet_link_coord_frame_1))
      {
        logger->logInfo("Post-condition 1 met");
        break;
      }

      std::cout << std::endl;

      // end loop
      i++;

      if (i == break_iteration_)
        break;
    }

    RobotSimulation simulation;
    simulation.run(&initial_joint_angles, q_vec, qd_vec, jnt_tau_vec);
    
    plotter->plotXYZ(positions, target_link_ef_coord_frame.p, "positions");

    return 1;
  };

  // run the motion

  int move_arm_forward_no_z_cons_status = move_arm_forward_no_z_cons();
  if (move_arm_forward_no_z_cons_status != 1)
  {
    std::cout << "Motion move_arm_forward_no_z_cons failed" << std::endl;
    return 0;
  }

  return 0;
}