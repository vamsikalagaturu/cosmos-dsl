#include "arm_actions/arm_actions.hpp"
#include <chrono>

int main()
{
  // initialize config
  bool _debug = false;
  bool _visualize = true;
  bool _plot = true;
  bool _save_data = true;
  ENV _env = ENV::SIM;

  // get current file path
  std::filesystem::path path = __FILE__;

  // get the current working directory
  std::filesystem::path cwd = std::filesystem::current_path();

  std::string log_path = cwd.parent_path().string() + "/logs/runs/";

  std::string data_path = cwd.parent_path().string() + "/logs/data/";

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

  auto move_arm_down_vel = [&]() {

  // initialize the chain
  // define the robot chain
  KDL::Chain robot_chain;

  int r = utils->initialize_robot_urdf(robot_urdf, robot_chain, base_link, tool_link);

  if (r != 0)
  {
    logger->logError("Failed to initialize robot");
    return -1;
  }

  // number of joints
  int n_joints = robot_chain.getNrOfJoints();

  // number of segments
  int n_segments = robot_chain.getNrOfSegments();

  // set chain for the tf utils
  tf_utils->setChain(&robot_chain);

  // set the solver parameters
  KDL::JntArray q(n_joints);                // input joint positions
  KDL::JntArray qd(n_joints);               // input joint velocities
  KDL::JntArray qdd(n_joints);              // output joint accelerations
  KDL::JntArray joint_torques_m(n_joints);  // measured joint torques
  KDL::Wrench f_tool_m;                     // measured tool wrench

  std::vector<double> initial_joint_angles;

  // define the initial joint angles
  if (_env == ENV::SIM)
  {
    initial_joint_angles = { 0.0, 0.26, 0.0, 2.26, 0.0, -0.95, -1.57 };

    // initialize the robot joint angles
    int r = utils->init_q(&robot_chain, q, initial_joint_angles, _env);

    if (r != 0)
    {
      logger->logError("Failed to initialize robot");
      return -1;
    }
  }
  else if (_env == ENV::ROB)
  {
    logger->logWarning("Real robot mode");
    km->initialize(0, 1, 0.0);
    km->set_control_mode(2);  // 0 is position, 1 is velocity, 2 is torque
    km->get_robot_state(q, qd, joint_torques_m, f_tool_m);
  }
  else
    logger->logError("Invalid mode");

  // coords
  std::string bracelet_link = "bracelet_link";
  KDL::Frame bracelet_link_coord_frame;
  if (utils->checkLinkInChain(robot_chain, bracelet_link))
  {
    int seg_n = utils->getLinkIdFromChain(robot_chain, bracelet_link);
    bracelet_link_coord_frame = solver_utils->computeFKFrame(&robot_chain, q, seg_n+1);
  }
  else
  {
    logger->logError("Not implemented");
    return -1;
  }
  
  // TODO: for now, we assume initial twist is zero
  KDL::Twist bracelet_link_coord_twist;
  
  
  // initialize the monitors
  // Pre-conditions
  
  Monitor pre_monitor_1(Monitor::MonitorType::PRE, logger, "eq", "M-PER-SEC", 0.0
                          ,KDL::Twist( { 0.0, 0.0, 0.0 }, {INFINITY, INFINITY, INFINITY} ));
  
  Monitor pre_monitor_2(Monitor::MonitorType::PRE, logger, "eq", "RAD-PER-SEC", 0.0
                          ,KDL::Twist( {INFINITY, INFINITY, INFINITY}, { 0.0, 0.0, 0.0 } ));
  

  // Post-conditions
  
  Monitor post_monitor_1(Monitor::MonitorType::POST, logger, "eq", "M-PER-SEC", 0.001
                          ,KDL::Twist( { 0.0, 0.0, 0.0 }, {INFINITY, INFINITY, INFINITY} ));
  
  Monitor post_monitor_2(Monitor::MonitorType::POST, logger, "eq", "RAD-PER-SEC", 0.001
                          ,KDL::Twist( {INFINITY, INFINITY, INFINITY}, { 0.0, 0.0, 0.0 } ));
  
  
  // initialize the PID controllers
  
  PIDController pid_move_arm_down_lin_vel_controller(20.0, 0.9, 0.0, 1.0);
  
  PIDController pid_move_arm_down_ang_vel_controller(20.0, 0.9, 0.0, 1.0);
  

  // initialize solver params
  // define unit constraint forces for EE
  KDL::Jacobian alpha_unit_forces;

  // beta - accel energy for EE
  KDL::JntArray beta_energy;

  // set the solver parameters
  KDL::JntArray ff_tau(n_joints);          // input feedforward torques
  KDL::JntArray constraint_tau(n_joints);  // output constraint torques
  KDL::Wrenches f_ext(n_segments);         // external forces at each segment

  // initialize the solver weights
  int solver_nc = 6;

  std::vector<std::vector<double>> solver_alpha_weights;
  
  std::vector<double> solver_alpha_col_1 = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  solver_alpha_weights.push_back(solver_alpha_col_1);
  std::vector<double> solver_alpha_col_2 = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 };
  solver_alpha_weights.push_back(solver_alpha_col_2);
  std::vector<double> solver_alpha_col_3 = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
  solver_alpha_weights.push_back(solver_alpha_col_3);
  std::vector<double> solver_alpha_col_4 = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 };
  solver_alpha_weights.push_back(solver_alpha_col_4);
  std::vector<double> solver_alpha_col_5 = { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
  solver_alpha_weights.push_back(solver_alpha_col_5);
  std::vector<double> solver_alpha_col_6 = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };
  solver_alpha_weights.push_back(solver_alpha_col_6);

  std::vector<double> solver_beta_weights = { 0.0, 0.0, 9.81, 0.0, 0.0, 0.0 };

  // initialize vereshchagin solver
  KDL::ChainHdSolver_Vereshchagin vereshchagin_solver = solver_utils->initializeVereshchaginSolver(
      &robot_chain, solver_nc, solver_alpha_weights, alpha_unit_forces, solver_beta_weights, beta_energy,
      qd, qdd, ff_tau,constraint_tau, f_ext);

  // time step
  double dt = 0.001;

  // initialize the control acceleration
  KDL::JntArray control_accelerations(6);

  // store positions and velocities for plotting
  std::vector<KDL::Vector> positions;
  std::vector<KDL::Vector> velocities;

  // data
  std::vector<KDL::JntArray> q_vec;
  std::vector<KDL::JntArray> qd_vec;
  std::vector<KDL::JntArray> qdd_vec;
  std::vector<KDL::JntArray> jnt_tau_vec;
  std::vector<KDL::Twist> current_vel_vec;
  std::vector<KDL::Twist> target_vel_vec;
  std::vector<KDL::Vector> current_pos_vec;
  std::vector<KDL::Vector> target_pos_vec;
  std::vector<KDL::JntArray> control_signal_vec;

  std::vector<KDL::Frame> frames(n_segments);
  std::vector<KDL::Twist> twists(n_segments);

  // check if pre-conditions are met
  if (!pre_monitor_1.checkAll(bracelet_link_coord_twist))
  {
    logger->logError("Pre-condition 1 not met");
    return -1;
  }
  if (!pre_monitor_2.checkAll(bracelet_link_coord_twist))
  {
    logger->logError("Pre-condition 2 not met");
    return -1;
  }

  // start loop
  // counter
  int i = 0;
  int break_iteration_ = 2500;

  // run the system
  while (true)
  {
    if (_debug) logger->logInfo("Iteration: %d", i);

    logger->logInfo("Iteration: %d", i);

    if (_env == ENV::ROB) km->get_robot_state(q, qd, joint_torques_m, f_tool_m);

    // update the beta energy and controllers

    if (i % 1 == 0)
    {
      int seg_n = utils->getLinkIdFromChain(robot_chain, bracelet_link);
      
      // get the current tool cartesian velocity
      KDL::Twist bracelet_link_coord_twist = twists[seg_n];

      auto cid = std::vector<int>{ 1,1,1 };

      KDL::Twist control_twist;

      auto vsp = KDL::Vector{ 0.0,0.0,-0.05 };

      for (int j = 0; j < 3; j++)
      {
        if (cid[j] == 1){
          control_twist.vel(j) = vsp[j];
        }
      }

      auto pid_move_arm_down_lin_vel_controller_io_output = pid_move_arm_down_lin_vel_controller.computeControlSignal_3d(bracelet_link_coord_twist.vel, 
          control_twist.vel);
      auto cod = std::vector<int>{ 1,1,1 };
      for (int j = 0; j < cod.size(); j++)
      {
        if (cod[j] == 1){
          control_accelerations(j) = pid_move_arm_down_lin_vel_controller_io_output(j);
        }
      }}

    

    if (i % 1 == 0)
    {
      int seg_n = utils->getLinkIdFromChain(robot_chain, bracelet_link);
      
      // get the current tool cartesian velocity
      KDL::Twist bracelet_link_coord_twist = twists[seg_n];

      auto cid = std::vector<int>{ 1,1,1 };

      KDL::Twist control_twist;

      auto vsp = KDL::Vector{ 0.0,0.0,0.0 };

      for (int j = 0; j < 3; j++)
      {
        if (cid[j] == 1){
          control_twist.rot(j) = vsp[j];
        }
      }

      auto pid_move_arm_down_ang_vel_controller_io_output = pid_move_arm_down_ang_vel_controller.computeControlSignal_3d(bracelet_link_coord_twist.rot, 
          control_twist.rot);

      
      auto cod = std::vector<int>{ 1,1,1 };
      for (int j = 0; j < cod.size(); j++)
      {
        if (cod[j] == 1){
          control_accelerations(j+3) = pid_move_arm_down_ang_vel_controller_io_output(j);
        }
      }}

    // update the default beta values with the controller values
    std::vector<double> solver_beta_weights = { 0.0, 0.0, 9.81, 0.0, 0.0, 0.0 };
    for (int j = 0; j < solver_beta_weights.size(); j++)
    {
      beta_energy(j) = solver_beta_weights[j] + control_accelerations(j);
    }

    // if (_debug) logger->logInfo("Beta energy: %s", beta_energy);

    // compute the inverse dynamics
    int sr = vereshchagin_solver.CartToJnt(q, qd, qdd, alpha_unit_forces, beta_energy, f_ext,
                                            ff_tau, constraint_tau);
    if (sr < 0)
    {
      // logger->logError("KDL: Vereshchagin solver ERROR: %d", sr);
      return -1;
    }  
    
    // get the current link cartesian poses
    vereshchagin_solver.getLinkCartesianPose(frames);  
    // get the current link cartesian velocities
    vereshchagin_solver.getLinkCartesianVelocity(twists);
    
    // visualization
    if (_save_data)
    {
      q_vec.push_back(q);
      qd_vec.push_back(qd);
      qdd_vec.push_back(qdd);
      jnt_tau_vec.push_back(constraint_tau);
      int seg_n_1 = utils->getLinkIdFromChain(robot_chain, bracelet_link);
      current_vel_vec.push_back(twists[seg_n_1]);
      target_vel_vec.push_back(KDL::Twist( { 0.0, 0.0, -0.05 }, {INFINITY, INFINITY, INFINITY} ));
      current_pos_vec.push_back(frames[seg_n_1].p);
      target_pos_vec.push_back(KDL::Vector{0.0, 0.0, 0.0});
      control_signal_vec.push_back(beta_energy);
    }

    // update the joint positions and velocities if in simulation
    if (_env == ENV::SIM) solver_utils->updateQandQd(q, qd, &qdd, dt);

    // print the joint positions, velocities, aceelerations and constraint torques
    if (_debug) {
      logger->logInfo("Joint accelerations: %s", qdd);
      logger->logInfo("Joint torques: %s", constraint_tau);
      logger->logInfo("Joint velocities: %s", qd);
      logger->logInfo("Joint positions: %s", q);
    }
    
    // send torques to the robot if in real robot mode
    if (_env == ENV::ROB)
    {
      // logger->logInfo("Sending torques to robot");
      km->set_joint_torques(constraint_tau);
    }

    // check if post-conditions are met
    int seg_n_1 = utils->getLinkIdFromChain(robot_chain, bracelet_link);
    
    KDL::Twist bracelet_link_coord_twist_1 = twists[seg_n_1];
    velocities.push_back(bracelet_link_coord_twist_1.vel);
    if (_debug) logger->logInfo("bracelet_link_coord_twist_1: %s", bracelet_link_coord_twist_1.vel);
    
    
    if (post_monitor_1.checkAll(bracelet_link_coord_twist_1))
    {
      logger->logInfo("Post-condition 1 met");
      // break;current_vel_lin_z
    }

    std::cout << std::endl;

    // end loop
    i++;

    if (i == break_iteration_)
      break;

    // usleep(300);
    
  }

  if (_save_data)
  {
    logger->logInfo("Saving the data");

    plotter->saveDataToCSV(q_vec, qd_vec, qdd_vec, jnt_tau_vec, current_vel_vec, target_vel_vec,
                           current_pos_vec, target_pos_vec, control_signal_vec, "move_arm_down_vel_sim");
  }

  if (_visualize)
  {
    logger->logInfo("Visualizing the simulation");
    RobotSimulation simulation;
    simulation.run(&initial_joint_angles, q_vec, qd_vec);
  }

  if (_plot)
  {
    logger->logInfo("Plotting the results");
    plotter->plotXYZ(velocities, KDL::Vector{0.0, 0.0, -0.05}, "velocities");
  }

  return 1;
  };
  
  // run the motion
  
  int move_arm_down_vel_status = move_arm_down_vel();
  if (move_arm_down_vel_status != 1)
  {
    std::cout << "Motion move_arm_down_vel failed" << std::endl;
    return 0;
  }
  

  return 0;
}