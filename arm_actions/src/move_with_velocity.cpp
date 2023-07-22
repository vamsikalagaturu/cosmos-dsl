#include "arm_actions/arm_actions.hpp"

int main()
{  
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

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "gen3_robotiq_2f_85.urdf").string();

  // set the base and tool links
  std::string base_link = "base_link";
  std::string tool_link = "bracelet_link";

  // define the robot chain
  KDL::Chain robot_chain;

  // define the initial joint angles
  std::vector initial_joint_angles = {0.0, 0.0, 0.0, -M_PI_2, 0.0, -M_PI_2, 0.0};

  // define the joint angles
  KDL::JntArray q;

  // initialize the robot
  int r = utils->initialize_robot(robot_urdf, robot_chain, base_link, tool_link,
                                  initial_joint_angles, q);
  if (r != 0)
  {
    logger->logError("Failed to initialize robot");
    return -1;
  }

  // number of joints
  int n_joints = robot_chain.getNrOfJoints();

  // number of segments
  int n_segments = robot_chain.getNrOfSegments();
  
  // coords
  std::string target_link = "target_link";
  std::array<double, 3> target_link_coord_position;
  target_link_coord_position = { -0.3044, -0.0149, 0.8056 };
  
  std::string bracelet_link = "bracelet_link";
  std::array<double, 3> bracelet_link_coord_position;
  if (utils->checkLinkInChain(robot_chain, bracelet_link))
  {
    int seg_n = utils->getLinkIdFromChain(robot_chain, bracelet_link);
    std::tuple<std::array<double, 3>, std::array<double, 3>> fk_result =
      solver_utils->computeFK(&robot_chain, q, seg_n);
    bracelet_link_coord_position = std::get<0>(fk_result);
  }
  else
  {
    logger->logError("Not implemented");
    return -1;
  }
  
  
  // initialize the monitors
  Monitor pre_monitor_1(logger, "gt", 
                          0.0025, 
                          "http://qudt.org/vocab/unit/M",
                          &target_link_coord_position);
  
  Monitor post_monitor_1(logger, "lt",
                          0.0025, 
                          "http://qudt.org/vocab/unit/M",
                          &target_link_coord_position);
  
  
  // initialize the PID controllers
  PIDController pid_velocity_controller(20.0, 0.9, 0.0);

  // initialize solver params
  // define unit constraint forces for EE
  KDL::Jacobian alpha_unit_forces;

  // beta - accel energy for EE
  KDL::JntArray beta_energy;

  // set the solver parameters
  KDL::JntArray qd;              // input joint velocities
  KDL::JntArray qdd;             // output joint accelerations
  KDL::JntArray ff_tau;          // input feedforward torques
  KDL::JntArray constraint_tau;  // output constraint torques
  KDL::Wrenches f_ext;           // external forces at each segment

  // initialize the solver weights
  int solver_nc = 3;

  std::vector<std::vector<double>> solver_alpha_weights;
  
  std::vector<double> solver_alpha_col_1 = { 1, 0, 0, 0, 0, 0 };
  solver_alpha_weights.push_back(solver_alpha_col_1);
  std::vector<double> solver_alpha_col_2 = { 0, 1, 0, 0, 0, 0 };
  solver_alpha_weights.push_back(solver_alpha_col_2);
  std::vector<double> solver_alpha_col_3 = { 0, 0, 1, 0, 0, 0 };
  solver_alpha_weights.push_back(solver_alpha_col_3);

  std::vector<double> solver_beta_weights = { 0, 0, 9.81 };

  // initialize vereshchagin solver
  KDL::ChainHdSolver_Vereshchagin vereshchagin_solver = solver_utils->initializeVereshchaginSolver(
      &robot_chain, solver_nc, solver_alpha_weights, alpha_unit_forces, solver_beta_weights, beta_energy,
      qd, qdd, ff_tau,constraint_tau, f_ext);

  // time step
  double dt = 0.001;

  // store positions and velocities
  std::vector<std::array<double, 3>> positions;
  std::vector<std::array<double, 3>> velocities;

  // control velocities and accelerations
  std::array<double, 6> control_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> control_accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // counter
  int i = 0;
  int break_iteration_ = 500;

  // check if pre-conditions are met
  if (!pre_monitor_1.checkAny(bracelet_link_coord_position))
  {
    logger->logError("Pre-condition 1 not met");
    return -1;
  }

  // run the system
  while (true)
  {
    logger->logInfo("Iteration: %d", i);

    // compute the inverse dynamics
    int sr = vereshchagin_solver.CartToJnt(q, qd, qdd, alpha_unit_forces, beta_energy, f_ext,
                                            ff_tau, constraint_tau);
    if (sr < 0)
    {
      logger->logError("KDL: Vereshchagin solver ERROR: %d", sr);
      return -1;
    }  
    // update the joint positions and velocities by integrating the accelerations
    for (int j = 0; j < n_joints; j++)
    {
      q(j) = q(j) + qd(j) * dt + 0.5 * qdd(j) * dt * dt;
      qd(j) = qd(j) + qdd(j) * dt;
    }  
    // get the current link cartesian poses
    std::vector<KDL::Frame> frames(n_segments);
    vereshchagin_solver.getLinkCartesianPose(frames);  
    // get the current link cartesian velocities
    std::vector<KDL::Twist> twists(n_segments);
    vereshchagin_solver.getLinkCartesianVelocity(twists);  
    // print the joint positions, velocities, aceelerations and constraint torques
    logger->logInfo("Joint accelerations: %s", qdd);
    logger->logInfo("Joint torques: %s", constraint_tau);
    logger->logInfo("Joint velocities: %s", qd);
    logger->logInfo("Joint positions: %s", q);

    // update the beta energy and controllers

    if (i % 10 == 0)
    {
      int seg_n = utils->getLinkIdFromChain(robot_chain, bracelet_link);
      
      // get the current tool cartesian velocity
      std::array<double, 3> bracelet_link_coord_velocity = {twists[seg_n].vel.x(),
                                                twists[seg_n].vel.y(),
                                                twists[seg_n].vel.z()};
      velocities.push_back(bracelet_link_coord_velocity);

      auto cid = std::vector<int>{ 1,1,1,0,0,0 };

      for (int j = 0; j < cid.size(); j++)
      {
        if (cid[j] == 1){
          control_velocities[j] = 0.01;
        }
      }

      auto pid_velocity_controller_io_output = pid_velocity_controller.computeControlSignal(bracelet_link_coord_velocity, 
          std::array<double, 3>{control_velocities[0], control_velocities[1],
                                                  control_velocities[2]},
          0.1);

      auto cod = std::vector<int>{ 1,1,1,0,0,0 };
      for (int j = 0; j < cod.size(); j++)
      {
        if (cod[j] == 1){
          control_accelerations[j] = pid_velocity_controller_io_output[j];
        }
      }
    }

    // update the default beta values with the controller values
    std::vector<double> solver_beta_weights = { 0, 0, 9.81 };
    for (int j = 0; j < solver_beta_weights.size(); j++)
    {
      beta_energy(j) = solver_beta_weights[j] + control_accelerations[j];
    }

    logger->logInfo("Control acc: %s", control_accelerations);

    std::cout << std::endl;

    // check if post-conditions are met
    int seg_n = utils->getLinkIdFromChain(robot_chain, bracelet_link);
    std::array<double, 3> bracelet_link_coord_position = {
        frames[seg_n].p.x(), frames[seg_n].p.y(), frames[seg_n].p.z()};
    positions.push_back(bracelet_link_coord_position);
    logger->logInfo("bracelet_link_coord_position: %s", bracelet_link_coord_position);
    if (post_monitor_1.checkAll(bracelet_link_coord_position))
    {
      logger->logInfo("Post-condition 1 met");
      break;
    }

    i++;
    
    if (i == break_iteration_)
      break;
  }

  plotter->plotXYZ(positions, target_link_coord_position);
  plotter->plotXYZ(velocities, {0.01, 0.01, 0.01});

  return 0;
}