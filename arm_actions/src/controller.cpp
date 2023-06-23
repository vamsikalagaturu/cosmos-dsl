#include <filesystem>

#include "arm_actions/gnu_plotter.hpp"
#include "arm_actions/logger.hpp"
#include "arm_actions/pid_controller.hpp"
#include "arm_actions/solver_utils.hpp"
#include "arm_actions/utils.hpp"
#include "chain.hpp"
#include "chainhdsolver_vereshchagin.hpp"
#include "chainidsolver.hpp"
#include "frames_io.hpp"

int main()
{
  // initialize logger
  std::shared_ptr<Logger> logger = std::make_shared<Logger>(true, false, "../logs/runs/");

  // initialize plotter
  std::shared_ptr<GNUPlotter> plotter = std::make_shared<GNUPlotter>("../logs/data/", false);

  // initialize utils
  std::shared_ptr<Utils> utils = std::make_shared<Utils>(logger);

  // initialize solver utils
  std::shared_ptr<SolverUtils> solver_utils = std::make_shared<SolverUtils>(logger);

  // get current file path
  std::filesystem::path path = __FILE__;

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

  // compute the forward kinematics
  std::tuple<std::array<double, 3>, std::array<double, 3>> fk_result =
      solver_utils->computeFK(&robot_chain, q);

  // current position
  std::array<double, 3> current_position = std::get<0>(fk_result);

  // target position
  std::array<double, 3> target_position = {current_position[0] + 0.01, current_position[1] + 0.01,
                                           current_position[2] + 0.1};

  // define unit constraint forces for EE
  KDL::Jacobian alpha_unit_forces;

  // beta - accel energy for EE
  KDL::JntArray beta_energy;

  // set the constraint weights
  std::array<double, 6> constraint_weights = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  // set the solver parameters
  KDL::JntArray qd;              // input joint velocities
  KDL::JntArray qdd;             // output joint accelerations
  KDL::JntArray ff_tau;          // input feedforward torques
  KDL::JntArray constraint_tau;  // output constraint torques
  KDL::Wrenches f_ext;           // external forces at each segment

  // initialize vereshchagin solver
  KDL::ChainHdSolver_Vereshchagin vereshchagin_solver = solver_utils->initializeVereshchaginSolver(
      &robot_chain, constraint_weights, alpha_unit_forces, beta_energy, qd, qdd, ff_tau,
      constraint_tau, f_ext);

  // initialize the PID controllers
  PIDController pos_controller(5, 0.9, 0);   // position controller
  PIDController vel_controller(30, 0.9, 0);  // velocity controller

  // time step
  double dt = 0.001;

  // store positions and velocities
  std::vector<std::array<double, 3>> positions;
  std::vector<std::array<double, 3>> velocities;

  // control velocities and accelerations
  double control_vel_x, control_vel_y, control_vel_z = 0.0;
  double control_acc_x, control_acc_y, control_acc_z = 0.0;

  // counter
  int i = 0;

  // run the system at 1KHz
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

    // get the current tool cartesian pose
    std::array<double, 3> current_position = {
        frames[n_segments - 1].p.x(), frames[n_segments - 1].p.y(), frames[n_segments - 1].p.z()};
    positions.push_back(current_position);

    // get the current link cartesian velocities
    std::vector<KDL::Twist> twists(n_segments);
    vereshchagin_solver.getLinkCartesianVelocity(twists);

    // get the current tool cartesian velocity
    std::array<double, 3> current_velocity = {twists[n_segments - 1].vel.x(),
                                              twists[n_segments - 1].vel.y(),
                                              twists[n_segments - 1].vel.z()};
    velocities.push_back(current_velocity);

    // print the joint positions, velocities, aceelerations and constraint torques
    logger->logInfo("Joint accelerations: %s", qdd);
    logger->logInfo("Joint torques: %s", constraint_tau);
    logger->logInfo("Joint velocities: %s", qd);
    logger->logInfo("Joint positions: %s", q);
    logger->logInfo("Target position: [%f, %f, %f]", target_position[0], target_position[1],
                    target_position[2]);
    logger->logInfo("Current position: [%f, %f, %f]", current_position[0], current_position[1],
                    current_position[2]);

    // call the position controller at frequency of 10hz
    if (i % 100 == 0)
    {
      std::tie(control_vel_x, control_vel_y, control_vel_z) =
          pos_controller.computeControlSignal(current_position, target_position, 0.01);
    }

    // call the velocity controller at frequency of 100hz
    if (i % 10 == 0)
    {
      std::tie(control_acc_x, control_acc_y, control_acc_z) = vel_controller.computeControlSignal(
          current_velocity, std::array<double, 3>{control_vel_x, control_vel_y, control_vel_z},
          0.1);
    }

    // update the beta energy
    solver_utils->updateBetaEnergy(beta_energy,
                                   {control_acc_x, control_acc_y, control_acc_z, 0, 0, 0});

    logger->logInfo("Control acc: [%f, %f, %f]", control_acc_x, control_acc_y, control_acc_z);

    std::cout << std::endl;

    i++;

    if (i > 500)
      break;
  }

  plotter->plotXYZ(positions, target_position);

  return 0;
}