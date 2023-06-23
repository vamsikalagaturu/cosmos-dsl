#include "arm_actions/solver_utils.hpp"

SolverUtils::SolverUtils(std::shared_ptr<Logger> logger)
{
  _logger = logger;
}

SolverUtils::~SolverUtils() {}

void SolverUtils::populateAlphaUnitForces(const std::vector<double> &alpha_lin,
                                          const std::vector<double> &alpha_ang,
                                          KDL::Jacobian *alpha_unit_forces)
{
  if (alpha_lin.size() != 3 || alpha_ang.size() != 3)
  {
    // Invalid input size, handle the error accordingly
    return;
  }

  KDL::Twist unit_force_x_l(KDL::Vector(alpha_lin[0], 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
  alpha_unit_forces->setColumn(0, unit_force_x_l);

  KDL::Twist unit_force_y_l(KDL::Vector(0.0, alpha_lin[1], 0.0), KDL::Vector(0.0, 0.0, 0.0));
  alpha_unit_forces->setColumn(1, unit_force_y_l);

  KDL::Twist unit_force_z_l(KDL::Vector(0.0, 0.0, alpha_lin[2]), KDL::Vector(0.0, 0.0, 0.0));
  alpha_unit_forces->setColumn(2, unit_force_z_l);

  KDL::Twist unit_force_x_a(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(alpha_ang[0], 0.0, 0.0));
  alpha_unit_forces->setColumn(3, unit_force_x_a);

  KDL::Twist unit_force_y_a(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, alpha_ang[1], 0.0));
  alpha_unit_forces->setColumn(4, unit_force_y_a);

  KDL::Twist unit_force_z_a(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, alpha_ang[2]));
  alpha_unit_forces->setColumn(5, unit_force_z_a);
}

KDL::ChainHdSolver_Vereshchagin SolverUtils::initializeVereshchaginSolver(
    const KDL::Chain *robot_chain, const std::array<double, 6> constraint_weights,
    KDL::Jacobian &alpha_unit_forces, KDL::JntArray &beta_energy, KDL::JntArray &qd,
    KDL::JntArray &qdd, KDL::JntArray &ff_tau, KDL::JntArray &constraint_tau, KDL::Wrenches &f_ext)
{
  // number of constraints
  int n_constraints = 6;

  // number of joints
  int n_joints = robot_chain->getNrOfJoints();

  // number of segments
  int n_segments = robot_chain->getNrOfSegments();

  // root acceleration - set gravity vector
  // direction is specified opposite for the solver
  KDL::Twist root_acc(KDL::Vector(0.0, 0.0, 9.81), KDL::Vector(0.0, 0.0, 0.0));

  // define unit constraint forces for EE
  alpha_unit_forces.resize(n_constraints);

  // linear unit forces
  std::vector<double> alpha_lin = {constraint_weights[0], constraint_weights[1],
                                   constraint_weights[2]};
  // angular unit forces
  std::vector<double> alpha_ang = {constraint_weights[3], constraint_weights[4],
                                   constraint_weights[5]};

  populateAlphaUnitForces(alpha_lin, alpha_ang, &alpha_unit_forces);

  // beta - accel energy for EE
  beta_energy.resize(n_constraints);
  beta_energy(0) = 0.0;
  beta_energy(1) = 0.0;
  beta_energy(2) = 9.81;  // to compensate gravity
  beta_energy(3) = 0.0;
  beta_energy(4) = 0.0;
  beta_energy(5) = 0.0;

  // use vereshchagin solver
  KDL::ChainHdSolver_Vereshchagin vereshchagin_solver(*robot_chain, root_acc, n_constraints);

  // set the solver parameters
  qd.resize(n_joints);              // input joint velocities
  qdd.resize(n_joints);             // input joint accelerations
  ff_tau.resize(n_joints);          // output feedforward torques
  constraint_tau.resize(n_joints);  // output constraint torques

  KDL::Vector f(0.0, 0.0, 0.0);
  KDL::Vector n(0.0, 0.0, 0.0);
  KDL::Wrench f_tool(f, n);
  f_ext.resize(n_segments);        // external forces at each segment
  f_ext[n_segments - 1] = f_tool;  // input external forces at EE

  return vereshchagin_solver;
}

std::tuple<std::array<double, 3>, std::array<double, 3>> SolverUtils::computeFK(
    KDL::Chain *robot_chain, KDL::JntArray &q)
{
  // create the solver
  KDL::ChainFkSolverPos_recursive fk_solver(*robot_chain);

  // create the frame that will contain the results
  KDL::Frame tool_tip_frame;

  // calculate forward kinematics
  int r = fk_solver.JntToCart(q, tool_tip_frame);
  if (r < 0)
  {
    std::cout << "Failed to compute forward kinematics with error: " << r << std::endl;
    return std::make_tuple(std::array<double, 3>{0, 0, 0}, std::array<double, 3>{0, 0, 0});
  }

  // extract position from frame
  double x = tool_tip_frame.p.x();
  double y = tool_tip_frame.p.y();
  double z = tool_tip_frame.p.z();

  // round the position to 4 decimal places
  x = round(x * 10000.0) / 10000.0;
  y = round(y * 10000.0) / 10000.0;
  z = round(z * 10000.0) / 10000.0;

  // convert the orientation to roll, pitch, yaw
  double roll, pitch, yaw;
  tool_tip_frame.M.GetRPY(roll, pitch, yaw);

  // convert the angles to degrees
  roll = roll * 180.0 / M_PI;
  pitch = pitch * 180.0 / M_PI;
  yaw = yaw * 180.0 / M_PI;

  // round the angles to 2 decimal places
  roll = round(roll * 100.0) / 100.0;
  pitch = round(pitch * 100.0) / 100.0;
  yaw = round(yaw * 100.0) / 100.0;

  // return the position and RPY values
  std::array<double, 3> position = {x, y, z};
  std::array<double, 3> rpy = {roll, pitch, yaw};
  return std::make_tuple(position, rpy);
}

void SolverUtils::updateBetaEnergy(KDL::JntArray &beta_energy,
                                   const std::array<double, 6> &control_ae)
{
  beta_energy(0) = control_ae[0];
  beta_energy(1) = control_ae[1];
  beta_energy(2) = 9.81 + control_ae[2];
  beta_energy(3) = control_ae[3];
  beta_energy(4) = control_ae[4];
  beta_energy(5) = control_ae[5];
}

void SolverUtils::updateQandQd(KDL::JntArray &q, KDL::JntArray &qd, const KDL::JntArray &qdd,
                               double dt)
{
  // update the joint positions
  for (int i = 0; i < qdd.rows(); i++)
  {
    qd(i) = qd(i) + qdd(i) * dt;
    q(i) = q(i) + qd(i) * dt;
  }
}
