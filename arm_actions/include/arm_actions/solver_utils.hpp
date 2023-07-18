#ifndef VERESHCHAGIN_SOLVER_HPP
#define VERESHCHAGIN_SOLVER_HPP

#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "chain.hpp"
#include "chainfksolver.hpp"
#include "chainfksolverpos_recursive.hpp"
#include "chainidsolver_vereshchagin.hpp"
#include "jacobian.hpp"
#include "jntarray.hpp"
#include "logger.hpp"

class SolverUtils
{
public:
  // get logger shared pointer in constructor
  SolverUtils(std::shared_ptr<Logger> logger);
  ~SolverUtils();

  /**
   * Initializes and configures the Vereshchagin solver for a given robot chain.
   *
   * @param robot_chain The robot chain to solve for.
   * @param constraint_weights An array of constraint weights for linear and angular forces.
   *                           The weights are expected to be in the following order:
   *                           [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z].
   * @param alpha_unit_forces The output Jacobian matrix of unit constraint forces for end
   * effector.
   * @param beta_energy The output joint acceleration energy for end effector.
   * @param qd The input joint velocities.
   * @param qdd The input joint accelerations.
   * @param ff_tau The output feedforward torques.
   * @param constraint_tau The output constraint torques.
   * @param f_ext The external forces at each segment of the robot chain.
   * @return The initialized Vereshchagin solver.
   */
  KDL::ChainHdSolver_Vereshchagin initializeVereshchaginSolver(
      const KDL::Chain *robot_chain, const std::array<double, 6> constraint_weights,
      KDL::Jacobian &alpha_unit_forces, KDL::JntArray &beta_energy, KDL::JntArray &qd,
      KDL::JntArray &qdd, KDL::JntArray &ff_tau, KDL::JntArray &constraint_tau,
      KDL::Wrenches &f_ext);

  /**
   * @brief Populates the columns of a Jacobian matrix with alpha unit forces.
   * @param alpha_lin A vector containing the linear components of the alpha unit forces (size 3).
   * @param alpha_ang A vector containing the angular components of the alpha unit forces (size 3).
   * @param alpha_unit_forces The KDL::Jacobian object to store the alpha unit forces.
   */
  static void populateAlphaUnitForces(const std::vector<double> &alpha_lin,
                                      const std::vector<double> &alpha_ang,
                                      KDL::Jacobian *alpha_unit_forces);

  /**
   * @brief Computes the forward kinematics (position and orientation) of a robot given joint
   * positions.
   * @param robot_chain The KDL::Chain object representing the robot's kinematic chain.
   * @param q The KDL::JntArray object representing the joint positions.
   * @param seg_nr The segment number to compute the forward kinematics for.
   * @return A tuple containing the position (x, y, z) and orientation (roll, pitch, yaw) of the
   * tool tip.
   */
  static std::tuple<std::array<double, 3>, std::array<double, 3>> computeFK(
      KDL::Chain *robot_chain, KDL::JntArray &q, int seg_nr = -1);

  /**
   * @brief Updates the beta_energy vector with the given control acceleration enegy input and
   * compensates for gravity.
   * @param beta_energy The KDL::JntArray object representing the joint acceleration energy.
   * @param control_ae The control acceleration energy input array (size 6).
   */
  static void updateBetaEnergy(KDL::JntArray &beta_energy,
                               const std::array<double, 6> &control_ae = {0, 0, 0, 0, 0, 0});

  /**
   * @brief Update q and qd with the given control qdd input.
   * @param q The KDL::JntArray object representing the joint positions.
   * @param qd The KDL::JntArray object representing the joint velocities.
   * @param qdd The KDL::JntArray object representing the joint accelerations.
   * @param dt The time step.
   */
  static void updateQandQd(KDL::JntArray &q, KDL::JntArray &qd, const KDL::JntArray &qdd,
                           double dt);

private:
  std::shared_ptr<Logger> _logger;
};

#endif  // VERESHCHAGIN_SOLVER_HPP