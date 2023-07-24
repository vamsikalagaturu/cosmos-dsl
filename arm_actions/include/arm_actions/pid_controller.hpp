#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <array>
#include <tuple>
#include <vector>

#include "frames.hpp"
#include "jntarray.hpp"

/**
 * @brief PID Controller class for computing control signals.
 */
class PIDController
{
public:
  /**
   * @brief Constructs a PIDController object with the specified gains.
   *
   * @param Kp The proportional gain.
   * @param Ki The integral gain.
   * @param Kd The derivative gain.
   */
  PIDController(double Kp, double Ki, double Kd);

  /**
   * @brief Computes the control signal based on the current and target values.
   *
   * @param current_value The current values of the system (x, y, z).
   * @param target_value The desired target values (x, y, z).
   * @param dt The time step or time difference.
   * @return The computed control signal as a tuple (u_x, u_y, u_z).
   */
  std::vector<double>  computeControlSignal(
      const std::array<double, 3>& current_value, const std::array<double, 3>& target_value,
      double dt);

  /**
   * @brief Computes the control signal based on the current and target values.
   * @param current_value The current values of the system kdl vector.
   * @param target_value The desired target values kdl vector.
   * @param dt The time step or time difference.
   * @return The computed control signal as a kdl JntArray.
   */
  KDL::JntArray computeControlSignal(const KDL::Vector& current_value,
                                   const KDL::Vector& target_value, double dt);

private:
  /**
   * @brief Calculates the error between two points.
   *
   * @param p1 The first point (x1, y1, z1).
   * @param p2 The second point (x2, y2, z2).
   * @return The error as a tuple (dx, dy, dz).
   */
  std::tuple<double, double, double> calc_error(const std::array<double, 3>& p1,
                                                const std::array<double, 3>& p2);

  /**
   * @brief Calculates the error between two vectors.
   *
   * @param v1 The first vector.
   * @param v2 The second vector.
   * @return The error as a kdl vector.
   */
  KDL::Vector calc_error(const KDL::Vector& v1, const KDL::Vector& v2);

  double Kp;            // Proportional gain
  double Ki;            // Integral gain
  double Kd;            // Derivative gain
  double error_sum_x;   // Accumulated error in the x-axis
  double error_sum_y;   // Accumulated error in the y-axis
  double error_sum_z;   // Accumulated error in the z-axis
  double last_error_x;  // Previous error in the x-axis
  double last_error_y;  // Previous error in the y-axis
  double last_error_z;  // Previous error in the z-axis

  KDL::Vector error_sum;  // Accumulated error
  KDL::Vector last_error; // Previous error
};

#endif /* PID_CONTROLLER_HPP */
