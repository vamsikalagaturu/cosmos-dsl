#include "arm_actions/pid_controller.hpp"

PIDController::PIDController(double Kp, double Ki, double Kd, double dt)
    : Kp(Kp),
      Ki(Ki),
      Kd(Kd),
      dt(dt),
      error_sum_x(0),
      error_sum_y(0),
      error_sum_z(0),
      last_error_x(0),
      last_error_y(0),
      last_error_z(0),
      error_sum_1d(0),
      last_error_1d(0)
{
}

PIDController::PIDController(double Kp, double Ki, double Kd, double dt, double threshold,
                             std::string op)
    : Kp(Kp),
      Ki(Ki),
      Kd(Kd),
      dt(dt),
      error_sum_x(0),
      error_sum_y(0),
      error_sum_z(0),
      last_error_x(0),
      last_error_y(0),
      last_error_z(0),
      error_sum_1d(0),
      last_error_1d(0),
      threshold(threshold),
      op(op)
{
}

PIDController::PIDController(double Kp, double Ki, double Kd, double dt, KDL::Vector threshold,
                             std::string op)
    : Kp(Kp),
      Ki(Ki),
      Kd(Kd),
      dt(dt),
      error_sum_x(0),
      error_sum_y(0),
      error_sum_z(0),
      last_error_x(0),
      last_error_y(0),
      last_error_z(0),
      error_sum_1d(0),
      last_error_1d(0),
      op(op)
{
  threshoold_vec = threshold;
}

std::vector<double> PIDController::computeControlSignal_3d(
    const std::array<double, 3>& current_value, const std::array<double, 3>& target_value)
{
  auto [error_x, error_y, error_z] = calc_error(current_value, target_value);

  // Proportional terms
  std::tuple<double, double, double> proportional_term =
      std::make_tuple(Kp * error_x, Kp * error_y, Kp * error_z);

  // Integral terms
  error_sum_x += error_x * dt;
  error_sum_y += error_y * dt;
  error_sum_z += error_z * dt;
  std::tuple<double, double, double> integral_term =
      std::make_tuple(Ki * error_sum_x, Ki * error_sum_y, Ki * error_sum_z);

  // Derivative terms
  std::tuple<double, double, double> derivative_term =
      std::make_tuple(Kd * (error_x - last_error_x) / dt, Kd * (error_y - last_error_y) / dt,
                      Kd * (error_z - last_error_z) / dt);

  last_error_x = error_x;
  last_error_y = error_y;
  last_error_z = error_z;

  std::vector<double> control_signal = {
      std::get<0>(proportional_term) + std::get<0>(integral_term) + std::get<0>(derivative_term),
      std::get<1>(proportional_term) + std::get<1>(integral_term) + std::get<1>(derivative_term),
      std::get<2>(proportional_term) + std::get<2>(integral_term) + std::get<2>(derivative_term)};

  return control_signal;
}

KDL::Vector PIDController::computeControlSignal_3d(const KDL::Vector& current_value,
                                                     const KDL::Vector& target_value)
{
  KDL::Vector error = calc_error(current_value, target_value);

  // Proportional terms
  KDL::Vector proportional_term = error * Kp;

  // Integral terms
  error_sum += error * dt;
  KDL::Vector integral_term = error_sum * Ki;

  // Derivative terms
  KDL::Vector derivative_term = (error - last_error) * Kd / dt;

  last_error = error;

  KDL::Vector control_signal;

  for (int i = 0; i < 3; i++)
  {
    control_signal(i) = proportional_term(i) + integral_term(i) + derivative_term(i);
  }

  // check if any of target val is inf and if so, set the control signal to 0
  for (int i = 0; i < 3; i++)
  {
    if (std::isinf(target_value(i)))
    {
      control_signal(i) = 0;
    }
  }

  return control_signal;
}

double PIDController::computeControlSignal_1d(const KDL::Vector& current_value,
                                              const KDL::Vector& target_value)
{
  KDL::Vector error = calc_error(current_value, target_value);

  // Proportional terms
  KDL::Vector proportional_term = error * Kp;

  // Integral terms
  error_sum += error * dt;
  KDL::Vector integral_term = error_sum * Ki;

  // Derivative terms
  KDL::Vector derivative_term = (error - last_error) * Kd / dt;

  last_error = error;

  KDL::JntArray control_signal(6);

  for (int i = 0; i < 3; i++)
  {
    control_signal(i) = proportional_term(i) + integral_term(i) + derivative_term(i);
  }

  // add all the values of the control signal and return the result
  double sum = 0;
  for (int i = 0; i < 3; i++)
  {
    sum += control_signal(i);
  }

  return sum;
}

double PIDController::computeControlSignal_1d(const double& current_value,
                                              const double& target_value)
{
  double error = calc_error(current_value, target_value);

  // Proportional terms
  double proportional_term = error * Kp;

  // Integral terms
  error_sum_1d += error * dt;
  double integral_term = error_sum_1d * Ki;

  // Derivative terms
  double derivative_term = (error - last_error_1d) * Kd / dt;

  last_error_1d = error;

  double control_signal = proportional_term + integral_term + derivative_term;

  return control_signal;
}

std::tuple<double, double, double> PIDController::calc_error(const std::array<double, 3>& p1,
                                                             const std::array<double, 3>& p2)
{
  double dx = p2[0] - p1[0];
  double dy = p2[1] - p1[1];
  double dz = p2[2] - p1[2];

  return std::make_tuple(dx, dy, dz);
}

KDL::Vector PIDController::calc_error(const KDL::Vector& v1, const KDL::Vector& v2)
{
  return v2 - v1 - threshoold_vec;
}

double PIDController::calc_error(const double& v1, const double& v2)
{
  return v2 - v1 - threshold;
}
