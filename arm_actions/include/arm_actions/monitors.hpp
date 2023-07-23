#ifndef MONITOR_HPP
#define MONITOR_HPP

#include <string>

#include "arm_actions/logger.hpp"
#include "arm_actions/utils.hpp"

#include "frames.hpp"

class Monitor
{
public:
  Monitor(std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
          std::string thresh_unit, std::array<double, 3> *target);
  
  Monitor(std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
          std::string thresh_unit, KDL::Frame *target);

  Monitor(std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
          std::string thresh_unit);
  
  ~Monitor();

  bool checkAny(std::array<double, 3> current);

  bool checkAll(std::array<double, 3> current);

  bool checkAll(std::array<double, 3> current, std::array<double, 3> target);

  bool checkAny(std::array<double, 3> current, std::array<double, 3> target);

  /**
   * @brief checks current position against target position
   * @param current position frame
  */
  bool checkAll(KDL::Frame current);

  /**
   * @brief checks current position against target position
   * @param current position frame
  */
  bool checkAny(KDL::Frame current);

  /**
   * @brief checks current position against target position
   * @param current position frame
  */
  bool checkAll(KDL::Frame current, KDL::Frame target);

  /**
   * @brief checks current position against target position
   * @param current position frame
  */
  bool checkAny(KDL::Frame current, KDL::Frame target);

  /**
   * @brief checks current velocity against threshold
   * @param current velocity
   */
  bool checkAll(KDL::Twist current);

  /**
   * @brief checks current velocity against threshold
   * @param current velocity
   */
  bool checkAny(KDL::Twist current);

private:

  /**
   * @brief takes in error and compares against threshold
   * @param error vector
   */
  bool _checkAll(std::vector<double> error);

  /**
   * @brief takes in error and compares against threshold
   * @param error vector
   */
  bool _checkAny(std::vector<double> error);

  /**
   * @brief takes in error and compares against threshold
   * @param error kdl vector
   */
  bool _checkAll(KDL::Vector error);

  /**
   * @brief takes in error and compares against threshold
   * @param error kdl vector
  */
  bool _checkAny(KDL::Vector error);

  std::string _comp_op;
  double _thresh_val;
  std::string _thresh_unit;

  std::array<double, 3> *_target;

  KDL::Frame *_target_frame;

  std::shared_ptr<Logger> _logger;

  std::shared_ptr<Utils> _utils;
};

#endif  // MONITOR_HPP