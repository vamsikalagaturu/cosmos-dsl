#ifndef MONITOR_HPP
#define MONITOR_HPP

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "arm_actions/fixed_size_queue.hpp"
#include "arm_actions/logger.hpp"
#include "arm_actions/math_utils.hpp"
#include "arm_actions/utils.hpp"
#include "frames.hpp"

// enum

class Monitor
{
public:
  enum MonitorType
  {
    PRE,
    POST
  };

  Monitor(MonitorType mt, std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
          std::string thresh_unit, std::array<double, 3> *target);

  Monitor(MonitorType mt, std::shared_ptr<Logger> logger, std::string comp_op, std::string thresh_unit,
          double thresh_val, KDL::Frame *target);

  Monitor(MonitorType mt, std::shared_ptr<Logger> logger, std::string comp_op, std::string thresh_unit,
          double thresh_val, KDL::Twist target);

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

  /**
   * @brief checks current velocity against threshold
   * @param current velocity
   * @param target velocity
   */
  bool checkAll(KDL::Twist current, KDL::Twist target);

  /**
   * @brief checks current velocity against threshold
   * @param current velocity
   * @param target velocity
   */
  bool checkAny(KDL::Twist current, KDL::Twist target);

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

  /**
   * @brief takes in error and compares against threshold
   * @param error kdl twist
   */
  bool _checkAny(KDL::Twist error);

  /**
   * @brief takes in error and compares against threshold
   * @param error kdl twist
   */
  bool _checkAll(KDL::Twist error);

  /**
   * @brief takes in error and compares against threshold
   * @param avg current twist
   * @param target twist
   */
  bool _checkAll(KDL::Twist avg_current, KDL::Twist target);

	MonitorType _mt;

  std::string _comp_op;
  double _thresh_val;
  std::string _thresh_unit;

  std::vector<double> _thresholds;

  KDL::Frame _threshold_frame;
  KDL::Twist _threshold_twist;

  std::array<double, 3> *_target;

  KDL::Frame *_target_frame;
  KDL::Twist _target_twist;

  FixedSizeQueue<KDL::Frame, 10> _current_frames;
  FixedSizeQueue<KDL::Twist, 10> _current_twists;

  std::shared_ptr<Logger> _logger;

  std::shared_ptr<Utils> _utils;

  std::shared_ptr<MathUtils> _math_utils;
};

#endif  // MONITOR_HPP