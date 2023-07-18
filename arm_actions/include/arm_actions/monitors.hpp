#ifndef MONITOR_HPP
#define MONITOR_HPP

#include <string>

#include "arm_actions/logger.hpp"
#include "arm_actions/utils.hpp"

class Monitor
{
public:
  Monitor(std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
          std::string thresh_unit, 
          std::array<double, 3> *target_position, std::string ns = "http://example.com/rob#");
  ~Monitor();

  bool checkAny(std::array<double, 3> current_position);

  bool checkAll(std::array<double, 3> current_position);

  bool check(std::array<double, 3> current_position,
             std::array<double, 3> target_position);

private:
  std::string _ns;
  std::string _comp_op;
  double _thresh_val;
  std::string _thresh_unit;

  std::array<double, 3> *_target_position;

  std::shared_ptr<Logger> _logger;

  std::shared_ptr<Utils> _utils;
};

#endif  // MONITOR_HPP