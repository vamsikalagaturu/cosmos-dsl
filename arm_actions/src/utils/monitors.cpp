#include "arm_actions/monitors.hpp"

Monitor::Monitor(std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
                 std::string thresh_unit, std::array<double, 3> *target_position, std::string ns)
{
  _ns = ns;
  _comp_op = comp_op;
  _thresh_val = thresh_val;
  _thresh_unit = thresh_unit;

  _target_position = target_position;

  _logger = logger;

  _utils = std::make_shared<Utils>(logger);
};

Monitor::~Monitor() {}

bool Monitor::checkAny(std::array<double, 3> current_position)
{
  auto error = _utils->calc_error(current_position, *_target_position);

  if (_comp_op == _ns + "lt")
  {
    if (std::any_of(error.begin(), error.end(), [&](double e) { return e < _thresh_val; }))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (_comp_op == _ns + "gt")
  {
    if (std::any_of(error.begin(), error.end(), [&](double e) { return e > _thresh_val; }))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (_comp_op == _ns + "eq")
  {
    if (std::any_of(error.begin(), error.end(), [&](double e) { return fabs(e - _thresh_val) < 0.0001; }))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    std::cerr << "Unknown comparison operator" << std::endl;
    std::cerr << "Handler not implemented" << std::endl;
    return false;
  }
}

bool Monitor::checkAll(std::array<double, 3> current_position)
{
  auto error = _utils->calc_error(current_position, *_target_position);

  if (_comp_op == _ns + "lt")
  {
    if (std::all_of(error.begin(), error.end(), [&](double e) { return e < _thresh_val; }))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (_comp_op == _ns + "gt")
  {
    if (std::all_of(error.begin(), error.end(), [&](double e) { return e > _thresh_val; }))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (_comp_op == _ns + "eq")
  {
    if (std::all_of(error.begin(), error.end(), [&](double e) { return fabs(e - _thresh_val) < 0.0001; }))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    std::cerr << "Unknown comparison operator" << std::endl;
    std::cerr << "Handler not implemented" << std::endl;
    return false;
  }
}

bool Monitor::check(std::array<double, 3> current_position, std::array<double, 3> target_position)
{
  double distance = _utils->computeEuclideanDistance(current_position, target_position);

  if (_comp_op == _ns + "lt")
  {
    if (distance < _thresh_val)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (_comp_op == _ns + "gt")
  {
    if (distance > _thresh_val)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (_comp_op == _ns + "eq")
  {
    if (fabs(distance - _thresh_val) < 0.0001)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    std::cerr << "Unknown comparison operator" << std::endl;
    std::cerr << "Handler not implemented" << std::endl;
    return false;
  }
}