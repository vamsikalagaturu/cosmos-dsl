#include "arm_actions/monitors.hpp"

Monitor::Monitor(std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
                 std::string thresh_unit, std::array<double, 3> *target)
{
  _comp_op = comp_op;
  _thresh_val = thresh_val;
  _thresh_unit = thresh_unit;

  _target = target;

  _logger = logger;

  _utils = std::make_shared<Utils>(logger);
};

Monitor::Monitor(std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
                 std::string thresh_unit, KDL::Frame *target)
{
  _comp_op = comp_op;
  _thresh_val = thresh_val;
  _thresh_unit = thresh_unit;

  _target_frame = target;

  _logger = logger;

  _utils = std::make_shared<Utils>(logger);
};

Monitor::~Monitor() {}

// std::array's

bool Monitor::checkAll(std::array<double, 3> current_position)
{
  auto error = _utils->calc_error(current_position, *_target);

  return _checkAll(error);
}

bool Monitor::checkAll(std::array<double, 3> current_position, std::array<double, 3> target_position)
{
  auto error = _utils->calc_error(current_position, target_position);

  return _checkAll(error);
}

bool Monitor::checkAny(std::array<double, 3> current_position)
{
  auto error = _utils->calc_error(current_position, *_target);

  return _checkAny(error);
}

bool Monitor::checkAny(std::array<double, 3> current_position, std::array<double, 3> target_position)
{
  auto error = _utils->calc_error(current_position, target_position);

  return _checkAny(error);
}

// KDL::Frames

bool Monitor::checkAll(KDL::Frame current)
{
  KDL::Vector current_position = current.p;

  KDL::Vector target_position = _target_frame->p;

  auto error = _utils->calc_error(current_position, target_position);

  return _checkAll(error);
}

bool Monitor::checkAll(KDL::Frame current, KDL::Frame target)
{
  KDL::Vector current_position = current.p;

  KDL::Vector target_position = target.p;

  auto error = _utils->calc_error(current_position, target_position);

  return _checkAll(error);
}

bool Monitor::checkAny(KDL::Frame current)
{
  KDL::Vector current_position = current.p;

  KDL::Vector target_position = _target_frame->p;

  auto error = _utils->calc_error(current_position, target_position);

  return _checkAny(error);
}

bool Monitor::checkAny(KDL::Frame current, KDL::Frame target)
{
  KDL::Vector current_position = current.p;

  KDL::Vector target_position = target.p;

  auto error = _utils->calc_error(current_position, target_position);

  return _checkAny(error);
}

bool Monitor::_checkAll(std::vector<double> error)
{
  if (_comp_op == "lt")
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
  else if (_comp_op == "gt")
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
  else if (_comp_op == "eq")
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

bool Monitor::_checkAny(std::vector<double> error)
{
  if (_comp_op == "lt")
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
  else if (_comp_op == "gt")
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
  else if (_comp_op == "eq")
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

bool Monitor::_checkAll(KDL::Vector error)
{
  if (_comp_op == "lt")
  {
    if (std::all_of(error.data, error.data + 3, [&](double e) { return e < _thresh_val; }))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (_comp_op == "gt")
  {
    if (std::all_of(error.data, error.data + 3, [&](double e) { return e > _thresh_val; }))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (_comp_op == "eq")
  {
    if (std::all_of(error.data, error.data + 3, [&](double e) { return fabs(e - _thresh_val) < 0.0001; }))
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

bool Monitor::_checkAny(KDL::Vector error)
{
  if (_comp_op == "lt")
  {
    if (std::any_of(error.data, error.data + 3, [&](double e) { return e < _thresh_val; }))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (_comp_op == "gt")
  {
    if (std::any_of(error.data, error.data + 3, [&](double e) { return e > _thresh_val; }))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if (_comp_op == "eq")
  {
    if (std::any_of(error.data, error.data + 3, [&](double e) { return fabs(e - _thresh_val) < 0.0001; }))
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

