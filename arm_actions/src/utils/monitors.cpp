#include "arm_actions/monitors.hpp"

Monitor::Monitor(MonitorType mt, std::shared_ptr<Logger> logger, std::string comp_op,
                 double thresh_val, std::string thresh_unit, std::array<double, 3> *target)
{
  _mt = mt;
  _comp_op = comp_op;
  _thresh_val = thresh_val;
  _thresh_unit = thresh_unit;

  _target = target;

  _logger = logger;

  _utils = std::make_shared<Utils>(logger);

  _math_utils = std::make_shared<MathUtils>(0.0001);
};

Monitor::Monitor(MonitorType mt, std::shared_ptr<Logger> logger, std::string comp_op,
                 std::string thresh_unit, double thresh_val, KDL::Frame *target)
{
  _mt = mt;
  _comp_op = comp_op;
  _thresh_val = thresh_val;
  _thresh_unit = thresh_unit;

  _target_frame = target;

  _logger = logger;

  _math_utils = std::make_shared<MathUtils>(0.0001);
};

Monitor::Monitor(MonitorType mt, std::shared_ptr<Logger> logger, std::string comp_op,
                 std::string thresh_unit, double thresh_val, KDL::Twist target)
{
  _mt = mt;
  _comp_op = comp_op;
  _thresh_unit = thresh_unit;
  _thresh_val = thresh_val;
  _target_twist = target;

  _logger = logger;

  _utils = std::make_shared<Utils>(logger);

  _math_utils = std::make_shared<MathUtils>(0.0001);
};

Monitor::~Monitor() {}

// std::array's

bool Monitor::checkAll(std::array<double, 3> current_position)
{
  auto error = _utils->calc_error(current_position, *_target);

  return _checkAll(error);
}

bool Monitor::checkAll(std::array<double, 3> current_position,
                       std::array<double, 3> target_position)
{
  auto error = _utils->calc_error(current_position, target_position);

  return _checkAll(error);
}

bool Monitor::checkAny(std::array<double, 3> current_position)
{
  auto error = _utils->calc_error(current_position, *_target);

  return _checkAny(error);
}

bool Monitor::checkAny(std::array<double, 3> current_position,
                       std::array<double, 3> target_position)
{
  auto error = _utils->calc_error(current_position, target_position);

  return _checkAny(error);
}

// KDL::Frames

bool Monitor::checkAll(KDL::Frame current)
{
  auto error = KDL::diff(current, *_target_frame);
  if (_mt == MonitorType::PRE)
  {
    return _checkAll(error);
  }
  else
  {
    _current_twists.push(error);

    if (_current_twists.isFull())
    {
      // take the average of the queue
      auto _current_twists_average = _math_utils->computeAverage(&_current_twists);

      return _checkAll(_current_twists_average);
    }
    else
    {
      return false;
    }
  }
}

bool Monitor::checkAll(KDL::Frame current, KDL::Frame target)
{
  auto error = KDL::diff(current, target);

  return _checkAll(error);
}

bool Monitor::checkAny(KDL::Frame current)
{
  auto error = KDL::diff(current, *_target_frame);
  if (_mt == MonitorType::PRE)
  {
    return _checkAny(error);
  }
  else
  {
    _current_twists.push(error);

    if (_current_twists.isFull())
    {
      // take the average of the queue
      auto _current_twists_average = _math_utils->computeAverage(&_current_twists);

      return _checkAny(_current_twists_average);
    }
    else
    {
      return false;
    }
  }
}

bool Monitor::checkAny(KDL::Frame current, KDL::Frame target)
{
  auto error = KDL::diff(current, target);

  return _checkAny(error);
}

bool Monitor::checkAll(KDL::Twist current)
{
  if (_mt == MonitorType::PRE)
  {
    // auto error = KDL::diff(current, _target_twist);
    
    return _checkAll(current, _target_twist);
  }
  else
  {
    _current_twists.push(current);

    if (_current_twists.isFull())
    {
      // take the average of the queue
      auto _current_twists_average = _math_utils->computeAverage(&_current_twists);

      // auto error = KDL::diff(_current_twists_average, _target_twist);

      return _checkAll(_current_twists_average, _target_twist);
    }
    else
    {
      return false;
    }
  }
}

bool Monitor::checkAny(KDL::Twist current)
{
  auto error = KDL::diff(current, _target_twist);

  return _checkAny(error);
}

bool Monitor::_checkAll(std::vector<double> error)
{
  if (_comp_op == "lt")
  {
    if (std::all_of(error.begin(), error.end(), [&](double e) { return abs(e) < _thresh_val; }))
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
    if (std::all_of(error.begin(), error.end(), [&](double e) { return abs(e) > _thresh_val; }))
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
    if (std::all_of(error.begin(), error.end(),
                    [&](double e) { return fabs(e - _thresh_val) < 0.0001; }))
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
    if (std::any_of(error.begin(), error.end(), [&](double e) { return abs(e) < _thresh_val; }))
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
    if (std::any_of(error.begin(), error.end(), [&](double e) { return abs(e) > _thresh_val; }))
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
    if (std::any_of(error.begin(), error.end(),
                    [&](double e) { return fabs(e - _thresh_val) < 0.0001; }))
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
    if (std::all_of(error.data, error.data + 3, [&](double e) { return abs(e) < _thresh_val; }))
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
    if (std::all_of(error.data, error.data + 3, [&](double e) { return abs(e) > _thresh_val; }))
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
    if (std::all_of(error.data, error.data + 3,
                    [&](double e) { return fabs(e - _thresh_val) < 0.0001; }))
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
    if (std::any_of(error.data, error.data + 3, [&](double e) { return abs(e) < _thresh_val; }))
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
    if (std::any_of(error.data, error.data + 3, [&](double e) { return abs(e) > _thresh_val; }))
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
    if (std::any_of(error.data, error.data + 3,
                    [&](double e) { return fabs(e - _thresh_val) < 0.0001; }))
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

bool Monitor::_checkAny(KDL::Twist error)
{
  auto result = _math_utils->compare(error, _thresh_val, _comp_op);

  return std::any_of(result.begin(), result.end(), [&](bool b) { return b; });
}

bool Monitor::_checkAll(KDL::Twist error)
{
  auto result = _math_utils->compare(error, _thresh_val, _comp_op);

  return std::all_of(result.begin(), result.end(), [&](bool b) { return b; });
}

bool Monitor::_checkAll(KDL::Twist avg_current, KDL::Twist target)
{
  auto result = _math_utils->compare(avg_current, target, _thresh_val, _comp_op);

  return std::all_of(result.begin(), result.end(), [&](bool b) { return b; });
}
