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

  _math_utils = std::make_shared<MathUtils>(0.0001);
};

Monitor::Monitor(std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
                 std::string thresh_unit, KDL::Frame *target)
{
  _comp_op = comp_op;
  _thresh_val = thresh_val;
  _thresh_unit = thresh_unit;

  _target_frame = target;

  // set threshold_frame
  std::fill(_threshold_twist.vel.data, _threshold_twist.vel.data + 3, thresh_val);

  // TODO: handle orientation

  _logger = logger;

  _utils = std::make_shared<Utils>(logger);

  _math_utils = std::make_shared<MathUtils>(0.0001);
};

Monitor::Monitor(std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
          std::string thresh_unit, KDL::Frame *target, std::vector<double> dimensions)
{
  _comp_op = comp_op;
  _thresh_val = thresh_val;
  _thresh_unit = thresh_unit;

  _target_frame = target;

  _dimensions = dimensions;

  // fill _threshold_frame with thresh_val based on non-zero dimensions
  std::transform(_dimensions.begin(), _dimensions.begin() + 3, _threshold_twist.vel.data,
                 [&](double d) { return d == 0.0 ? 0.0 : thresh_val; });

  // TODO: handle orientation

  _logger = logger;

  _math_utils = std::make_shared<MathUtils>(0.0001);
};

Monitor::Monitor(std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
                 std::string thresh_unit)
{
  _comp_op = comp_op;
  _thresh_val = thresh_val;
  _thresh_unit = thresh_unit;

  // set _threshold_twist with thresh_val
  std::fill(_threshold_twist.vel.data, _threshold_twist.vel.data + 3, thresh_val);
  
  // TODO: handle orientation

  _logger = logger;

  _utils = std::make_shared<Utils>(logger);

  _math_utils = std::make_shared<MathUtils>(0.0001);
};

Monitor::Monitor(std::shared_ptr<Logger> logger, std::string comp_op, double thresh_val,
                 std::string thresh_unit, std::vector<double> dimensions)
{
  _comp_op = comp_op;
  _thresh_val = thresh_val;
  _thresh_unit = thresh_unit;

  _dimensions = dimensions;

  // fill _threshold_twist with thresh_val based on non-zero dimensions
  std::transform(_dimensions.begin(), _dimensions.begin() + 3, _threshold_twist.vel.data,
                 [&](double d) { return d == 0.0 ? 0.0 : thresh_val; });

  // TODO: handle orientation

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

  // make the angular error 0
  std::fill(error.rot.data, error.rot.data + 3, 0.0);

  return _checkAll(error);
}

bool Monitor::checkAll(KDL::Frame current, KDL::Frame target)
{
  auto error = KDL::diff(current, target);

  // make the angular error 0
  std::fill(error.rot.data, error.rot.data + 3, 0.0);

  return _checkAll(error);
}

bool Monitor::checkAny(KDL::Frame current)
{
  auto error = KDL::diff(current, *_target_frame);

  // make the angular error 0
  std::fill(error.rot.data, error.rot.data + 3, 0.0);

  return _checkAny(error);
}

bool Monitor::checkAny(KDL::Frame current, KDL::Frame target)
{
  auto error = KDL::diff(current, target);

  // make the angular error 0
  std::fill(error.rot.data, error.rot.data + 3, 0.0);

  return _checkAny(error);
}

bool Monitor::checkAll(KDL::Twist current)
{
  return _checkAll(current);
}

bool Monitor::checkAny(KDL::Twist current)
{
  return _checkAny(current);
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
  auto result = _math_utils->compare(error, _threshold_twist, _comp_op);

  return std::any_of(result.begin(), result.end(), [&](bool b) { return b; });
}

bool Monitor::_checkAll(KDL::Twist error)
{
  auto result = _math_utils->compare(error, _threshold_twist, _comp_op);

  return std::all_of(result.begin(), result.end(), [&](bool b) { return b; });
}



