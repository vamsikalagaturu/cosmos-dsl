#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "frames.hpp"

class MathUtils
{
public:
  MathUtils(double eq_tol = 0.0001);

  ~MathUtils();

  /**
   * @brief compare Twists
   * @param current_twist a KDL::Twist
   * @param target_twist a KDL::Twist
   * @param comp_op comparison operator string
   * @return a vector of bools, one for each element of the twist
   */
  std::vector<bool> compare(KDL::Twist current_twist, KDL::Twist target_twist,
                            std::string comp_op);

  /**
   * @brief compare Frames
   * @param current_frame a KDL::Frame
   * @param target_frame a KDL::Frame
   * @param comp_op comparison operator string
   * @return a vector of bools, one for each element of the frame
   */
  std::vector<bool> compare(KDL::Frame current_frame, KDL::Frame target_frame,
                            std::string comp_op);

private:
  double _eq_tol;
  // Functions for different comparison operators

  /**
   * @brief checks if a < b
   * @param a
   * @param b
   */
  struct LessThan
  {
    template <typename T>
    bool operator()(T const &a, T const &b) const
    {
      return fabs(a) < fabs(b);
    }
  };

  /**
   * @brief checks if a > b
   * @param a
   * @param b
   */
  struct GreaterThan
  {
    template <typename T>
    bool operator()(T const &a, T const &b) const
    {
      return fabs(a) > fabs(b);
    }
  };

  /**
   * @brief checks if a == b (within a tolerance)
   * @param a
   * @param b
   */
  struct EqualTo
  {
    EqualTo(double eq_tol) : _eq_tol(eq_tol) {}
    template <typename T>
    bool operator()(T const &a, T const &b) const
    {
      return fabs(a - b) < _eq_tol;
    }

  private:
    double _eq_tol;
  };

  /**
   * @brief maps comparison operator strings to their corresponding functions
   * @param comp_op comparison operator string
   * @return function corresponding to comp_op
   */
  const std::map<std::string, std::function<bool(double, double)>> comp_op_map = {
      {"lt", LessThan()}, {"gt", GreaterThan()}, {"eq", EqualTo(_eq_tol)}};
};

#endif  // MATH_UTILS_HPP