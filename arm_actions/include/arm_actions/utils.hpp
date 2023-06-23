#ifndef UTILS_HPP
#define UTILS_HPP

#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "chain.hpp"
#include "frames.hpp"
#include "jacobian.hpp"
#include "jntarray.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "logger.hpp"
#include "tree.hpp"

class Utils
{
public:
  Utils(std::shared_ptr<Logger> logger);
  ~Utils();

  /**
   * @brief Prints the names of all links in a KDL::Tree.
   * @param tree The KDL::Tree object representing the robot's kinematic tree.
   */
  static void printLinkNames(KDL::Tree& tree);

  /**
   * @brief Prints the names of all links in a KDL::Chain.
   * @param chain The KDL::Chain object representing a subset of the robot's kinematic chain.
   */
  static void printLinkNamesFromChain(KDL::Chain& chain);

  /**
   * @brief Prints the names of all joints in a KDL::Chain.
   * @param chain The KDL::Chain object representing a subset of the robot's kinematic chain.
   */
  static void printJointNames(KDL::Chain& chain);

  /**
   * @brief Prints the elements of a vector.
   * @tparam T The type of the vector elements.
   * @param vec The vector to be printed.
   */
  template <typename T>
  static void printVec(const T& vec);

  /**
   * @brief Prints the elements of a KDL::JntArray.
   * @tparam T The type of the KDL::JntArray.
   * @param jntArr The KDL::JntArray to be printed.
   */
  template <typename T>
  static void printJntArr(const T& jntArr);

  /**
   * @brief Initializes the robot using a URDF file and sets the initial joint angles.
   *
   * @param urdf_path The file path to the robot's URDF description.
   * @param robot_chain [out] The KDL chain representing the robot.
   * @param base_link The name of the robot's base link.
   * @param tool_link The name of the robot's tool link.
   * @param initial_joint_angles A vector containing the initial joint angles.
   * @param q [out] The KDL joint array representing the robot's joint angles.
   * @param logger A pointer to a logger for logging errors and information.
   * @return 0 on success, -1 on failure.
   */
  int initialize_robot(const std::string& urdf_path, KDL::Chain& robot_chain,
                       const std::string& base_link, const std::string& tool_link,
                       const std::vector<double>& initial_joint_angles, KDL::JntArray& q);

private:
  std::shared_ptr<Logger> _logger;
};

#endif  // UTILS_HPP
