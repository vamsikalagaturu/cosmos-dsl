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

enum ENV
{
  SIM,
  ROB
};

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
   * @brief Get the names of all links in a KDL::Chain.
   * @param chain The KDL::Chain object representing a subset of the robot's kinematic chain.
   * @return A vector containing the names of all links in the chain.
   */
  static std::vector<std::string> getLinkNamesFromChain(KDL::Chain& chain);

  /**
   * @brief Get the id of a link in a KDL::Chain.
   * @param chain The KDL::Chain object representing a subset of the robot's kinematic chain.
   * @param link_name The name of the link.
   * @return The id of the link.
   */
  static int getLinkIdFromChain(KDL::Chain& chain, const std::string& link_name);

  /**
   * @brief Check if the given link name is in the KDL::Chain.
   * @param chain The KDL::Chain object representing a subset of the robot's kinematic chain.
   */
  static bool checkLinkInChain(KDL::Chain& chain, const std::string& link_name);

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
   * @param env a ENV enum value representing the environment the robot is in.
   * @return 0 on success, -1 on failure.
   */
  int initialize_robot_urdf(const std::string& urdf_path, KDL::Chain& robot_chain,
                       const std::string& base_link, const std::string& tool_link,
                       KDL::JntArray& q,
                       const std::vector<double>& initial_joint_angles = std::vector<double>(),
                       ENV env = ENV::SIM);

  /**
   * @brief Computes the euclidean distance between two 3d points.
   * @param p1 The first point of type std::array<double, 3>.
   * @param p2 The second point of type std::array<double, 3>.
   * @return The euclidean distance between the two points.
   */
  static double computeEuclideanDistance(const std::array<double, 3>& current,
                                         const std::array<double, 3>& target);

  /**
   * @brief Calculates the error between two points.
   *
   * @param p1 The first point (x1, y1, z1).
   * @param p2 The second point (x2, y2, z2).
   * @return The error as a tuple (dx, dy, dz).
   */
  std::vector<double> calc_error(const std::array<double, 3>& p1, const std::array<double, 3>& p2);

  /**
   * @brief Calculates the error between two kdl vectors.
   * @param v1 The first vector.
   * @param v2 The second vector.
   * @return The error as a kdl vector.
   */
  KDL::Vector calc_error(const KDL::Vector& v1, const KDL::Vector& v2);

private:
  std::shared_ptr<Logger> _logger;
};

#endif  // UTILS_HPP
