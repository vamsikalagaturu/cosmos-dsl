#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include <vector>
#include <array>
#include <iostream>
#include "tree.hpp"
#include "chain.hpp"
#include "frames.hpp"
#include "jntarray.hpp"
#include "jacobian.hpp"
#include "chainfksolver.hpp"
#include "chainfksolverpos_recursive.hpp"

class Utils
{
public:
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
     * @brief Computes the forward kinematics (position and orientation) of a robot given joint positions.
     * @param fk_solver The KDL::ChainFkSolverPos_recursive object for calculating forward kinematics.
     * @param q The KDL::JntArray object representing the joint positions.
     * @param tool_tip_frame The KDL::Frame object to store the computed tool tip frame.
     * @return A tuple containing the position (x, y, z) and orientation (roll, pitch, yaw) of the tool tip.
     */
    static std::tuple<std::array<double, 3>, std::array<double, 3>> computeFK(KDL::ChainFkSolverPos_recursive fk_solver,
                                                                               KDL::JntArray& q,
                                                                               KDL::Frame& tool_tip_frame);

    /**
     * @brief Populates the columns of a Jacobian matrix with alpha unit forces.
     * @param alpha_lin A vector containing the linear components of the alpha unit forces (size 3).
     * @param alpha_ang A vector containing the angular components of the alpha unit forces (size 3).
     * @param alpha_unit_forces The KDL::Jacobian object to store the alpha unit forces.
     */
    static void populateAlphaUnitForces(const std::vector<double>& alpha_lin,
                                        const std::vector<double>& alpha_ang,
                                        KDL::Jacobian* alpha_unit_forces);

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
};

#endif  // UTILS_HPP
