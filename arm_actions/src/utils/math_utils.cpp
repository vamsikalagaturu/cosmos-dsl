#include "arm_actions/math_utils.hpp"

MathUtils::MathUtils(double eq_tol): _eq_tol(eq_tol)
{
    std::cout << "mu eq_tol: " << _eq_tol << std::endl;
};

MathUtils::~MathUtils(){};

std::vector<bool> MathUtils::compare(KDL::Twist current_twist, KDL::Twist target_twist, std::string comp_op)
{
    // get the operator function
    std::function<bool(double, double)> op = comp_op_map.at(comp_op);

    std::vector<bool> result;

    // compare linear velocities
    for (int i = 0; i < 3; i++)
    {
        result.push_back(op(current_twist.vel[i], target_twist.vel[i]));
    }

    // TODO: compare angular velocities
    // for (int i = 0; i < 3; i++)
    // {
    //     result.push_back(op(current_twist.rot[i], target_twist.rot[i]));
    // }

    return result;
}

std::vector<bool> MathUtils::compare(KDL::Frame current_frame, KDL::Frame target_frame, std::string comp_op)
{
    // get the operator function
    std::function<bool(double, double)> op = comp_op_map.at(comp_op);

    std::vector<bool> result;

    // compare position
    for (int i = 0; i < 3; i++)
    {
        result.push_back(op(current_frame.p[i], target_frame.p[i]));
    }

    // compare orientation
    // TOOD: figure out how to compare orientations

    return result;
}

