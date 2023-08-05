#include "arm_actions/math_utils.hpp"

MathUtils::MathUtils(double eq_tol): _eq_tol(eq_tol)
{
};

MathUtils::~MathUtils(){};

std::vector<bool> MathUtils::compare(KDL::Twist current_twist, double thresh, std::string comp_op)
{
    // get the operator function
    std::function<bool(double, double)> op = comp_op_map.at(comp_op);

    std::vector<bool> result;

    // compare linear velocities
    for (int i = 0; i < 3; i++)
    {
        // if infinities are involved, just return true
        if (std::isinf(current_twist.vel[i]) || std::isinf(thresh))
        {
            result.push_back(true);
            continue;
        }
        result.push_back(op(current_twist.vel[i], thresh));
    }

    // TODO: compare angular velocities
    for (int i = 0; i < 3; i++)
    {
        // if infinities are involved, just return true
        if (std::isinf(current_twist.rot[i]) || std::isinf(thresh))
        {
            result.push_back(true);
            continue;
        }
        result.push_back(op(current_twist.rot[i], thresh));
    }

    return result;
}

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
    for (int i = 0; i < 3; i++)
    {
        result.push_back(op(current_twist.rot[i], target_twist.rot[i]));
    }

    return result;
}

std::vector<bool> MathUtils::compare(KDL::Twist current_twist, KDL::Twist target_twist, double thresh, std::string comp_op)
{
    // get the operator function
    std::function<bool(double, double)> op = comp_op_map.at(comp_op);

    std::vector<bool> result;

    // compare linear velocities
    for (int i = 0; i < 3; i++)
    {
        if (std::isinf(target_twist.vel[i]) || std::isinf(thresh))
        {
            result.push_back(true);
            continue;
        }
        result.push_back(op(current_twist.vel[i] + thresh, target_twist.vel[i]));
    }

    // TODO: compare angular velocities
    for (int i = 0; i < 3; i++)
    {
        if (std::isinf(target_twist.rot[i]) || std::isinf(thresh))
        {
            result.push_back(true);
            continue;
        }
        result.push_back(op(current_twist.rot[i] + thresh, target_twist.rot[i]));
    }

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


KDL::Twist MathUtils::computeAverage(FixedSizeQueue<KDL::Twist, 10> *queue)
{
    KDL::Twist sum = KDL::Twist::Zero();
    for (int i = 0; i < queue->size(); i++)
    {
        sum += queue->at(i);
    }
    return sum / queue->size();
}
