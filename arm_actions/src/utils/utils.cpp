#include "arm_actions/utils.hpp"

void Utils::printLinkNames(KDL::Tree& tree)
{
    std::vector<std::string> link_names;

    // get all segments
    KDL::SegmentMap segments = tree.getSegments();

    // iterate through all segments
    for (auto segment : segments)
    {
        // get the name of the segment
        std::string name = segment.first;

        // add the name to the vector
        link_names.push_back(name);
    }

    // print the link names
    std::cout << "Link names: " << std::endl;
    for (int i = 0; i < link_names.size(); i++)
    {
        std::cout << link_names[i] << std::endl;
    }
    std::cout << std::endl;
}

void Utils::printLinkNamesFromChain(KDL::Chain& chain)
{
    std::vector<std::string> link_names;
    for (int i = 0; i < chain.getNrOfSegments(); i++)
    {
        link_names.push_back(chain.getSegment(i).getName());
    }

    std::cout << "Link names: " << std::endl;
    for (int i = 0; i < link_names.size(); i++)
    {
        std::cout << link_names[i] << std::endl;
    }
    std::cout << std::endl;
}

void Utils::printJointNames(KDL::Chain& chain)
{
    std::vector<std::string> joint_names;
    for (int i = 0; i < chain.getNrOfSegments(); i++)
    {
        joint_names.push_back(chain.getSegment(i).getJoint().getName());
    }

    std::cout << "Joint names: " << std::endl;
    for (int i = 0; i < joint_names.size(); i++)
    {
        std::cout << joint_names[i] << std::endl;
    }
    std::cout << std::endl;
}

std::tuple<std::array<double, 3>, std::array<double, 3>> Utils::computeFK(KDL::ChainFkSolverPos_recursive fk_solver, KDL::JntArray& q, KDL::Frame& tool_tip_frame)
{
    // calculate forward kinematics
    int r = fk_solver.JntToCart(q, tool_tip_frame);
    if (r < 0) {
        std::cout << "Failed to compute forward kinematics with error: " << r << std::endl;
        return std::make_tuple(std::array<double, 3>{0, 0, 0}, std::array<double, 3>{0, 0, 0});
    }

    // extract position from frame
    double x = tool_tip_frame.p.x();
    double y = tool_tip_frame.p.y();
    double z = tool_tip_frame.p.z();

    // round the position to 4 decimal places
    x = round(x * 10000.0) / 10000.0;
    y = round(y * 10000.0) / 10000.0;
    z = round(z * 10000.0) / 10000.0;

    // convert the orientation to roll, pitch, yaw
    double roll, pitch, yaw;
    tool_tip_frame.M.GetRPY(roll, pitch, yaw);

    // convert the angles to degrees
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;

    // round the angles to 2 decimal places
    roll = round(roll * 100.0) / 100.0;
    pitch = round(pitch * 100.0) / 100.0;
    yaw = round(yaw * 100.0) / 100.0;

    // return the position and RPY values
    std::array<double, 3> position = {x, y, z};
    std::array<double, 3> rpy = {roll, pitch, yaw};
    return std::make_tuple(position, rpy);
}

void Utils::populateAlphaUnitForces(const std::vector<double>& alpha_lin, const std::vector<double>& alpha_ang, KDL::Jacobian* alpha_unit_forces)
{
    if (alpha_lin.size() != 3 || alpha_ang.size() != 3)
    {
        // Invalid input size, handle the error accordingly
        return;
    }

    KDL::Twist unit_force_x_l(
        KDL::Vector(alpha_lin[0], 0.0, 0.0),
        KDL::Vector(0.0, 0.0, 0.0)
    );
    alpha_unit_forces->setColumn(0, unit_force_x_l);

    KDL::Twist unit_force_y_l(
        KDL::Vector(0.0, alpha_lin[1], 0.0),
        KDL::Vector(0.0, 0.0, 0.0)
    );
    alpha_unit_forces->setColumn(1, unit_force_y_l);

    KDL::Twist unit_force_z_l(
        KDL::Vector(0.0, 0.0, alpha_lin[2]),
        KDL::Vector(0.0, 0.0, 0.0)
    );
    alpha_unit_forces->setColumn(2, unit_force_z_l);

    KDL::Twist unit_force_x_a(
        KDL::Vector(0.0, 0.0, 0.0),
        KDL::Vector(alpha_ang[0], 0.0, 0.0)
    );
    alpha_unit_forces->setColumn(3, unit_force_x_a);

    KDL::Twist unit_force_y_a(
        KDL::Vector(0.0, 0.0, 0.0),
        KDL::Vector(0.0, alpha_ang[1], 0.0)
    );
    alpha_unit_forces->setColumn(4, unit_force_y_a);

    KDL::Twist unit_force_z_a(
        KDL::Vector(0.0, 0.0, 0.0),
        KDL::Vector(0.0, 0.0, alpha_ang[2])
    );
    alpha_unit_forces->setColumn(5, unit_force_z_a);
}

template <typename T>
void Utils::printVec(const T& vec)
{
    std::cout << "[";
    for (int i = 0; i < vec.size(); ++i)
    {
        std::cout << vec[i] << ", ";
        if (i != vec.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}

template <typename T>
void Utils::printJntArr(const T& jntArr)
{
    std::cout << "[";
    for (int i = 0; i < jntArr.rows(); ++i)
    {
        std::cout << jntArr(i);
        if (i != jntArr.rows() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}
