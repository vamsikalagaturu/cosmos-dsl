#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

int main()
{
    KDL::Tree my_tree;
    KDL::Chain my_chain;

    std::string robot_urdf = "/home/batsy/rnd/src/arm_actions/urdf/gen3_robotiq_2f_85.urdf";

    if (!kdl_parser::treeFromFile(robot_urdf, my_tree))
    {
        std::cout << "Failed to construct kdl tree" << std::endl;
        return -1;
    }

    // print joint names
    std::cout << "Joints and limits:" << std::endl;
    for (auto const& joint : my_tree.getSegments())
    {
        std::cout << joint.first << std::endl;
    }

    return 0;
}