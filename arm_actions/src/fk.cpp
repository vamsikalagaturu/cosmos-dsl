#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <filesystem>

void printLinkNames(KDL::Tree& tree)
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

void printJointNames(KDL::Chain& chain)
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

void computeFK(KDL::ChainFkSolverPos_recursive fk_solver, KDL::JntArray& q, KDL::Frame& tool_tip_frame)
{
    // calculate forward kinematics
    fk_solver.JntToCart(q, tool_tip_frame);

    // extract position from frame
    double x = tool_tip_frame.p.x();
    double y = tool_tip_frame.p.y();
    double z = tool_tip_frame.p.z();

    // Print the frame
    std::cout << "End-Effector frame\n";
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++) {
            double a = tool_tip_frame(i, j);
            if (a < 0.001 && a > -0.001) {
                a = 0.0;
            }
            std::cout << a << "\t";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl;

    // round the position to 4 decimal places
    x = round(x * 10000.0) / 10000.0;
    y = round(y * 10000.0) / 10000.0;
    z = round(z * 10000.0) / 10000.0;

    // Print the position
    std::cout << "End-Effector position\n";
    std::cout << "x: " << x << "\ty: " << y << "\tz: " << z << std::endl;

    std::cout << std::endl;

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

    // Print the orientation
    std::cout << "End-Effector orientation\n";
    std::cout << "roll: " << roll << "\tpitch: " << pitch << "\tyaw: " << yaw << std::endl;

}

int main()
{
    KDL::Tree my_tree;
    KDL::Chain my_chain;

    // get current file path
    std::filesystem::path path = __FILE__;

    std::string robot_urdf = (path.parent_path().parent_path() / "urdf" / "gen3_robotiq_2f_85.urdf").string();

    // load the robot urdf into the kdl tree
    if (!kdl_parser::treeFromFile(robot_urdf, my_tree))
    {
        std::cout << "Failed to construct kdl tree" << std::endl;
        return -1;
    }

    // set the base and tool frames
    std::string base_link = "base_link";
    std::string tool_frame = "tool_frame";

    // create the kdl chain
    if (!my_tree.getChain(base_link, tool_frame, my_chain))
    {
        std::cout << "Failed to get kdl chain" << std::endl;
        return -1;
    }

    // print the link names
    printLinkNames(my_tree);

    // print the joint names
    printJointNames(my_chain);

    // set the initial joint positions
    KDL::JntArray q(my_chain.getNrOfJoints());
    for (int i = 0; i < my_chain.getNrOfJoints(); i++)
    {
        q(i) = 0.0;
    }

    // create the forward kinematics solver
    KDL::ChainFkSolverPos_recursive fk_solver(my_chain);

    // create the frame that will contain the results
    KDL::Frame tool_tip_frame;

    // compute the forward kinematics
    computeFK(fk_solver, q, tool_tip_frame);

    // set the jonit_1 position to 90 degrees
    q(0) = M_PI;

    // compute the forward kinematics
    std::cout << "Joint 1 rotated" << std::endl;
    computeFK(fk_solver, q, tool_tip_frame);
    

    return 0;
}