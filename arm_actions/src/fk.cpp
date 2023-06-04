#include "kdl/chain.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainidsolver.hpp"
#include "kdl/frames_io.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "chainhdsolver_vereshchagin.hpp"
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

void printLinkNamesFromChain(KDL::Chain& chain)
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

std::tuple<std::array<double, 3>, std::array<double, 3>> computeFK(KDL::ChainFkSolverPos_recursive fk_solver, KDL::JntArray& q, KDL::Frame& tool_tip_frame)
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
};

double calc_dist(std::array<double, 3> p1, std::array<double, 3> p2)
{
    double dx = p1[0] - p2[0];
    double dy = p1[1] - p2[1];
    double dz = p1[2] - p2[2];
    return dx;
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
    std::string tool_frame = "bracelet_link";

    // create the kdl chain
    if (!my_tree.getChain(base_link, tool_frame, my_chain))
    {
        std::cout << "Failed to get kdl chain" << std::endl;
        return -1;
    }

    // print the link names
    printLinkNamesFromChain(my_chain);

    // print the joint names
    printJointNames(my_chain);
    
    // number of joints
    int n_joints = my_chain.getNrOfJoints();

    std::cout << "Number of joints: " << n_joints << std::endl;

    // number of segments
    // chain has 9 segments, but only 7 joints
    int n_segments = my_chain.getNrOfSegments();

    std::cout << "Number of segments: " << n_segments << std::endl;

    // set the initial joint positions
    KDL::JntArray q(n_joints);
    q(0) = 0.0; // joint_1
    q(1) = M_PI_4; // joint_2
    q(2) = 0.0; // joint_3
    q(3) = -M_PI_2; // joint_4
    q(4) = 0.0; // joint_5
    q(5) = M_PI_4; // joint_6
    q(6) = 0.0; // joint_7

    // create the forward kinematics solver
    KDL::ChainFkSolverPos_recursive fk_solver(my_chain);

    // create the frame that will contain the results
    KDL::Frame tool_tip_frame;

    // compute the forward kinematics
    std::tuple<std::array<double, 3>, std::array<double, 3>> fk_result = computeFK(fk_solver, q, tool_tip_frame);

    std::array<double, 3> current_position = std::get<0>(fk_result);

    std::cout << "Current position: " << std::endl;
    std::cout << current_position[0] << ", " << current_position[1] << ", " << current_position[2] << std::endl;

    // target position with z+0.1
    std::array<double, 3> target_position = {current_position[0] + 0.1, current_position[1], current_position[2]};

    std::cout << std::endl;

    // number of constraints
    int n_constraints = 6;

    // root acceleration - set gravity vector
    // direction is specified opposite for the solver
    KDL::Twist root_acc(KDL::Vector(0.0, 0.0, 9.81), KDL::Vector(0.0, 0.0, 0.0));
    
    // define unit constraint forces for EE
    KDL::Jacobian alpha_unit_forces(n_constraints);

    // set the unit forces for each direction
    KDL::Twist unit_force_x_l(
        KDL::Vector(1.0, 0.0, 0.0),
        KDL::Vector(0.0, 0.0, 0.0)
    );
    alpha_unit_forces.setColumn(0, unit_force_x_l);

    KDL::Twist unit_force_y_l(
        KDL::Vector(0.0, 1.0, 0.0),
        KDL::Vector(0.0, 0.0, 0.0)
    );
    alpha_unit_forces.setColumn(1, unit_force_y_l);

    KDL::Twist unit_force_z_l(
        KDL::Vector(0.0, 0.0, 1.0),
        KDL::Vector(0.0, 0.0, 0.0)
    );
    alpha_unit_forces.setColumn(2, unit_force_z_l);

    KDL::Twist unit_force_x_a(
        KDL::Vector(0.0, 0.0, 0.0),
        KDL::Vector(0.0, 0.0, 0.0)
    );
    alpha_unit_forces.setColumn(3, unit_force_x_a);

    KDL::Twist unit_force_y_a(
        KDL::Vector(0.0, 0.0, 0.0),
        KDL::Vector(0.0, 0.0, 0.0)
    );
    alpha_unit_forces.setColumn(4, unit_force_y_a);

    KDL::Twist unit_force_z_a(
        KDL::Vector(0.0, 0.0, 0.0),
        KDL::Vector(0.0, 0.0, 0.0)
    );
    alpha_unit_forces.setColumn(5, unit_force_z_a);

    // beta - accel energy for EE
    KDL::JntArray beta_energy(n_constraints);
    beta_energy(0) = 1.0;
    beta_energy(1) = 0.0;
    beta_energy(2) = 0.0;
    beta_energy(3) = 0.0;
    beta_energy(4) = 0.0;
    beta_energy(5) = 0.0;

    // use vereshchagin solver 
    KDL::ChainHdSolver_Vereshchagin vereshchagin_solver(my_chain, root_acc, n_constraints);

    // set the solver parameters
    KDL::JntArray qd(n_joints); // input joint velocities
    KDL::JntArray qdd(n_joints); // output joint accelerations
    KDL::JntArray ff_tau(n_joints); // input feedforward torques
    KDL::JntArray constraint_tau(n_joints); // output constraint torques

    KDL::Vector f(0.0, 0.0, 0.0);
    KDL::Vector n(0.0, 0.0, 0.0);
    KDL::Wrench f_tool(f, n);
    KDL::Wrenches f_ext(n_segments);
    f_ext[n_segments - 1] = f_tool; // input external forces at EE

    // set the initial joint velocities to zero
    for (int i = 0; i < n_joints; i++)
    {
        qd(i) = 0.0;
    }

    // set the feedforward torques to zero
    for (int i = 0; i < n_joints; i++)
    {
        ff_tau(i) = 0.0;
    }
    
    // compute the inverse dynamics
    std::cout<<"starting solver..."<<std::endl;

    // time step
    double dt = 0.01;

    // p gain for cartesian position control
    double kp = 1;

    double current_error = calc_dist(current_position, target_position);
    std::cout << "Current error: " << current_error << std::endl;

    double current_control_vel = 0.0;
    double current_control_acc;

    int i = 0;

    while(abs(current_error) > 0.01)
    {
        // compute the inverse dynamics
        int sr = vereshchagin_solver.CartToJnt(q, qd, qdd, alpha_unit_forces, beta_energy, f_ext, ff_tau, constraint_tau);
        if (sr < 0) std::cout << "KDL: Vereshchagin solver ERROR: " << sr << std::endl;

        // set the constraint torques to feedforward torques
        for (int j = 0; j < n_joints; j++)
        {
            ff_tau(j) = constraint_tau(j);
        }

        // update the joint positions and velocities by integrating the accelerations
        for (int j = 0; j < n_joints; j++)
        {
            q(j) = q(j) + qd(j) * dt + 0.5 * qdd(j) * dt * dt;
            qd(j) = qd(j) + qdd(j) * dt;
        }

        std::cout << std::endl;
        std::cout << "Time step: " << i << std::endl;

        // print the joint positions, velocities, aceelerations and constraint torques
        std::cout << "Joint accelerations: " << std::endl;
        for (int j = 0; j < n_joints; j++)
        {
            std::cout << qdd(j) << " ";
        }
        std::cout << std::endl;

        std::cout << "Constraint torques: " << std::endl;
        for (int j = 0; j < n_joints; j++)
        {
            std::cout << constraint_tau(j) << " ";
        }
        std::cout << std::endl;

        std::cout << "Joint positions: " << std::endl;
        for (int j = 0; j < n_joints; j++)
        {
            std::cout << q(j) << " ";
        }
        std::cout << std::endl;

        std::cout << "Joint velocities: " << std::endl;
        for (int j = 0; j < n_joints; j++)
        {
            std::cout << qd(j) << " ";
        }
        std::cout << std::endl;

        // create the frame that will contain the results
        KDL::Frame tool_tip_frame;

        // compute the forward kinematics
        std::tuple<std::array<double, 3>, std::array<double, 3>> fk_result = computeFK(fk_solver, q, tool_tip_frame);

        current_position = std::get<0>(fk_result);

        std::cout << "Current position: " << std::endl;
        std::cout << current_position[0] << " " << current_position[1] << " " << current_position[2] << std::endl;

        double new_error = calc_dist(current_position, target_position);
        std::cout << "Position error: " << new_error << std::endl;

        // differentiate the control signal to get the velocity
        double control_vel = (new_error - current_error) / dt;

        // differentiate the control velocity to get the acceleration
        double control_acc = (control_vel - current_control_vel) / dt;

        double control_signal = kp * control_acc;

        std::cout << "Control signal: " << control_signal << std::endl;

        // update the beta energy
        beta_energy(0) = control_signal;
        // beta_energy(1) = control_signal;
        // beta_energy(2) = control_signal;

        current_error = new_error;
        current_control_vel = control_vel;
        current_control_acc = control_acc;

        i++;

        if (i > 50) break;
    }

    return 0;
}