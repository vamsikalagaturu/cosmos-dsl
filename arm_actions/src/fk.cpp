#include "kdl/chain.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainidsolver.hpp"
#include "kdl/frames_io.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "chainhdsolver_vereshchagin.hpp"
#include <filesystem>
#include <gnuplot-iostream.h>

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

void populateAlphaUnitForces(const std::vector<double>& alpha_lin, const std::vector<double>& alpha_ang, KDL::Jacobian* alpha_unit_forces)
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

std::tuple<double, double, double> calc_dist(std::array<double, 3> p1, std::array<double, 3> p2)
{
    double dx = p1[0] - p2[0];
    double dy = p1[1] - p2[1];
    double dz = p1[2] - p2[2];
    return std::make_tuple(dx, dy, dz);
}

// print a vector
template <typename T>
void printVec(const T& vec)
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

// print a kdl jnt array
template <typename T>
void printJntArr(const T& jntArr)
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

    // number of segments
    // chain has 9 segments, but only 7 joints
    int n_segments = my_chain.getNrOfSegments();

    // set the initial joint positions
    KDL::JntArray q(n_joints);
    q(3) = -M_PI_2; // joint_4
    q(5) = -M_PI_2; // joint_6

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
    std::array<double, 3> target_position = {current_position[0] + 0.025, current_position[1] + 0.025, current_position[2] - 0.02};
    std::cout << "Target position: " << std::endl;
    std::cout << target_position[0] << ", " << target_position[1] << ", " << target_position[2] << std::endl;

    std::cout << std::endl;

    // number of constraints
    int n_constraints = 6;

    // root acceleration - set gravity vector
    // direction is specified opposite for the solver
    KDL::Twist root_acc(KDL::Vector(0.0, 0.0, 9.81), KDL::Vector(0.0, 0.0, 0.0));
    
    // define unit constraint forces for EE
    KDL::Jacobian alpha_unit_forces(n_constraints);

    std::vector<double> alpha_lin = {1.0, 1.0, 1.0};
    std::vector<double> alpha_ang = {1.0, 1.0, 1.0};

    populateAlphaUnitForces(alpha_lin, alpha_ang, &alpha_unit_forces);

    // beta - accel energy for EE
    KDL::JntArray beta_energy(n_constraints);
    beta_energy(0) = 1.0;
    beta_energy(1) = 0.0;
    beta_energy(2) = 9.81;
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
    
    // compute the inverse dynamics
    std::cout<<"starting solver..."<<std::endl;

    // time step
    double dt = 0.01;

    // PID gains
    double kp = 63.2;
    double ki = 15.2;
    double kd = 4.2;

    auto [prev_error_x, prev_error_y, prev_error_z] = calc_dist(current_position, target_position);

    // variables for tracking integral and previous error
    double integral_x = 0.0;
    double integral_y = 0.0;
    double integral_z = 0.0;

    double current_control_vel_x = 0.0;
    double current_control_vel_y = 0.0;
    double current_control_vel_z = 0.0;

    int i = 0;

    double eps = 0.001;

    // store all the current positions
    std::vector<std::array<double, 3>> positions;

    // Current position: 
    // -0.3144, -0.0249, 0.5996

    while(abs(prev_error_x) > eps || abs(prev_error_y) > eps || abs(prev_error_z) > eps)
    {
        // compute the inverse dynamics
        int sr = vereshchagin_solver.CartToJnt(q, qd, qdd, alpha_unit_forces, beta_energy, f_ext, ff_tau, constraint_tau);
        if (sr < 0) std::cout << "KDL: Vereshchagin solver ERROR: " << sr << std::endl;

        // set the constraint torques to feedforward torques
        // for (int j = 0; j < n_joints; j++)
        // {
        //     ff_tau(j) = constraint_tau(j);
        // }

        // update the joint positions and velocities by integrating the accelerations
        for (int j = 0; j < n_joints; j++)
        {
            q(j) = q(j) + qd(j) * dt + 0.5 * qdd(j) * dt * dt;
            qd(j) = qd(j) + qdd(j) * dt;
        }

        printf("Iteration: %d\n", i);

        // print the joint positions, velocities, aceelerations and constraint torques
        std::cout << "Joint accelerations: " << std::endl;
        printJntArr(qdd);

        std::cout << "Constraint torques: " << std::endl;
        printJntArr(constraint_tau);

        std::cout << "Joint positions: " << std::endl;
        printJntArr(q);

        std::cout << "Joint velocities: " << std::endl;
        printJntArr(qd);

        // create the frame that will contain the results
        KDL::Frame tool_tip_frame;

        // compute the forward kinematics
        std::tuple<std::array<double, 3>, std::array<double, 3>> fk_result = computeFK(fk_solver, q, tool_tip_frame);

        current_position = std::get<0>(fk_result);

        std::cout << "Current position: " << std::endl;
        std::cout << current_position[0] << " " << current_position[1] << " " << current_position[2] << std::endl;
        positions.push_back(current_position);

        // std::vector<KDL::Frame> frames;
        // vereshchagin_solver.getLinkCartesianPose(frames); TODO: check this

        std::cout << "Target position: " << std::endl;
        std::cout << target_position[0] << ", " << target_position[1] << ", " << target_position[2] << std::endl;

        auto [error_x, error_y, error_z] = calc_dist(current_position, target_position);
        
        printf("error: [%f, %f, %f]\n", error_x, error_y, error_z);

        // update the integral terms
        integral_x += error_x * dt;
        integral_y += error_y * dt;
        integral_z += error_z * dt;

        // compute the derivative terms
        double derivative_x = (error_x - prev_error_x) / dt;
        double derivative_y = (error_y - prev_error_y) / dt;
        double derivative_z = (error_z - prev_error_z) / dt;

        // differentiate the control signal to get the velocity
        double control_vel_x = (error_x - prev_error_x) / dt;
        double control_vel_y = (error_y - prev_error_y) / dt;
        double control_vel_z = (error_z - prev_error_z) / dt;

        // differentiate the control velocity to get the acceleration
        double control_acc_x = (control_vel_x - current_control_vel_x) / dt;
        double control_acc_y = (control_vel_y - current_control_vel_y) / dt;
        double control_acc_z = (control_vel_z - current_control_vel_z) / dt;

        // double control_signal_x = kp * control_acc_x;
        // double control_signal_y = kp * control_acc_y;
        // double control_signal_z = kp * control_acc_z;

        // calculate the control signals
        double control_signal_x = kp * error_x + ki * integral_x + kd * derivative_x;
        double control_signal_y = kp * error_y + ki * integral_y + kd * derivative_y;
        double control_signal_z = kp * error_z + ki * integral_z + kd * derivative_z;

        if (error_x > 0){
            control_signal_x = -abs(control_signal_x);
        } else {
            control_signal_x = abs(control_signal_x);
        }

        if (error_y > 0){
            control_signal_y = -abs(control_signal_y);
        } else {
            control_signal_y = abs(control_signal_y);
        }

        if (error_z > 0){
            control_signal_z = -abs(control_signal_z);
        } else {
            control_signal_z = abs(control_signal_z);
        }

        // update each beta energy based on the error
        if (abs(error_x) > eps)
        {
            beta_energy(0) = control_signal_x;
        } else {
            beta_energy(0) = 0.0;
        }

        if (abs(error_y) > eps)
        {
            beta_energy(1) = control_signal_y;
        } else {
            beta_energy(1) = 0.0;
        }

        if (abs(error_z) > eps)
        {
            beta_energy(2) = 9.81 + control_signal_z;
        } else {
            beta_energy(2) = 9.81;
        }

        printf("Control signal: [%f, %f, %f]\n", beta_energy(0), beta_energy(1), beta_energy(2));

        prev_error_x = error_x;
        prev_error_y = error_y;
        prev_error_z = error_z;

        // update the current control velocity
        current_control_vel_x = control_vel_x;
        current_control_vel_y = control_vel_y;
        current_control_vel_z = control_vel_z;

        std::cout << std::endl;

        i++;

        if (i > 100) break;
    }

    Gnuplot gp;

    // plot 3 graphs: x, y, z with target position and current position
    std::vector<std::pair<double, double>> x;
    std::vector<std::pair<double, double>> y;
    std::vector<std::pair<double, double>> z;

    for (int i = 0; i < positions.size(); i++)
    {
        x.push_back(std::make_pair(i, positions[i][0]));
        y.push_back(std::make_pair(i, positions[i][1]));
        z.push_back(std::make_pair(i, positions[i][2]));
    }

    gp << "set multiplot layout 3,1 title 'Position'\n";

    gp << "set ylabel 'x'\n";
    // Set the size of the graph
    gp << "set size 1, 0.4\n";
    gp << "set origin 0, 0.66\n";
    gp << "plot '-' with lines title 'position', '-' with lines title 'target position' lc rgb 'red'\n";
    gp.send1d(x);
    gp.send1d(std::vector<double>(positions.size(), target_position[0]));

    gp << "set ylabel 'y'\n";
    // Set the size of the graph
    gp << "set size 1, 0.4\n";
    gp << "set origin 0, 0.33\n";
    gp << "plot '-' with lines title 'position', '-' with lines title 'target position' lc rgb 'red'\n";
    gp.send1d(y);
    gp.send1d(std::vector<double>(positions.size(), target_position[1]));

    gp << "set xlabel 'time'\n";
    gp << "set ylabel 'z'\n";
    // Set the size of the graph
    gp << "set size 1, 0.4\n";
    gp << "set origin 0, 0.0\n";
    gp << "plot '-' with lines title 'position', '-' with lines title 'target position' lc rgb 'red'\n";
    gp.send1d(z);
    gp.send1d(std::vector<double>(positions.size(), target_position[2]));

    // Adjust the overall size and margins of the multiplot
    gp << "set size 1, 1\n";
    gp << "set origin 0, 0\n";
    gp << "unset multiplot\n";

    return 0;
}