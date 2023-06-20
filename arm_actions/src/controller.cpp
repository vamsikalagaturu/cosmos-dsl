#include "chain.hpp"
#include "chainfksolver.hpp"
#include "chainfksolverpos_recursive.hpp"
#include "chainidsolver.hpp"
#include "frames_io.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "chainhdsolver_vereshchagin.hpp"
#include <filesystem>
#include "arm_actions/logger.hpp"
// #include "arm_actions/gnu_plotter.hpp"
#include "arm_actions/pid_controller.hpp"
#include "arm_actions/utils.hpp"

int main()
{
    // initialize logger
    Logger logger(true, false);
    logger.test();
    // GNUPlotter plotter;

    // initialize utils
    Utils utils;

    KDL::Tree my_tree;
    KDL::Chain my_chain;

    // get current file path
    std::filesystem::path path = __FILE__;

    std::string robot_urdf = (path.parent_path().parent_path() / "urdf" / "gen3_robotiq_2f_85.urdf").string();

    // load the robot urdf into the kdl tree
    if (!kdl_parser::treeFromFile(robot_urdf, my_tree))
    {
        logger.logError("Failed to construct kdl tree");
        return -1;
    }

    // set the base and tool frames
    std::string base_link = "base_link";
    std::string tool_frame = "bracelet_link";

    // create the kdl chain
    if (!my_tree.getChain(base_link, tool_frame, my_chain))
    {
        logger.logError("Failed to get kdl chain");
        return -1;
    }
    
    // number of joints
    int n_joints = my_chain.getNrOfJoints();

    // number of segments
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
    std::tuple<std::array<double, 3>, std::array<double, 3>> fk_result = utils.computeFK(fk_solver, q, tool_tip_frame);

    std::array<double, 3> current_position = std::get<0>(fk_result);

    // target position
    std::array<double, 3> target_position = {current_position[0] + 0.01, current_position[1] + 0.01, current_position[2] + 0.1};

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

    utils.populateAlphaUnitForces(alpha_lin, alpha_ang, &alpha_unit_forces);

    // beta - accel energy for EE
    KDL::JntArray beta_energy(n_constraints);
    beta_energy(0) = 0.0;
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
    logger.logInfo("Starting solver...");

    // initialize the PID position controller
    PIDController pos_controller(5, 0.9, 0);
    PIDController vel_controller(30, 0.9, 0);

    // time step
    double dt = 0.001;

    int i = 0;

    // store all the current positions
    std::vector<std::array<double, 3>> positions;
    std::vector<std::array<double, 3>> velocities;

    double control_vel_x, control_vel_y, control_vel_z = 0.0;
    double control_acc_x, control_acc_y, control_acc_z = 0.0;

    // run the system at 1KHz
    while(true)
    {
        // compute the inverse dynamics
        int sr = vereshchagin_solver.CartToJnt(q, qd, qdd, alpha_unit_forces, beta_energy, f_ext, ff_tau, constraint_tau);
        if (sr < 0){
            logger.logError("KDL: Vereshchagin solver ERROR: %d", sr);
            return -1;
        }

        // update the joint positions and velocities by integrating the accelerations
        for (int j = 0; j < n_joints; j++)
        {
            q(j) = q(j) + qd(j) * dt + 0.5 * qdd(j) * dt * dt;
            qd(j) = qd(j) + qdd(j) * dt;
        }
    
        logger.logInfo("Iteration: %d", i);

        std::vector<KDL::Frame> frames(n_segments);
        vereshchagin_solver.getLinkCartesianPose(frames);

        // get the current position
        std::array<double, 3> current_position = {frames[n_segments - 1].p.x(), frames[n_segments - 1].p.y(), frames[n_segments - 1].p.z()};
        positions.push_back(current_position);

        std::vector<KDL::Twist> twists(n_segments);
        vereshchagin_solver.getLinkCartesianVelocity(twists);

        // get the current velocity
        std::array<double, 3> current_velocity = {twists[n_segments - 1].vel.x(), twists[n_segments - 1].vel.y(), twists[n_segments - 1].vel.z()};
        velocities.push_back(current_velocity);

        // print the joint positions, velocities, aceelerations and constraint torques
        logger.logInfo("Joint accelerations: %s", qdd);
        logger.logInfo("Joint torques: %s", constraint_tau);
        logger.logInfo("Joint velocities: %s", qd);
        logger.logInfo("Joint positions: %s", q);
        logger.logInfo("Target position: [%f, %f, %f]", target_position[0], target_position[1], target_position[2]);
        logger.logInfo("Current position: [%f, %f, %f]", current_position[0], current_position[1], current_position[2]);

        // call the position controller at frequency of 10hz
        if (i % 100 == 0){
            std::tie(control_vel_x, control_vel_y, control_vel_z) = pos_controller.computeControlSignal(current_position, target_position, 0.01);
        }

        // call the velocity controller at frequency of 100hz
        if (i % 10 == 0){
            std::tie(control_acc_x, control_acc_y, control_acc_z) = vel_controller.computeControlSignal(
                current_velocity,
                std::array<double, 3>{control_vel_x, control_vel_y, control_vel_z},
                0.1);
        }

        // update the beta energy
        beta_energy(0) = control_acc_x;
        beta_energy(1) = control_acc_y;
        beta_energy(2) = 9.81 + control_acc_z;

        logger.logInfo("Control acc: [%f, %f, %f]", control_acc_x, control_acc_y, control_acc_z);

        std::cout << std::endl;

        i++;

        if (i > 10) break;
    }

    // plotter.plotXYZ(positions, target_position);

    // // plot 3 graphs: x, y, z with target position and current position
    // std::vector<std::pair<double, double>> x;
    // std::vector<std::pair<double, double>> y;
    // std::vector<std::pair<double, double>> z;

    // for (int i = 0; i < positions.size(); i++)
    // {
    //     x.push_back(std::make_pair(i, positions[i][0]));
    //     y.push_back(std::make_pair(i, positions[i][1]));
    //     z.push_back(std::make_pair(i, positions[i][2]));
    // }

    // gp << "set multiplot layout 3,1 title 'Position'\n";

    // gp << "set ylabel 'x'\n";
    // // Set the size of the graph
    // gp << "set size 1, 0.4\n";
    // gp << "set origin 0, 0.66\n";
    // // gp << "set yrange [-1:1]\n";
    // gp << "plot '-' with lines title 'position', '-' with lines title 'target position' lc rgb 'red'\n";
    // gp.send1d(x);
    // gp.send1d(std::vector<double>(positions.size(), target_position[0]));

    // gp << "set ylabel 'y'\n";
    // // Set the size of the graph
    // gp << "set size 1, 0.4\n";
    // gp << "set origin 0, 0.33\n";
    // // gp << "set yrange [-1:1]\n";
    // gp << "plot '-' with lines title 'position', '-' with lines title 'target position' lc rgb 'red'\n";
    // gp.send1d(y);
    // gp.send1d(std::vector<double>(positions.size(), target_position[1]));

    // gp << "set xlabel 'time'\n";
    // gp << "set ylabel 'z'\n";
    // // Set the size of the graph
    // gp << "set size 1, 0.4\n";
    // gp << "set origin 0, 0.0\n";
    // // gp << "set yrange [-0.5:0.5]\n";
    // gp << "plot '-' with lines title 'position', '-' with lines title 'target position' lc rgb 'red'\n";
    // gp.send1d(z);
    // gp.send1d(std::vector<double>(positions.size(), target_position[2]));

    // // Adjust the overall size and margins of the multiplot
    // gp << "set size 1, 1\n";
    // gp << "set origin 0, 0\n";
    // gp << "unset multiplot\n";

    return 0;
}