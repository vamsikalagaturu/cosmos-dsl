#include "arm_actions/arm_actions.hpp"

RobotSimulation* RobotSimulation::instance = nullptr;

// Constructor
RobotSimulation::RobotSimulation()
    : button_left(false), button_middle(false), button_right(false), lastx(0), lasty(0)
{
  printf("Initializing robot simulation\n");
  // Get current file path
  path = __FILE__;
  // Get the robot urdf path
  robot_urdf = (path.parent_path().parent_path() / "urdf" / "gen3.urdf").string();
  // Load the robot urdf into mujoco
  error = nullptr;
  model = mj_loadXML(robot_urdf.c_str(), nullptr, error, 1000);
  data = mj_makeData(model);
}

// Destructor
RobotSimulation::~RobotSimulation()
{
  // glfwTerminate();
  // mjv_freeScene(&scn);
  // mjr_freeContext(&con);

  // delete data;
  // delete model;
}

// mouse button callback
void RobotSimulation::mouse_button(GLFWwindow* window, int button, int act, int mods)
{
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void RobotSimulation::mouse_move(GLFWwindow* window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right)
    return;

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right)
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if (button_left)
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
    action = mjMOUSE_ZOOM;

  // move camera
  mjv_moveCamera(model, action, dx / height, dy / height, &scn, &cam);
}

int RobotSimulation::simulate()
{
  instance = this;

  if (!glfwInit())
  {
    // Failed to initialize GLFW
    // Handle the error
  }

  // init GLFW, create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultPerturb(&pert);
  mjv_defaultOption(&opt);
  mjr_defaultContext(&con);

  // init MuJoCo rendering, get OpenGL info
  mjv_defaultScene(&scn);

  // set viwer camera position
  cam.type = mjCAMERA_FREE;
  cam.distance = 111.5;
  cam.azimuth = 10;
  cam.elevation = -10;
  cam.lookat[0] = 10.0;
  cam.lookat[1] = 20.0;
  cam.lookat[2] = 10.0;

  mjv_defaultCamera(&cam);

  mjv_makeScene(model, &scn, 1000);
  mjr_makeContext(model, &con, mjFONTSCALE_100);

  // add keyboard and mouse interaction
  glfwSetCursorPosCallback(window, static_mouse_move);
  glfwSetMouseButtonCallback(window, static_mouse_button);

  // --------------------------------------------------------------------
  // initialize config
  bool _debug = true;
  bool _visualize = true;
  bool _plot = true;
  ENV _env = ENV::SIM;

  // get current file path
  std::filesystem::path path = __FILE__;

  std::string log_path = path.parent_path().parent_path().parent_path().string() + "/logs/runs/";

  std::string data_path = path.parent_path().parent_path().parent_path().string() + "/logs/data/";

  // initialize logger
  std::shared_ptr<Logger> logger = std::make_shared<Logger>(true, false, log_path);

  // initialize plotter
  std::shared_ptr<GNUPlotter> plotter = std::make_shared<GNUPlotter>(data_path, true, false);

  // initialize utils
  std::shared_ptr<Utils> utils = std::make_shared<Utils>(logger);

  // initialize solver utils
  std::shared_ptr<SolverUtils> solver_utils = std::make_shared<SolverUtils>(logger);

  // initialize tf utils
  std::shared_ptr<TfUtils> tf_utils = std::make_shared<TfUtils>(logger);

  // get the robot urdf path
  std::string robot_urdf =
      (path.parent_path().parent_path() / "urdf" / "gen3_robotiq_2f_85.urdf").string();

  // set the base and tool links
  std::string base_link = "base_link";
  std::string tool_link = "bracelet_link";

  // initialize the kinova mediator to control the robot
  std::shared_ptr<kinova_mediator> km = std::make_shared<kinova_mediator>();

  // initialize the chain
  // define the robot chain
  KDL::Chain robot_chain;

  int r = utils->initialize_robot_urdf(robot_urdf, robot_chain, base_link, tool_link);

  if (r != 0)
  {
    logger->logError("Failed to initialize robot");
    return -1;
  }

  // number of joints
  int n_joints = robot_chain.getNrOfJoints();

  // number of segments
  int n_segments = robot_chain.getNrOfSegments();

  // set chain for the tf utils
  tf_utils->setChain(&robot_chain);

  // set the solver parameters
  KDL::JntArray q(n_joints);                // input joint positions
  KDL::JntArray qd(n_joints);               // input joint velocities
  KDL::JntArray qdd(n_joints);              // output joint accelerations
  KDL::JntArray joint_torques_m(n_joints);  // measured joint torques
  KDL::Wrench f_tool_m;                     // measured tool wrench

  std::vector<double> initial_joint_angles;

  // define the initial joint angles
  if (_env == ENV::SIM)
  {
    initial_joint_angles = {0.0, 0.26, 0.0, 2.26, 0.0, -0.95, -1.57};

    // initialize the robot joint angles
    int r = utils->init_q(&robot_chain, q, initial_joint_angles, _env);

    if (r != 0)
    {
      logger->logError("Failed to initialize robot");
      return -1;
    }
  }

  // coords
  std::string bracelet_link = "bracelet_link";
  KDL::Frame bracelet_link_coord_frame;
  if (utils->checkLinkInChain(robot_chain, bracelet_link))
  {
    int seg_n = utils->getLinkIdFromChain(robot_chain, bracelet_link);
    bracelet_link_coord_frame = solver_utils->computeFKFrame(&robot_chain, q, seg_n + 1);
  }
  else
  {
    logger->logError("Not implemented");
    return -1;
  }

  // TODO: for now, we assume initial twist is zero
  KDL::Twist bracelet_link_coord_twist;

  // initialize the PID controllers

  PIDController pid_move_arm_down_lin_vel_controller(20.0, 0.9, 0.0, 1.0);

  PIDController pid_move_arm_down_ang_vel_controller(20.0, 0.9, 0.0, 1.0);

  // initialize solver params
  // define unit constraint forces for EE
  KDL::Jacobian alpha_unit_forces;

  // beta - accel energy for EE
  KDL::JntArray beta_energy;

  // set the solver parameters
  KDL::JntArray ff_tau(n_joints);          // input feedforward torques
  KDL::JntArray constraint_tau(n_joints);  // output constraint torques
  KDL::Wrenches f_ext(n_segments);         // external forces at each segment

  // initialize the solver weights
  int solver_nc = 6;

  std::vector<std::vector<double>> solver_alpha_weights;

  std::vector<double> solver_alpha_col_1 = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  solver_alpha_weights.push_back(solver_alpha_col_1);
  std::vector<double> solver_alpha_col_2 = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  solver_alpha_weights.push_back(solver_alpha_col_2);
  std::vector<double> solver_alpha_col_3 = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  solver_alpha_weights.push_back(solver_alpha_col_3);
  std::vector<double> solver_alpha_col_4 = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  solver_alpha_weights.push_back(solver_alpha_col_4);
  std::vector<double> solver_alpha_col_5 = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  solver_alpha_weights.push_back(solver_alpha_col_5);
  std::vector<double> solver_alpha_col_6 = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  solver_alpha_weights.push_back(solver_alpha_col_6);

  std::vector<double> solver_beta_weights = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};

  // initialize vereshchagin solver
  KDL::ChainHdSolver_Vereshchagin vereshchagin_solver = solver_utils->initializeVereshchaginSolver(
      &robot_chain, solver_nc, solver_alpha_weights, alpha_unit_forces, solver_beta_weights,
      beta_energy, qd, qdd, ff_tau, constraint_tau, f_ext);

  // time step
  double dt = 0.001;

  // make the robot stay still
  for (int i = 0; i < model->nq; i++)
  {
    data->qpos[i] = initial_joint_angles[i];
    data->qvel[i] = 0.0;
  }

  int i = 0;

  // initialize the control acceleration
  KDL::JntArray control_accelerations(6);

  std::vector<KDL::Twist> twists(n_segments);

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window))
  {
    if (_debug)
      logger->logInfo("Iteration: %d", i);

    // update q and qd
    for (int i = 0; i < model->nq; i++)
    {
      q(i) = data->qpos[i];
      qd(i) = data->qvel[i];
    }

    if (i % 1 == 0)
    {
      int seg_n = utils->getLinkIdFromChain(robot_chain, bracelet_link);

      // get the current tool cartesian velocity
      KDL::Twist bracelet_link_coord_twist = twists[seg_n];

      auto cid = std::vector<int>{1, 1, 1};

      KDL::Twist control_twist;

      auto vsp = KDL::Vector{0.0, 0.0, -0.05};

      for (int j = 0; j < 3; j++)
      {
        if (cid[j] == 1)
        {
          control_twist.vel(j) = vsp[j];
        }
      }

      auto pid_move_arm_down_lin_vel_controller_io_output =
          pid_move_arm_down_lin_vel_controller.computeControlSignal_3d(
              bracelet_link_coord_twist.vel, control_twist.vel);
      auto cod = std::vector<int>{1, 1, 1};
      for (int j = 0; j < cod.size(); j++)
      {
        if (cod[j] == 1)
        {
          control_accelerations(j) = pid_move_arm_down_lin_vel_controller_io_output(j);
        }
      }
    }

    if (i % 1 == 0)
    {
      int seg_n = utils->getLinkIdFromChain(robot_chain, bracelet_link);

      // get the current tool cartesian velocity
      KDL::Twist bracelet_link_coord_twist = twists[seg_n];

      auto cid = std::vector<int>{1, 1, 1};

      KDL::Twist control_twist;

      auto vsp = KDL::Vector{0.0, 0.0, 0.0};

      for (int j = 0; j < 3; j++)
      {
        if (cid[j] == 1)
        {
          control_twist.rot(j) = vsp[j];
        }
      }

      auto pid_move_arm_down_ang_vel_controller_io_output =
          pid_move_arm_down_ang_vel_controller.computeControlSignal_3d(
              bracelet_link_coord_twist.rot, control_twist.rot);

      auto cod = std::vector<int>{1, 1, 1};
      for (int j = 0; j < cod.size(); j++)
      {
        if (cod[j] == 1)
        {
          control_accelerations(j + 3) = pid_move_arm_down_ang_vel_controller_io_output(j);
        }
      }
    }

    // update the default beta values with the controller values
    std::vector<double> solver_beta_weights = {0.0, 0.0, -9.81, 0.0, 0.0, 0.0};
    for (int j = 0; j < solver_beta_weights.size(); j++)
    {
      beta_energy(j) = solver_beta_weights[j];// + control_accelerations(j);
    }

    if (_debug)
      logger->logInfo("Beta energy: %s", beta_energy);

    // compute the inverse dynamics
    int sr = vereshchagin_solver.CartToJnt(q, qd, qdd, alpha_unit_forces, beta_energy, f_ext,
                                           ff_tau, constraint_tau);
    if (sr < 0)
    {
      logger->logError("KDL: Vereshchagin solver ERROR: %d", sr);
      return -1;
    }
    // if (_env == ENV::SIM) solver_utils->updateQandQd(q, qd, &qdd, dt);

    // get the current link cartesian velocities
    vereshchagin_solver.getLinkCartesianVelocity(twists);

    // make the robot stay still
    for (int i = 0; i < model->nq; i++)
    {
      data->qfrc_applied[i] = constraint_tau(i);
    }

    // advance simulation
    mjtNum simstart = data->time;
    while (data->time - simstart < 1.0 / 60.0)
      mj_step(model, data);

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // visualize frames
    opt.frame = mjtFrame::mjFRAME_GEOM;

    // update scene and render
    mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

    i++;
  }

  return 0;
};

int main()
{
  RobotSimulation robot_simulation;

  robot_simulation.simulate();

  return 0;
}