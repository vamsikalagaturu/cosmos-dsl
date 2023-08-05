#include "arm_actions/mujoco_env.hpp"

RobotSimulation* RobotSimulation::instance = nullptr;

// Constructor
RobotSimulation::RobotSimulation()
    : button_left(false), button_middle(false), button_right(false), lastx(0), lasty(0)
{
  // Get current file path
  path = __FILE__;
  // Get the robot urdf path
  robot_urdf = (path.parent_path().parent_path().parent_path() / "urdf" / "gen3.urdf").string();
  // Load the robot urdf into mujoco
  error = nullptr;
  model = mj_loadXML(robot_urdf.c_str(), nullptr, error, 1000);
  data = mj_makeData(model);
}

// Destructor
RobotSimulation::~RobotSimulation()
{
  glfwTerminate();
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  delete data;
  delete model;
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

int RobotSimulation::run(std::vector<KDL::JntArray> joint_angles,
                         std::vector<KDL::JntArray> joint_velocities,
                         std::vector<KDL::JntArray> joint_taus)
{
  instance = this;

  if (!glfwInit())
  {
    // Failed to initialize GLFW
    // Handle the error
  }

  // init GLFW, create window, make OpenGL context current, request v-sync
  glfwInit();
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

  double counter = 0.0;
  double rotation_speed = 0.1;

  std::vector initial_joint_angles = {0.0, 0.0, 0.0, M_PI_2, 0.0, M_PI_2, 0.0};

  // make the robot stay still
  for (int i = 0; i < model->nq; i++)
  {
    data->qpos[i] = initial_joint_angles[i];
    data->qvel[i] = 0.0;
  }

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window) && joint_angles.size() > 0)
  {
    // pop from front of vector
    KDL::JntArray joint_angle = joint_angles.front();
    // remove from front of vector
    joint_angles.erase(joint_angles.begin());
    KDL::JntArray joint_velocity = joint_velocities.front();
    joint_velocities.erase(joint_velocities.begin());
    KDL::JntArray joint_tau = joint_taus.front();
    joint_taus.erase(joint_taus.begin());

    // make the robot stay still
    for (int i = 0; i < model->nq; i++)
    {
      data->qpos[i] = joint_angle(i);
      data->qvel[i] = joint_velocity(i);
      // data->ctrl[i] = joint_tau(i);
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
  }

  return 0;
}

// int main()
// {
//   RobotSimulation simulation;
//   return simulation.run();
// }