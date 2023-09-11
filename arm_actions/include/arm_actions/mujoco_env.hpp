/**
 * Author: Vamsi Kalagaturu
 * Contributors: Ravisankar Selvaraju
 * 
 * Description: Library to simulate the kinova robot in mujoco
 *
 * Copyright (c) [2023]
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#ifndef MUJOCO_HPP
#define MUJOCO_HPP

#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <filesystem>
#include <iostream>

#include <jntarray.hpp>
#include <vector>

class RobotSimulation
{
private:
  // Member variables
  bool button_left;
  bool button_middle;
  bool button_right;
  double lastx;
  double lasty;

  std::filesystem::path path;
  std::string robot_urdf;
  char* error;
  mjModel* model;
  mjData* data;
  mjvScene scn;
  mjvCamera cam;
  mjvOption opt;
  mjvPerturb pert;
  mjvFigure fig;
  mjrContext con;

public:
  // Constructor
  RobotSimulation();

  // Destructor
  ~RobotSimulation();

  // Methods
  void mouse_button(GLFWwindow* window, int button, int act, int mods);
  void mouse_move(GLFWwindow* window, double xpos, double ypos);

  static void static_mouse_move(GLFWwindow* window, double xpos, double ypos)
  {
    instance->mouse_move(window, xpos, ypos);
  }

  static void static_mouse_button(GLFWwindow* window, int button, int act, int mods)
  {
    instance->mouse_button(window, button, act, mods);
  }

  static RobotSimulation* instance;
  
  int run(std::vector<double> *initial_joint_angles, std::vector<KDL::JntArray> joint_angles, std::vector<KDL::JntArray> joint_velocities);

  int simulate();
};

#endif  // MUJOCO_HPP
