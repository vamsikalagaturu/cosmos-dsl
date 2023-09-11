/**
 * Author: Vamsi Kalagaturu
 * Contributors: Ravisankar Selvaraju, Wing Ki Lau
 * 
 * Description: Library to plot KDL data variables using gnuplot
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

#ifndef GNU_PLOTTER_HPP
#define GNU_PLOTTER_HPP

#include <gnuplot-iostream.h>

#include <array>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "frames.hpp"
#include "frames_io.hpp"
#include "kinfam_io.hpp"

class GNUPlotter
{
public:
  // take logs dir as input to constructor
  GNUPlotter(std::string logs_dir, bool plot_data, bool save_data);

  ~GNUPlotter();

  void testPlot();
  std::string getNewFileName(std::string filename);

  // input: positions - std::vector<std::array<double, 3>>
  // input: target_pos = std::array<double, 3>
  void plotXYZ(const std::vector<std::array<double, 3>>& positions,
               const std::array<double, 3>& target_pos);
  void saveDataToCSV(const std::vector<std::array<double, 3>>& positions,
                     const std::array<double, 3>& target_pos);

  // Use with KDL
  // input: current_val - std::vector<KDL::Vector>
  // input: target_val - std::vector<KDL::Vector>
  // KDL::Vector contain - value.x(),value.y(),value.z()

  void plotXYZ(const std::vector<KDL::Vector>& current_val,
               const KDL::Vector& target_val, std::string title, double ytick = 0.01);
  void saveDataToCSV(const std::vector<KDL::Vector>& current_val,
                     const KDL::Vector& target_val, std::string logname);

  void saveDataToCSV(const std::vector<KDL::JntArray>& q,
                     const std::vector<KDL::JntArray>& qdot,
                     const std::vector<KDL::JntArray>& qddot,
                     const std::vector<KDL::JntArray>& constraint_tau,
                     const std::vector<KDL::Twist>& current_vel, 
                     const std::vector<KDL::Twist>& target_vel,
                     const std::vector<KDL::Vector>& current_pos, 
                     const std::vector<KDL::Vector>& target_pos,
                     const std::vector<KDL::JntArray>& control_signal,
                     std::string logname);


private:
  // plot data
  bool plot_data_;
  // save data
  bool save_data_;
  // logs dir
  std::string logs_dir_;
};

#endif  // GNU_PLOTTER_HPP