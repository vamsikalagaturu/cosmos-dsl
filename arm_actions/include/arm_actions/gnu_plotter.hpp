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