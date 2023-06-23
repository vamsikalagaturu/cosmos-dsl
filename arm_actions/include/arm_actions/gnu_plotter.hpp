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

class GNUPlotter
{
public:
  // take logs dir as input to constructor
  GNUPlotter(std::string logs_dir, bool plot_data, bool save_data);

  ~GNUPlotter();

  // input: positions - std::vector<std::array<double, 3>>
  // input: target_pos = std::array<double, 3>
  void plotXYZ(const std::vector<std::array<double, 3>>& positions,
               const std::array<double, 3>& target_pos);

  void testPlot();

  std::string getNewFileName(std::string filename);

  void saveDataToCSV(const std::vector<std::array<double, 3>>& positions,
                     const std::array<double, 3>& target_pos);

private:
  // plot data
  bool plot_data_;
  // save data
  bool save_data_;
  // logs dir
  std::string logs_dir_;
};

#endif  // GNU_PLOTTER_HPP