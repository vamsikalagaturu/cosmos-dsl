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
  GNUPlotter(std::string logs_dir, bool save_data);

  ~GNUPlotter();

  // logs dir
  std::string logs_dir_;

  // save data to csv
  bool save_data_;

  // input: positions - std::vector<std::array<double, 3>>
  // input: target_pos = std::array<double, 3>
  void plotXYZ(const std::vector<std::array<double, 3>>& positions,
               const std::array<double, 3>& target_pos);

  void testPlot();

  std::string getNewFileName(std::string filename);

  void saveDataToCSV(const std::vector<std::array<double, 3>>& positions,
                     const std::array<double, 3>& target_pos);

private:
};

#endif  // GNU_PLOTTER_HPP