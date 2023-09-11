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

#include "arm_actions/gnu_plotter.hpp"

GNUPlotter::GNUPlotter(std::string logs_dir, bool plot_data, bool save_data)
    : logs_dir_(logs_dir), save_data_(save_data)
{
  // create the logs directory if it doesn't exist
  if (!std::filesystem::exists(logs_dir))
  {
    std::filesystem::create_directory(logs_dir);
  }

  // get the number of subdirectories
  int n_subdirs = std::distance(std::filesystem::directory_iterator(logs_dir),
                                std::filesystem::directory_iterator{});

  // create the new subdirectory: 0 if no subdirectories exist, else n_subdirs + 1
  std::string subdir = logs_dir + "/" + std::to_string(n_subdirs ? n_subdirs : 0);

  // create the subdirectory
  std::filesystem::create_directory(subdir);

  // set the logs dir to the new subdirectory
  logs_dir_ = subdir;
}

GNUPlotter::~GNUPlotter() {}

std::string GNUPlotter::getNewFileName(std::string filename)
{
  // get current time
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);

  // convert to string
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_c), "%d_%m_%Y_%H_%M_%S");
  std::string time_str = ss.str();

  // create filename
  filename = logs_dir_ + "/" + filename + "_" + time_str + ".csv";

  return filename;
}

void GNUPlotter::saveDataToCSV(const std::vector<std::array<double, 3>>& positions,
                               const std::array<double, 3>& target_pos)
{
  // get the file
  std::string filename = getNewFileName("xyz_positions");

  // create the file
  std::ofstream file(filename);

  file << "Index,X,Y,Z,Target_X,Target_Y,Target_Z\n";
  for (size_t i = 0; i < positions.size(); ++i)
  {
    file << i << "," << positions[i][0] << "," << positions[i][1] << "," << positions[i][2] << ","
         << target_pos[0] << "," << target_pos[1] << "," << target_pos[2] << "\n";
  }

  file.close();
}

void GNUPlotter::testPlot()
{
  Gnuplot gp;

  // Generate test data
  std::vector<std::pair<double, double>> data;
  for (double x = 0; x <= 2 * M_PI; x += 0.1)
  {
    double y = std::sin(x);
    data.emplace_back(x, y);
  }

  // Plot the sine function
  gp << "set xlabel 'x'\n";
  gp << "set ylabel 'sin(x)'\n";
  gp << "plot '-' with lines title 'Sine function'\n";
  gp.send1d(data);
}

void GNUPlotter::plotXYZ(const std::vector<std::array<double, 3>>& positions,
                         const std::array<double, 3>& target_pos)
{
  // Save data to csv if save_data_ is true
  if (save_data_)
  {
    saveDataToCSV(positions, target_pos);
  }

  Gnuplot gp;
  gp << "set multiplot\n";
  gp << "set xlabel 'time'\n";

  // Prepare target_pos data
  std::vector<double> target_x(positions.size(), target_pos[0]);
  std::vector<double> target_y(positions.size(), target_pos[1]);
  std::vector<double> target_z(positions.size(), target_pos[2]);
  // std::vector<double> target_x(current_val.size(), target_val[0]);
  // std::vector<double> target_y(current_val.size(), target_val[1]);
  // std::vector<double> target_z(current_val.size(), target_val[2]);

  // Separate x, y, and z values from positions vector
  std::vector<double> x_values, y_values, z_values;
  for (const auto& pos : positions)
  {
    x_values.push_back(pos[0]);
    y_values.push_back(pos[1]);
    z_values.push_back(pos[2]);
  }

  // Plot X values
  gp << "set origin 0,0.66\n";
  gp << "set size 1,0.33\n";
  gp << "set ylabel 'X'\n";
  gp << "set ytics 0.01\n";
  gp << "plot '-' with lines title 'X values', '-' with lines title 'Target X'\n";
  gp.send1d(x_values);
  gp.send1d(target_x);

  // Plot Y values
  gp << "set origin 0,0.33\n";
  gp << "set size 1,0.33\n";
  gp << "set ylabel 'Y'\n";
  gp << "set ytics 0.01\n";
  gp << "plot '-' with lines title 'Y values', '-' with lines title 'Target Y'\n";
  gp.send1d(y_values);
  gp.send1d(target_y);

  // Plot Z values
  gp << "set origin 0,0\n";
  gp << "set size 1,0.33\n";
  gp << "set ylabel 'Z'\n";
  gp << "set ytics 0.01\n";
  gp << "plot '-' with lines title 'Z values', '-' with lines title 'Target Z'\n";
  gp.send1d(z_values);
  gp.send1d(target_z);

  gp << "unset multiplot\n";
}

// use with KDL
void GNUPlotter::saveDataToCSV(const std::vector<KDL::Vector>& current_val,
                               const KDL::Vector& target_val, std::string logname)
{
  // get the filefilename
  std::string filename = getNewFileName(logname);

  // create the file
  std::ofstream file(filename);

  file << "Index,X,Y,Z,Target_X,Target_Y,Target_Z\n";
  for (size_t i = 0; i < current_val.size(); ++i)
  {
    file << i << "," << current_val[i].x() << "," << current_val[i].y() << ","
         << current_val[i].z() << "," << target_val.x() << "," << target_val.y() << ","
         << target_val.z() << "\n";
  }

  file.close();
}

void GNUPlotter::plotXYZ(const std::vector<KDL::Vector>& current_val,
                         const KDL::Vector& target_val, std::string title, double ytick)
{
  std::vector<double> x_values, y_values, z_values;
  std::vector<double> target_x, target_y, target_z;
  if (save_data_)
  {
    saveDataToCSV(current_val, target_val, title + "_log");
  }

  for (const auto& curr : current_val)
  {
    x_values.push_back(curr.x());
    y_values.push_back(curr.y());
    z_values.push_back(curr.z());
  }

  // Prepare target_pos data
  for (size_t i = 0; i < current_val.size(); ++i)
  {
    target_x.push_back(target_val.x());
    target_y.push_back(target_val.y());
    target_z.push_back(target_val.z());
  }

  Gnuplot gp;
  gp << "set multiplot\n";
  gp << "set xlabel 'time'\n";

  // Plot X values
  gp << "set origin 0,0.66\n";
  gp << "set size 1,0.33\n";
  gp << "set ylabel 'X'\n";
  gp << "set ytics " << ytick << "\n";
  gp << "plot '-' with lines title 'X values', '-' with lines title '" << title << "X'\n";
  gp.send1d(x_values);
  gp.send1d(target_x);

  // Plot Y values
  gp << "set origin 0,0.33\n";
  gp << "set size 1,0.33\n";
  gp << "set ylabel 'Y'\n";
  gp << "set ytics " << ytick << "\n";
  gp << "plot '-' with lines title 'Y values', '-' with lines title '" << title << "Y'\n";
  gp.send1d(y_values);
  gp.send1d(target_y);

  // Plot Z values
  gp << "set origin 0,0\n";
  gp << "set size 1,0.33\n";
  gp << "set ylabel 'Z'\n";
  gp << "set ytics "
     << "0.02"
     << "\n";
  gp << "plot '-' with lines title 'Z values', '-' with lines title '" << title << "Z'\n";
  gp.send1d(z_values);
  gp.send1d(target_z);

  gp << "unset multiplot\n";
}

void GNUPlotter::saveDataToCSV(
    const std::vector<KDL::JntArray>& q, const std::vector<KDL::JntArray>& qdot,
    const std::vector<KDL::JntArray>& qddot, const std::vector<KDL::JntArray>& constraint_tau,
    const std::vector<KDL::Twist>& current_vel, const std::vector<KDL::Twist>& target_vel,
    const std::vector<KDL::Vector>& current_pos, const std::vector<KDL::Vector>& target_pos,
    const std::vector<KDL::JntArray>& control_signal, std::string logname)
{
  // get the filefilename
  std::string filename = getNewFileName(logname);

  // create the file
  std::ofstream file(filename);

  file << "Index,q1,q2,q3,q4,q5,q6,q7,qdot1,qdot2,qdot3,qdot4,qdot5,qdot6,qdot7,qddot1,qddot2,"
          "qddot3,qddot4,qddot5,qddot6,qddot7,constraint_tau1,constraint_tau2,constraint_tau3,"
          "constraint_tau4,constraint_tau5,constraint_tau6,constraint_tau7,current_vel_lin_x,"
          "current_vel_lin_y,current_vel_lin_z,current_vel_ang_x,current_vel_ang_y,"
          "current_vel_ang_z,target_vel_lin_x,target_vel_lin_y,target_vel_lin_z,target_vel_ang_x,"
          "target_vel_ang_y,target_vel_ang_z,current_pos_x,current_pos_y,current_pos_z,"
          "target_pos_x,target_pos_y,target_pos_z,control_signal1,control_signal2,control_signal3,"
          "control_signal4,control_signal5,control_signal6\n";

  for (size_t i = 0; i < q.size(); i++)
  {
    file << i << "," << q[i](0) << "," << q[i](1) << "," << q[i](2) << "," << q[i](3) << ","
         << q[i](4) << "," << q[i](5) << "," << q[i](6) << "," << qdot[i](0) << "," << qdot[i](1)
         << "," << qdot[i](2) << "," << qdot[i](3) << "," << qdot[i](4) << "," << qdot[i](5) << ","
         << qdot[i](6) << "," << qddot[i](0) << "," << qddot[i](1) << "," << qddot[i](2) << ","
         << qddot[i](3) << "," << qddot[i](4) << "," << qddot[i](5) << "," << qddot[i](6) << ","
         << constraint_tau[i](0) << "," << constraint_tau[i](1) << "," << constraint_tau[i](2)
         << "," << constraint_tau[i](3) << "," << constraint_tau[i](4) << ","
         << constraint_tau[i](5) << "," << constraint_tau[i](6) << "," << current_vel[i].vel.x()
         << "," << current_vel[i].vel.y() << "," << current_vel[i].vel.z() << ","
         << current_vel[i].rot.x() << "," << current_vel[i].rot.y() << ","
         << current_vel[i].rot.z() << "," << target_vel[i].vel.x() << "," << target_vel[i].vel.y()
         << "," << target_vel[i].vel.z() << "," << target_vel[i].rot.x() << ","
         << target_vel[i].rot.y() << "," << target_vel[i].rot.z() << "," << current_pos[i].x()
         << "," << current_pos[i].y() << "," << current_pos[i].z() << "," << target_pos[i].x()
         << "," << target_pos[i].y() << "," << target_pos[i].z() << "," << control_signal[i](0)
         << "," << control_signal[i](1) << "," << control_signal[i](2) << ","
         << control_signal[i](3) << "," << control_signal[i](4) << "," << control_signal[i](5)
         << "\n";
  }

  file.close();
}