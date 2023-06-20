#include "arm_actions/gnu_plotter.hpp"

GNUPlotter::GNUPlotter(std::string logs_dir, bool save_data) : logs_dir_(logs_dir), save_data_(save_data) {
    // create the logs directory if it doesn't exist
    if (!std::filesystem::exists(logs_dir)) {
        std::filesystem::create_directory(logs_dir);
    }

    // get the number of subdirectories
    int n_subdirs = std::distance(std::filesystem::directory_iterator(logs_dir), std::filesystem::directory_iterator{});

    // create the new subdirectory: 0 if no subdirectories exist, else n_subdirs + 1
    std::string subdir = logs_dir + "/" + std::to_string(n_subdirs ? n_subdirs : 0);

    // create the subdirectory
    std::filesystem::create_directory(subdir);

    // set the logs dir to the new subdirectory
    logs_dir_ = subdir;
}

GNUPlotter::~GNUPlotter() {}

std::string GNUPlotter::getNewFileName(std::string filename) {
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

void GNUPlotter::saveDataToCSV(const std::vector<std::array<double, 3>>& positions, const std::array<double, 3>& target_pos){
    // get the file
    std::string filename = getNewFileName("xyz_positions");

    // create the file
    std::ofstream file(filename);

    file << "Index,X,Y,Z,Target_X,Target_Y,Target_Z\n";
    for (size_t i = 0; i < positions.size(); ++i) {
        file << i << ","
             << positions[i][0] << ","
             << positions[i][1] << ","
             << positions[i][2] << ","
             << target_pos[0] << ","
             << target_pos[1] << ","
             << target_pos[2] << "\n";
    }

    file.close();
}

void GNUPlotter::testPlot() {
    Gnuplot gp;

    // Generate test data
    std::vector<std::pair<double, double>> data;
    for (double x = 0; x <= 2 * M_PI; x += 0.1) {
        double y = std::sin(x);
        data.emplace_back(x, y);
    }

    // Plot the sine function
    gp << "set xlabel 'x'\n";
    gp << "set ylabel 'sin(x)'\n";
    gp << "plot '-' with lines title 'Sine function'\n";
    gp.send1d(data);
}

void GNUPlotter::plotXYZ(const std::vector<std::array<double, 3>>& positions, const std::array<double, 3>& target_pos) {

    // Save data to csv if save_data_ is true
    if (save_data_) {
        saveDataToCSV(positions, target_pos);
    }

    Gnuplot gp;
    gp << "set multiplot\n";
    gp << "set xlabel 'time'\n";

    // Prepare target_pos data
    std::vector<double> target_x(positions.size(), target_pos[0]);
    std::vector<double> target_y(positions.size(), target_pos[1]);
    std::vector<double> target_z(positions.size(), target_pos[2]);

    // Separate x, y, and z values from positions vector
    std::vector<double> x_values, y_values, z_values;
    for (const auto& pos : positions) {
        x_values.push_back(pos[0]);
        y_values.push_back(pos[1]);
        z_values.push_back(pos[2]);
    }

    // Plot X values
    gp << "set origin 0,0.66\n";
    gp << "set size 1,0.33\n";
    gp << "set ylabel 'X'\n";
    gp << "set ytics 0.002\n";
    gp << "plot '-' with lines title 'X values', '-' with lines title 'Target X'\n";
    gp.send1d(x_values);
    gp.send1d(target_x);

    // Plot Y values
    gp << "set origin 0,0.33\n";
    gp << "set size 1,0.33\n";
    gp << "set ylabel 'Y'\n";
    gp << "set ytics 0.003\n";
    gp << "plot '-' with lines title 'Y values', '-' with lines title 'Target Y'\n";
    gp.send1d(y_values);
    gp.send1d(target_y);

    // Plot Z values
    gp << "set origin 0,0\n";
    gp << "set size 1,0.33\n";
    gp << "set ylabel 'Z'\n";
    gp << "set ytics 0.05\n";
    gp << "plot '-' with lines title 'Z values', '-' with lines title 'Target Z'\n";
    gp.send1d(z_values);
    gp.send1d(target_z);

    gp << "unset multiplot\n";
}