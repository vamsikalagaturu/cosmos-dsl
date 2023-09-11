/**
 * Author: Vamsi Kalagaturu
 * 
 * Description: Library to log messages to terminal and/or file.
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

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <array>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "jntarray.hpp"
#include "kinfam_io.hpp"

class Logger
{
public:
  enum LogLevel
  {
    INFO,
    WARNING,
    ERROR
  };

  Logger(bool log_to_terminal, bool log_to_file, std::string logs_dir, bool show_date = false);

  ~Logger();

  void test();

  template <typename T, size_t N>
  void logInfo(const std::array<T, N>& arr);

  template <typename T>
  void logInfo(const std::vector<T>& vec);

  template <typename T>
  void logInfo(const std::vector<std::vector<T>>& vec_of_vec);

  void logInfo(const KDL::JntArray& jnt_array);

  void logInfo(const KDL::Vector& vector);

  void logInfo(const KDL::Twist& twist);

  void logInfo(const KDL::Jacobian& jacobian);

  template <typename... Args>
  void logInfo(const char* format, Args... args);

  template <typename T>
  void logInfo(const char* format, const std::vector<T>& vec);

  template <typename T, std::size_t N>
  void logInfo(const char* format, const std::array<T, N>& arr);

  void logInfo(const char* format, const KDL::JntArray& jnt_array);

  void logInfo(const char* format, const KDL::Vector& vector);

  void logInfo(const char* format, const KDL::Twist& twist);

  void logInfo(const char* format, const KDL::Jacobian& jacobian);

  // logwarning
  template <typename... Args>
  void logWarning(const char* format, Args... args);

  template <typename T, size_t N>
  void logWarning(const std::array<T, N>& arr);

  template <typename T>
  void logWarning(const std::vector<T>& vec);

  template <typename T>
  void logWarning(const std::vector<std::vector<T>>& vec_of_vec);

  void logWarning(const KDL::JntArray& jnt_array);

  template <typename T>
  void format_helper(std::stringstream& ss, const std::string& format, T arg);

  template <typename... Args>
  std::string string_format(const std::string& format, Args... args);

  template <typename T>
  void logWarning(const char* format, const std::vector<T>& vec);

  template <typename T, std::size_t N>
  void logWarning(const char* format, const std::array<T, N>& arr);

  void logWarning(const char* format, const KDL::JntArray& jnt_array);

  // logerror
  template <typename T, size_t N>
  void logError(const std::array<T, N>& arr);

  template <typename T>
  void logError(const std::vector<T>& vec);

  template <typename T>
  void logError(const std::vector<std::vector<T>>& vec_of_vec);

  void logError(const KDL::JntArray& jnt_array);

  template <typename... Args>
  void logError(const char* format, Args... args);

  template <typename T>
  void logError(const char* format, const std::vector<T>& vec);

  template <typename T, std::size_t N>
  void logError(const char* format, const std::array<T, N>& arr);

  void logError(const char* format, const KDL::JntArray& jnt_array);

private:
  bool log_to_terminal_;
  bool log_to_file_;
  bool show_date_;
  std::string logs_dir_;
  std::ofstream log_file_;

  std::string currentTimestamp();

  template <typename... Args>
  void logFormattedMessage(LogLevel level, const std::string& format, Args... args);

  void logMessage(Logger::LogLevel level, const std::string& msg);
};

namespace LoggerUtil
{
template <typename T>
std::string containerToString(const T& container);

template <typename T>
std::string vectorOfVectorToString(const std::vector<std::vector<T>>& vec_of_vec);

std::string jntArrayToString(const KDL::JntArray& jnt_array);
}  // namespace LoggerUtil

#endif  // LOGGER_HPP
