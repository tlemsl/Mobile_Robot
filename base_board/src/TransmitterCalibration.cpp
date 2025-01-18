#include <yaml-cpp/yaml.h>

// C++ system headers
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <numeric>

// Project headers
#include "BaseBoardHandler.h"

namespace {
constexpr double kTimeInterval = 0.005;  // 200Hz
constexpr double kMinDuration = 2.0;
constexpr double kMaxDuration = 10.0;
constexpr double kStabilityTolerance = 1.0;
constexpr uint32_t kMaxUint32 = std::numeric_limits<uint32_t>::max();

// Helper function to calculate standard deviation
double CalculateStdDev(const std::deque<uint32_t>& values, double mean) {
  if (values.empty()) return 0.0;
  double sum_squares = std::accumulate(values.begin(), values.end(), 0.0,
                                       [mean](double sum, uint32_t value) {
                                         return sum + std::pow(value - mean, 2);
                                       });
  return std::sqrt(sum_squares / values.size());
}

// Helper function to calibrate a single channel
bool CalibrateChannel(BaseBoardHandler* /* handler */,
                      const std::string& channel_name,
                      std::function<uint32_t()> get_value, uint32_t* result,
                      int min_samples) {
  std::deque<uint32_t> readings;
  bool is_stable = false;
  double mean = 0.0;
  double std_dev = 0.0;

  std::cout << "Calibrating " << channel_name << "..." << std::endl;

  for (int i = 0; i < static_cast<int>(kMaxDuration / kTimeInterval); ++i) {
    readings.push_back(get_value());

    if (static_cast<int>(readings.size()) >= min_samples) {
      readings.pop_front();
      mean = std::accumulate(readings.begin(), readings.end(), 0.0) /
             readings.size();
      std_dev = CalculateStdDev(readings, mean);

      if (std_dev < kStabilityTolerance) {
        is_stable = true;
        *result = static_cast<uint32_t>(mean);
        std::cout << channel_name << " stabilized at " << *result << " after "
                  << i * kTimeInterval << " seconds" << std::endl;
        break;
      }

      if (i % 100 == 0) {  // Print status every 0.5 seconds
        std::cout << channel_name << " not yet stable. StdDev: " << std_dev
                  << " (target: " << kStabilityTolerance << ")" << std::endl;
      }
    }
  }

  return is_stable;
}
}  // namespace

int main(int /* argc */, char** /* argv */) {
  BaseBoardHandler handler("/dev/ttyACM0", 0xAA55, 200);
  handler.Start();

  const int min_samples = static_cast<int>(kMinDuration / kTimeInterval);
  uint32_t throttle_down = kMaxUint32, throttle_up = 0, throttle_idle = 0;
  uint32_t steer_left = kMaxUint32, steer_right = 0, steer_idle = 0;
  uint32_t aux_down = kMaxUint32, aux_up = 0, aux_idle = 0;

  std::cout << "Starting transmitter calibration..." << std::endl;
  std::cout << "Press Enter to begin..." << std::endl;
  std::cin.ignore();

  // Calibrate idle positions
  std::cout << "Calibrating idle values..." << std::endl;
  std::cout << "Ensure throttle, steering, and auxiliary controls are in idle "
               "positions."
            << std::endl;
  std::cout << "Press Enter to start..." << std::endl;
  std::cin.ignore();

  bool success = true;
  success &= CalibrateChannel(
      &handler, "Throttle Idle",
      [&handler]() { return handler.GetTransmitterThrottleRaw(); },
      &throttle_idle, min_samples);
  success &= CalibrateChannel(
      &handler, "Steering Idle",
      [&handler]() { return handler.GetTransmitterSteerRaw(); }, &steer_idle,
      min_samples);
  success &= CalibrateChannel(
      &handler, "Aux Idle",
      [&handler]() { return handler.GetTransmitterAuxRaw(); }, &aux_idle,
      min_samples);

  if (!success) {
    std::cerr << "Failed to calibrate idle positions" << std::endl;
    return 1;
  }

  // Calibrate up positions
  std::cout << "\nMove controls to their UP/RIGHT positions" << std::endl;
  std::cout << "Press Enter when ready..." << std::endl;
  std::cin.ignore();

  success &= CalibrateChannel(
      &handler, "Throttle Up",
      [&handler]() { return handler.GetTransmitterThrottleRaw(); },
      &throttle_up, min_samples);
  success &= CalibrateChannel(
      &handler, "Steering Right",
      [&handler]() { return handler.GetTransmitterSteerRaw(); }, &steer_right,
      min_samples);
  success &= CalibrateChannel(
      &handler, "Aux Up",
      [&handler]() { return handler.GetTransmitterAuxRaw(); }, &aux_up,
      min_samples);

  if (!success) {
    std::cerr << "Failed to calibrate up positions" << std::endl;
    return 1;
  }

  // Calibrate down positions
  std::cout << "\nMove controls to their DOWN/LEFT positions" << std::endl;
  std::cout << "Press Enter when ready..." << std::endl;
  std::cin.ignore();

  success &= CalibrateChannel(
      &handler, "Throttle Down",
      [&handler]() { return handler.GetTransmitterThrottleRaw(); },
      &throttle_down, min_samples);
  success &= CalibrateChannel(
      &handler, "Steering Left",
      [&handler]() { return handler.GetTransmitterSteerRaw(); }, &steer_left,
      min_samples);
  success &= CalibrateChannel(
      &handler, "Aux Down",
      [&handler]() { return handler.GetTransmitterAuxRaw(); }, &aux_down,
      min_samples);

  if (!success) {
    std::cerr << "Failed to calibrate down positions" << std::endl;
    return 1;
  }

  // Save calibration to YAML file
  YAML::Node config;

  config["Throttle"]["up"] = throttle_up;
  config["Throttle"]["idle"] = throttle_idle;
  config["Throttle"]["down"] = throttle_down;

  config["Steer"]["left"] = steer_left;
  config["Steer"]["idle"] = steer_idle;
  config["Steer"]["right"] = steer_right;

  config["Aux"]["up"] = aux_up;
  config["Aux"]["idle"] = aux_idle;
  config["Aux"]["down"] = aux_down;

  std::ofstream fout("transmitter_calibration.yaml");
  fout << config;
  fout.close();

  std::cout
      << "\nCalibration completed and saved to transmitter_calibration.yaml"
      << std::endl;
  handler.Stop();
  return 0;
}
