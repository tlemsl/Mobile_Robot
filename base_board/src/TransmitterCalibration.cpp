#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <numeric>

#include "BaseBoardHandler.h"

int main(int argc, char** argv) {
  BaseBoardHandler handler("/dev/ttyACM0", 0xAA55, 200);
  handler.start();
  double time_interval = 0.005;  // 200Hz
  double min_duration = 2;
  double max_duration = 10;
  int min_samples = min_duration / time_interval;
  int max_samples = max_duration / time_interval;
  double stability_tolerance = 1;
  uint32_t throttle_down = UINT32_MAX, throttle_up = 0, throttle_idle = 0;
  uint32_t steer_left = UINT32_MAX, steer_right = 0, steer_idle = 0;
  uint32_t aux_down = UINT32_MAX, aux_up = 0, aux_idle = 0;

  std::cout << "Starting transmitter calibration..." << std::endl;
  std::cout << "Press Enter to begin..." << std::endl;
  std::cin.ignore();

  std::cout << "Calibrating idle values..." << std::endl;
  std::cout << "Ensure throttle, steering, and auxiliary controls are in idle "
               "positions."
            << std::endl;
  std::cout << "Calibration will take between 2 to 10 seconds to stabilize."
            << std::endl;
  std::cout << "Press Enter to start..." << std::endl;
  std::cin.ignore();
  std::deque<uint32_t> throttle_readings, steer_readings, aux_readings;
  bool is_throttle_stable = false, is_steer_stable = false,
       is_aux_stable = false;

  double throttle_avg = 0, steer_avg = 0, aux_avg = 0;
  double throttle_deviation = 0, steer_deviation = 0, aux_deviation = 0;

  for (int i = 0; i < max_samples; ++i) {
    aux_readings.push_back(handler.getTransimitterAuxRaw());

    // Throttle calibration
    if (!is_throttle_stable) {
      throttle_readings.push_back(handler.getTransimitterThrottleRaw());
      if (throttle_readings.size() >= min_samples) {
        throttle_readings.pop_front();
        throttle_avg = std::accumulate(throttle_readings.begin(),
                                       throttle_readings.end(), 0) /
                       throttle_readings.size();
        throttle_deviation =
            std::sqrt(std::accumulate(
                          throttle_readings.begin(), throttle_readings.end(), 0,
                          [throttle_avg](double sum, uint32_t value) {
                            return sum + std::pow(value - throttle_avg, 2);
                          }) /
                      throttle_readings.size());
        if (throttle_deviation < stability_tolerance) {
          is_throttle_stable = true;
        }
      }
    }
    // Steering calibration
    if (!is_steer_stable) {
      steer_readings.push_back(handler.getTransimitterSteerRaw());
      if (steer_readings.size() >= min_samples) {
        steer_readings.pop_front();
        steer_avg =
            std::accumulate(steer_readings.begin(), steer_readings.end(), 0) /
            steer_readings.size();
        steer_deviation = std::sqrt(
            std::accumulate(steer_readings.begin(), steer_readings.end(), 0,
                            [steer_avg](double sum, uint32_t value) {
                              return sum + std::pow(value - steer_avg, 2);
                            }) /
            steer_readings.size());
        if (steer_deviation < stability_tolerance) {
          is_steer_stable = true;
        }
      }
    }
    // Auxiliary calibration
    if (!is_aux_stable) {
      aux_readings.push_back(handler.getTransimitterAuxRaw());
      if (aux_readings.size() >= min_samples) {
        aux_readings.pop_front();
        aux_avg = std::accumulate(aux_readings.begin(), aux_readings.end(), 0) /
                  aux_readings.size();
        aux_deviation = std::sqrt(
            std::accumulate(aux_readings.begin(), aux_readings.end(), 0,
                            [aux_avg](double sum, uint32_t value) {
                              return sum + std::pow(value - aux_avg, 2);
                            }) /
            aux_readings.size());
        if (aux_deviation < stability_tolerance) {
          is_aux_stable = true;
        }
      }
    }
    if (i % 100 == 0 && i > min_samples) {
      std::cout << "Elapsed time: " << i * time_interval << " seconds / "
                << max_duration << " seconds" << std::endl;
      std::cout << "Throttle: " << throttle_avg << ", " << throttle_deviation;
      if (is_throttle_stable) {
        std::cout << " (stable)";
      }
      std::cout << std::endl;
      std::cout << "Steer: " << steer_avg << ", " << steer_deviation;
      if (is_steer_stable) {
        std::cout << " (stable)";
      }
      std::cout << std::endl;
      std::cout << "Aux: " << aux_avg << ", " << aux_deviation;
      if (is_aux_stable) {
        std::cout << " (stable)";
      }
      std::cout << std::endl;
    }
    if (is_throttle_stable && is_steer_stable && is_aux_stable &&
        i > min_samples) {
      std::cout << "Idle calibration complete! Duration: " << i * time_interval
                << " seconds" << std::endl;
      throttle_idle = throttle_avg;
      steer_idle = steer_avg;
      aux_idle = aux_avg;
      std::cout << "Idle throttle: " << throttle_idle
                << " Idle steer: " << steer_idle << " Idle aux: " << aux_idle
                << std::endl;
      break;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(time_interval * 1000)));
  }
  if (!is_throttle_stable || !is_steer_stable || !is_aux_stable) {
    std::cout << "Idle calibration failed!" << std::endl;
    std::cout << "Throttle: " << throttle_avg << ", " << throttle_deviation;
    if (!is_throttle_stable) {
      std::cout << " (unstable)";
    }
    std::cout << std::endl;
    std::cout << "Steer: " << steer_avg << ", " << steer_deviation;
    if (!is_steer_stable) {
      std::cout << " (unstable)";
    }
    std::cout << std::endl;
    std::cout << "Aux: " << aux_avg << ", " << aux_deviation;
    if (!is_aux_stable) {
      std::cout << " (unstable)";
    }
    std::cout << std::endl;
    std::cout << "Please try again!" << std::endl;
    throw std::runtime_error("Calibration failed!");
  }
  // Throttle up calibration
  std::cout << "Calibrating throttle up..." << std::endl;
  std::cout << "Set throttle to maximum position..." << std::endl;
  std::cout << "Press Enter to start..." << std::endl;
  std::cin.ignore();
  throttle_readings.clear();
  throttle_avg = 0;
  throttle_deviation = 0;
  is_throttle_stable = false;
  throttle_readings.clear();
  for (int i = 0; i < max_samples; ++i) {
    throttle_readings.push_back(handler.getTransimitterThrottleRaw());
    if (i > min_samples) {
      throttle_readings.pop_front();
      throttle_avg = std::accumulate(throttle_readings.begin(),
                                     throttle_readings.end(), 0.0) /
                     throttle_readings.size();
      throttle_deviation =
          std::sqrt(std::accumulate(throttle_readings.begin(),
                                    throttle_readings.end(), 0.0,
                                    [throttle_avg](double acc, double val) {
                                      return acc + (val - throttle_avg) *
                                                       (val - throttle_avg);
                                    }) /
                    (throttle_readings.size() - 1));
      if (throttle_deviation < stability_tolerance &&
          std::abs(throttle_avg - throttle_idle) > 100) {
        is_throttle_stable = throttle_deviation < stability_tolerance;
      }
      if (i % 100 == 0 && i > min_samples) {
        std::cout << "Throttle: " << throttle_avg << ", " << throttle_deviation;
        if (is_throttle_stable) {
          std::cout << " (stable)" << std::endl;
          throttle_up = throttle_avg;
          std::cout << "Throttle up calibration complete! Duration: "
                    << i * time_interval << " seconds" << std::endl;
          break;
        } else {
          std::cout << " (unstable)" << std::endl;
        }
      }
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(time_interval * 1000)));
  }
  if (!is_throttle_stable) {
    std::cout << "Throttle up calibration failed!" << std::endl;
    std::cout << "Please try again!" << std::endl;
    throw std::runtime_error("Calibration failed!");
  }
  // Throttle down calibration
  std::cout << "Calibrating throttle down..." << std::endl;
  std::cout << "Set throttle to minimum position..." << std::endl;
  std::cout << "Press Enter to start..." << std::endl;
  std::cin.ignore();
  throttle_readings.clear();
  throttle_avg = 0;
  throttle_deviation = 0;
  is_throttle_stable = false;
  for (int i = 0; i < max_samples; ++i) {
    throttle_readings.push_back(handler.getTransimitterThrottleRaw());
    if (i > min_samples) {
      throttle_readings.pop_front();
      throttle_avg = std::accumulate(throttle_readings.begin(),
                                     throttle_readings.end(), 0.0) /
                     throttle_readings.size();
      throttle_deviation =
          std::sqrt(std::accumulate(throttle_readings.begin(),
                                    throttle_readings.end(), 0.0,
                                    [throttle_avg](double acc, double val) {
                                      return acc + (val - throttle_avg) *
                                                       (val - throttle_avg);
                                    }) /
                    (throttle_readings.size() - 1));
      if (throttle_deviation < stability_tolerance &&
          std::abs(throttle_avg - throttle_idle) > 100) {
        is_throttle_stable = throttle_deviation < stability_tolerance;
      }
      if (i % 100 == 0 && i > min_samples) {
        std::cout << "Throttle: " << throttle_avg << ", " << throttle_deviation;
        if (is_throttle_stable) {
          std::cout << " (stable)" << std::endl;
          throttle_down = throttle_avg;
          std::cout << "Throttle down calibration complete! Duration: "
                    << i * time_interval << " seconds" << std::endl;
          break;
        } else {
          std::cout << " (unstable)" << std::endl;
        }
      }
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(time_interval * 1000)));
  }
  if (!is_throttle_stable) {
    std::cout << "Throttle down calibration failed!" << std::endl;
    std::cout << "Please try again!" << std::endl;
    throw std::runtime_error("Calibration failed!");
  }

  // Steering right calibration
  std::cout << "Calibrating steering right..." << std::endl;
  std::cout << "Set steering to maximum right position..." << std::endl;
  std::cout << "Press Enter to start..." << std::endl;
  std::cin.ignore();
  steer_readings.clear();
  steer_avg = 0;
  steer_deviation = 0;
  is_steer_stable = false;
  for (int i = 0; i < max_samples; ++i) {
    steer_readings.push_back(handler.getTransimitterSteerRaw());
    if (i > min_samples) {
      steer_readings.pop_front();
      steer_avg =
          std::accumulate(steer_readings.begin(), steer_readings.end(), 0.0) /
          steer_readings.size();
      steer_deviation = std::sqrt(
          std::accumulate(steer_readings.begin(), steer_readings.end(), 0.0,
                          [steer_avg](double acc, double val) {
                            return acc + (val - steer_avg) * (val - steer_avg);
                          }) /
          (steer_readings.size() - 1));
      if (steer_deviation < stability_tolerance &&
          std::abs(steer_avg - steer_idle) > 100) {
        is_steer_stable = steer_deviation < stability_tolerance;
      }
      if (i % 100 == 0 && i > min_samples) {
        std::cout << "Steer: " << steer_avg << ", " << steer_deviation;
        if (is_steer_stable) {
          std::cout << " (stable)" << std::endl;
          steer_right = steer_avg;
          std::cout << "Steering right calibration complete! Duration: "
                    << i * time_interval << " seconds" << std::endl;
          break;
        } else {
          std::cout << " (unstable)" << std::endl;
        }
      }
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(time_interval * 1000)));
  }
  if (!is_steer_stable) {
    std::cout << "Steering right calibration failed!" << std::endl;
    std::cout << "Please try again!" << std::endl;
    throw std::runtime_error("Calibration failed!");
  }

  // Steering left calibration
  std::cout << "Calibrating steering left..." << std::endl;
  std::cout << "Set steering to maximum left position..." << std::endl;
  std::cout << "Press Enter to start..." << std::endl;
  std::cin.ignore();
  steer_readings.clear();
  steer_avg = 0;
  steer_deviation = 0;
  is_steer_stable = false;
  for (int i = 0; i < max_samples; ++i) {
    steer_readings.push_back(handler.getTransimitterSteerRaw());
    if (i > min_samples) {
      steer_readings.pop_front();
      steer_avg =
          std::accumulate(steer_readings.begin(), steer_readings.end(), 0.0) /
          steer_readings.size();
      steer_deviation = std::sqrt(
          std::accumulate(steer_readings.begin(), steer_readings.end(), 0.0,
                          [steer_avg](double acc, double val) {
                            return acc + (val - steer_avg) * (val - steer_avg);
                          }) /
          (steer_readings.size() - 1));
      if (steer_deviation < stability_tolerance &&
          std::abs(steer_avg - steer_idle) > 100) {
        is_steer_stable = steer_deviation < stability_tolerance;
      }
      if (i % 100 == 0 && i > min_samples) {
        std::cout << "Steer: " << steer_avg << ", " << steer_deviation;
        if (is_steer_stable) {
          std::cout << " (stable)" << std::endl;
          steer_left = steer_avg;
          std::cout << "Steering left calibration complete! Duration: "
                    << i * time_interval << " seconds" << std::endl;
          break;
        } else {
          std::cout << " (unstable)" << std::endl;
        }
      }
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(time_interval * 1000)));
  }
  if (!is_steer_stable) {
    std::cout << "Steering left calibration failed!" << std::endl;
    std::cout << "Please try again!" << std::endl;
    throw std::runtime_error("Calibration failed!");
  }

  // Auxiliary up calibration
  std::cout << "Calibrating auxiliary up..." << std::endl;
  std::cout << "Set auxiliary to maximum position..." << std::endl;
  std::cout << "Press Enter to start..." << std::endl;
  std::cin.ignore();
  aux_readings.clear();
  aux_avg = 0;
  aux_deviation = 0;
  is_aux_stable = false;
  for (int i = 0; i < max_samples; ++i) {
    aux_readings.push_back(handler.getTransimitterAuxRaw());
    if (i > min_samples) {
      aux_readings.pop_front();
      aux_avg = std::accumulate(aux_readings.begin(), aux_readings.end(), 0.0) /
                aux_readings.size();
      aux_deviation = std::sqrt(
          std::accumulate(aux_readings.begin(), aux_readings.end(), 0.0,
                          [aux_avg](double acc, double val) {
                            return acc + (val - aux_avg) * (val - aux_avg);
                          }) /
          (aux_readings.size() - 1));
      if (aux_deviation < stability_tolerance &&
          std::abs(aux_avg - aux_idle) > 100) {
        is_aux_stable = aux_deviation < stability_tolerance;
      }
      if (i % 100 == 0 && i > min_samples) {
        std::cout << "Aux: " << aux_avg << ", " << aux_deviation;
        if (is_aux_stable) {
          std::cout << " (stable)" << std::endl;
          aux_up = aux_avg;
          std::cout << "Auxiliary up calibration complete! Duration: "
                    << i * time_interval << " seconds" << std::endl;
          break;
        } else {
          std::cout << " (unstable)" << std::endl;
        }
      }
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(time_interval * 1000)));
  }
  if (!is_aux_stable) {
    std::cout << "Auxiliary up calibration failed!" << std::endl;
    std::cout << "Please try again!" << std::endl;
    throw std::runtime_error("Calibration failed!");
  }
  // Auxiliary down calibration
  std::cout << "Calibrating auxiliary down..." << std::endl;
  std::cout << "Set auxiliary to minimum position..." << std::endl;
  std::cout << "Press Enter to start..." << std::endl;
  std::cin.ignore();
  aux_readings.clear();
  aux_avg = 0;
  aux_deviation = 0;
  is_aux_stable = false;
  for (int i = 0; i < max_samples; ++i) {
    aux_readings.push_back(handler.getTransimitterAuxRaw());
    if (i > min_samples) {
      aux_readings.pop_front();
      aux_avg = std::accumulate(aux_readings.begin(), aux_readings.end(), 0.0) /
                aux_readings.size();
      aux_deviation = std::sqrt(
          std::accumulate(aux_readings.begin(), aux_readings.end(), 0.0,
                          [aux_avg](double acc, double val) {
                            return acc + (val - aux_avg) * (val - aux_avg);
                          }) /
          (aux_readings.size() - 1));
      if (aux_deviation < stability_tolerance &&
          std::abs(aux_avg - aux_idle) > 100) {
        is_aux_stable = aux_deviation < stability_tolerance;
      }
      if (i % 100 == 0 && i > min_samples) {
        std::cout << "Aux: " << aux_avg << ", " << aux_deviation;
        if (is_aux_stable) {
          std::cout << " (stable)" << std::endl;
          aux_down = aux_avg;
          std::cout << "Auxiliary down calibration complete! Duration: "
                    << i * time_interval << " seconds" << std::endl;
          break;
        } else {
          std::cout << " (unstable)" << std::endl;
        }
      }
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(time_interval * 1000)));
  }
  if (!is_aux_stable) {
    std::cout << "Auxiliary down calibration failed!" << std::endl;
    std::cout << "Please try again!" << std::endl;
    throw std::runtime_error("Calibration failed!");
  }

  // Save to YAML file
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "Throttle" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "down" << YAML::Value << throttle_down;
  out << YAML::Key << "up" << YAML::Value << throttle_up;
  out << YAML::Key << "idle" << YAML::Value << throttle_idle;
  out << YAML::EndMap;

  out << YAML::Key << "Steer" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "left" << YAML::Value << steer_left;
  out << YAML::Key << "right" << YAML::Value << steer_right;
  out << YAML::Key << "idle" << YAML::Value << steer_idle;
  out << YAML::EndMap;

  out << YAML::Key << "Aux" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "down" << YAML::Value << aux_down;
  out << YAML::Key << "up" << YAML::Value << aux_up;
  out << YAML::Key << "idle" << YAML::Value << aux_idle;
  out << YAML::EndMap;
  out << YAML::EndMap;

  std::ofstream fout("transmitter_calibration.yaml");
  fout << out.c_str();
  fout.close();

  handler.stop();
  return 0;
}
