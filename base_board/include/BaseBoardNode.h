#ifndef BASE_BOARD_INCLUDE_BASE_BOARD_NODE_H_
#define BASE_BOARD_INCLUDE_BASE_BOARD_NODE_H_

#include <ros/ros.h>  // ROS core header first

// C++ system headers
#include <thread>

// Other libraries' headers
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

// Project headers
#include "BaseBoardHandler.h"

class BaseBoardNode {
 public:
  BaseBoardNode(ros::NodeHandle* nh);
  ~BaseBoardNode();

 private:
  // Methods
  void CmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
  void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void PublishBaseInfo();

  // Member variables (with trailing underscores)
  ros::NodeHandle* nh_;
  ros::Subscriber cmd_sub_;
  ros::Publisher controller_cmd_pub_;
  ros::Publisher controller_mode_pub_;
  BaseBoardHandler* handler_;
  std::thread info_thread_;

  double publish_hz_;
  bool cmd_mode_ = true;  // false: raw command, true: physical command
  int accel_ref_;
  int steer_ref_;

  // Calibration parameters
  double velocity_actual_to_pwm_scale_;
  double velocity_actual_to_pwm_offset_;
  double velocity_pwm_to_actual_scale_;
  double velocity_pwm_to_actual_offset_;
  double steer_actual_to_pwm_scale_;
  double steer_actual_to_pwm_offset_;
  double steer_pwm_to_actual_scale_;
  double steer_pwm_to_actual_offset_;

  // PID control parameters
  double p_gain_;
  double i_gain_;
  double d_gain_;
  double i_error_;
};

#endif  // BASE_BOARD_INCLUDE_BASE_BOARD_NODE_H_
