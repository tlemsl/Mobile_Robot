#ifndef BASE_BOARD_INCLUDE_BASE_BOARD_NODE_H_
#define BASE_BOARD_INCLUDE_BASE_BOARD_NODE_H_

// ROS core header first
#include <ros/ros.h>

// Other libraries' headers
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>

// C++ system headers
#include <thread>

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
  void PIDLoop();

  // Member variables (with trailing underscores)
  ros::NodeHandle* nh_;
  std::string cmd_topic_, odom_topic_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher controller_cmd_pub_;
  ros::Publisher controller_raw_cmd_pub_;
  ros::Publisher controller_mode_pub_;
  ros::Publisher controller_pid_status_pub_;
  BaseBoardHandler* handler_;
  std::thread info_thread_;
  std::thread pid_thread_;

  double publish_hz_;
  double current_velocity_;
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
  double i_error_threshold_;
  double target_velocity_;
  double steering_cmd_;
  double max_velocity_;
};

#endif  // BASE_BOARD_INCLUDE_BASE_BOARD_NODE_H_
