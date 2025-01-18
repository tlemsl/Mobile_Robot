#ifndef BASE_BOARD_NODE_H
#define BASE_BOARD_NODE_H

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

#include <thread>

#include "BaseBoardHandler.h"

class BaseBoardNode {
 public:
  BaseBoardNode(ros::NodeHandle* nh);
  ~BaseBoardNode();

 private:
  ros::NodeHandle* nh_;
  ros::Subscriber cmd_sub_;
  ros::Publisher controller_cmd_pub_, controller_mode_pub_;
  BaseBoardHandler* phandler_;
  std::thread info_thread;

  double publish_hz;

  bool cmd_mode_ = true;  // false : raw command, true: physical cammand

  int accel_ref, steer_ref;

  double A_actual_to_pwm_velocity, b_actual_to_pwm_velocity;
  double A_pwm_to_velocity, b_pwm_to_velocity;
  double A_actual_to_pwm_steer, b_actual_to_pwm_steer;
  double A_pwm_to_steer, b_pwm_to_steer;

  void cmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
  void publishBaseInfo();
};

#endif  // BASE_BOARD_NODE_H
