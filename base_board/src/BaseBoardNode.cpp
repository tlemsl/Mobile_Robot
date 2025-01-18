#include "BaseBoardNode.h"

#include <stdexcept>

BaseBoardNode::BaseBoardNode(ros::NodeHandle* nh) {
  nh_ = nh;
  ros::NodeHandle pnh("~");
  std::string port;
  int start_seq;
  std::string transimitter_config_path;
  pnh.param<std::string>("transimitter_config_path", transimitter_config_path,
                         "");
  pnh.param<std::string>("port", port, "/dev/ttyACM0");
  pnh.param<int>("start_seq", start_seq, 0xAA55);
  pnh.param<double>("publish_hz", publish_hz, 100.0);
  pnh.param<double>("A_actual_to_pwm_velocity", A_actual_to_pwm_velocity,
                    10.7987);
  pnh.param<double>("b_actual_to_pwm_velocity", b_actual_to_pwm_velocity,
                    38.8791);

  pnh.param<double>("A_pwm_to_velocity", A_pwm_to_velocity, 0.0925);
  pnh.param<double>("b_pwm_to_velocity", b_pwm_to_velocity, -3.5954);

  pnh.param<double>("A_actual_to_pwm_steer", A_actual_to_pwm_steer, -1229.7202);
  pnh.param<double>("b_actual_to_pwm_steer", b_actual_to_pwm_steer, -9.6785);

  pnh.param<double>("A_pwm_to_steer", A_pwm_to_steer, -0.0008106807517642265);
  pnh.param<double>("b_pwm_to_steer", b_pwm_to_steer, -0.007870494615355378);

  pnh.param<bool>("cmd_mode", cmd_mode_, false);

  phandler_ =
      new BaseBoardHandler(transimitter_config_path, port,
                           static_cast<uint16_t>(start_seq), publish_hz);
  cmd_sub_ = nh_->subscribe<ackermann_msgs::AckermannDriveStamped>(
      "/base_board/cmd", 1, &BaseBoardNode::cmdCallback, this);
  controller_cmd_pub_ = nh_->advertise<ackermann_msgs::AckermannDriveStamped>(
      "/base_board/controller_cmd", 1);
  controller_mode_pub_ =
      nh_->advertise<std_msgs::String>("/base_board/controller_mode", 1);

  info_thread = std::thread(&BaseBoardNode::publishBaseInfo, this);
  ROS_INFO("BaseBoardNode initialized with cmd_mode: %d", cmd_mode_);
  phandler_->start();
}

BaseBoardNode::~BaseBoardNode() { phandler_->stop(); }

void BaseBoardNode::cmdCallback(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
  double velocity_cmd = msg->drive.speed;
  double steering_cmd = msg->drive.steering_angle;

  ROS_INFO("velocity_cmd: %f, steering_cmd: %f", velocity_cmd, steering_cmd);
  phandler_->sendPacket(static_cast<int>(velocity_cmd),
                        static_cast<int>(steering_cmd));
}

void BaseBoardNode::publishBaseInfo() {
  ackermann_msgs::AckermannDriveStamped raw_response, response;
  ros::Rate r(publish_hz);
  while (ros::ok()) {
    response.header.stamp = ros::Time::now();
    raw_response.header = response.header;
    int pwm_velocity = phandler_->getBaseBoardMotorCmd();
    int pwm_steer = phandler_->getBaseBoardServoCmd();
    AuxState aux_state = phandler_->getTransimitterAux();
    raw_response.drive.speed = pwm_velocity;
    raw_response.drive.steering_angle = pwm_steer;
    controller_cmd_pub_.publish(response);
    std_msgs::String mode_msg;
    switch (aux_state) {
      case AuxState::DOWN:
        mode_msg.data = "SKIP-THROUGH";
        break;
      case AuxState::MIDDLE:
        mode_msg.data = "TRANSIMITTER VELOCITY CONTROL";
        break;
      case AuxState::UP:
        mode_msg.data = "PC VELOCITY CONTROL";
        break;
      default:
        mode_msg.data = "INVALID";
        break;
    }
    controller_mode_pub_.publish(mode_msg);
    r.sleep();
  }
}