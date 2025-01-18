#include "BaseBoardNode.h"

#include <stdexcept>

BaseBoardNode::BaseBoardNode(ros::NodeHandle* nh) {
  nh_ = nh;
  ros::NodeHandle pnh("~");
  std::string port;
  int start_seq;
  std::string transmitter_config_path;

  // Load parameters
  pnh.param<std::string>("transmitter_config_path", transmitter_config_path,
                         "");
  pnh.param<std::string>("port", port, "/dev/ttyACM0");
  pnh.param<int>("start_seq", start_seq, 0xAA55);
  pnh.param<double>("publish_hz", publish_hz_, 100.0);

  // Load velocity calibration parameters
  pnh.param<double>("velocity_actual_to_pwm_scale",
                    velocity_actual_to_pwm_scale_, 10.7987);
  pnh.param<double>("velocity_actual_to_pwm_offset",
                    velocity_actual_to_pwm_offset_, 38.8791);
  pnh.param<double>("velocity_pwm_to_actual_scale",
                    velocity_pwm_to_actual_scale_, 0.0925);
  pnh.param<double>("velocity_pwm_to_actual_offset",
                    velocity_pwm_to_actual_offset_, -3.5954);

  // Load steering calibration parameters
  pnh.param<double>("steer_actual_to_pwm_scale", steer_actual_to_pwm_scale_,
                    -1229.7202);
  pnh.param<double>("steer_actual_to_pwm_offset", steer_actual_to_pwm_offset_,
                    -9.6785);
  pnh.param<double>("steer_pwm_to_actual_scale", steer_pwm_to_actual_scale_,
                    -0.0008106807517642265);
  pnh.param<double>("steer_pwm_to_actual_offset", steer_pwm_to_actual_offset_,
                    -0.007870494615355378);

  pnh.param<bool>("cmd_mode", cmd_mode_, false);

  handler_ =
      new BaseBoardHandler(transmitter_config_path, port,
                           static_cast<uint16_t>(start_seq), publish_hz_);

  // Set up ROS communication
  cmd_sub_ = nh_->subscribe<ackermann_msgs::AckermannDriveStamped>(
      "/base_board/cmd", 1, &BaseBoardNode::CmdCallback, this);
  controller_cmd_pub_ = nh_->advertise<ackermann_msgs::AckermannDriveStamped>(
      "/base_board/controller_cmd", 1);
  controller_mode_pub_ =
      nh_->advertise<std_msgs::String>("/base_board/controller_mode", 1);

  info_thread_ = std::thread(&BaseBoardNode::PublishBaseInfo, this);
  ROS_INFO("BaseBoardNode initialized with cmd_mode: %d", cmd_mode_);
  handler_->Start();
}

BaseBoardNode::~BaseBoardNode() { handler_->Stop(); }

void BaseBoardNode::CmdCallback(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
  double velocity_cmd = msg->drive.speed;
  double steering_cmd = msg->drive.steering_angle;

  ROS_INFO("velocity_cmd: %f, steering_cmd: %f", velocity_cmd, steering_cmd);
  handler_->SendPacket(static_cast<int>(velocity_cmd),
                       static_cast<int>(steering_cmd));
}

void BaseBoardNode::PublishBaseInfo() {
  ackermann_msgs::AckermannDriveStamped raw_response, response;
  ros::Rate rate(publish_hz_);

  while (ros::ok()) {
    response.header.stamp = ros::Time::now();
    raw_response.header = response.header;

    int pwm_velocity = handler_->GetBaseBoardMotorCmd();
    int pwm_steer = handler_->GetBaseBoardServoCmd();
    AuxState aux_state = handler_->GetTransmitterAux();

    raw_response.drive.speed = pwm_velocity;
    raw_response.drive.steering_angle = pwm_steer;
    controller_cmd_pub_.publish(response);

    std_msgs::String mode_msg;
    switch (aux_state) {
      case AuxState::kDown:
        mode_msg.data = "SKIP-THROUGH";
        break;
      case AuxState::kMiddle:
        mode_msg.data = "TRANSMITTER VELOCITY CONTROL";
        break;
      case AuxState::kUp:
        mode_msg.data = "PC VELOCITY CONTROL";
        break;
      default:
        mode_msg.data = "INVALID";
        break;
    }
    controller_mode_pub_.publish(mode_msg);
    rate.sleep();
  }
}