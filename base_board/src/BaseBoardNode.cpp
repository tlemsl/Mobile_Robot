#include "BaseBoardNode.h"

#include <stdexcept>

// Other libraries' headers
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
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

  // PID parameters
  pnh.param<double>("p_gain", p_gain_, 15.0);
  pnh.param<double>("i_gain", i_gain_, 1.0);
  pnh.param<double>("i_error_threshold", i_error_threshold_, 5.0);
  // ROS TOPICS
  pnh.param<std::string>("cmd_topic", cmd_topic_, "/base_board/cmd");
  pnh.param<std::string>("odom_topic", odom_topic_, "/odom");
  ROS_INFO("Odometry topic: %s", odom_topic_.c_str());
  pnh.param<double>("max_velocity", max_velocity_, 3.0);
  handler_ =
      new BaseBoardHandler(transmitter_config_path, port,
                           static_cast<uint16_t>(start_seq), publish_hz_);

  // Set up ROS communication
  cmd_sub_ = nh_->subscribe<ackermann_msgs::AckermannDriveStamped>(
      cmd_topic_, 1, &BaseBoardNode::CmdCallback, this);
  odom_sub_ = nh_->subscribe<nav_msgs::Odometry>(
      odom_topic_, 1, &BaseBoardNode::OdometryCallback, this);
  controller_cmd_pub_ = nh_->advertise<ackermann_msgs::AckermannDriveStamped>(
      "/base_board/controller_cmd", 1);
  controller_raw_cmd_pub_ =
      nh_->advertise<ackermann_msgs::AckermannDriveStamped>(
          "/base_board/controller_raw_cmd", 1);
  controller_mode_pub_ =
      nh_->advertise<std_msgs::String>("/base_board/controller_mode", 1);
  controller_pid_status_pub_ = nh_->advertise<std_msgs::Float64MultiArray>(
      "/base_board/controller_pid_status", 1);
  current_velocity_ = 0.0;
  target_velocity_ = 0.0;
  steering_cmd_ = 0.0;
  i_error_ = 0.0;
  info_thread_ = std::thread(&BaseBoardNode::PublishBaseInfo, this);
  pid_thread_ = std::thread(&BaseBoardNode::PIDLoop, this);
  handler_->Start();
}

BaseBoardNode::~BaseBoardNode() { handler_->Stop(); }

void BaseBoardNode::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  current_velocity_ = msg->twist.twist.linear.x;
}

void BaseBoardNode::CmdCallback(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
  target_velocity_ = msg->drive.speed;
  steering_cmd_ = msg->drive.steering_angle;
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
    controller_raw_cmd_pub_.publish(raw_response);
    response.drive.speed = velocity_pwm_to_actual_scale_ * pwm_velocity +
                           velocity_pwm_to_actual_offset_;
    response.drive.steering_angle =
        steer_pwm_to_actual_scale_ * pwm_steer + steer_pwm_to_actual_offset_;
    controller_cmd_pub_.publish(response);

    std_msgs::String mode_msg;
    switch (aux_state) {
      case AuxState::kDown:
        mode_msg.data = "TRANSMITTER VELOCITY CONTROL";
        break;
      case AuxState::kMiddle:
        mode_msg.data = "DIRECT CONTROL";
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
void BaseBoardNode::PIDLoop() {
  ROS_INFO("PIDLoop started");
  ROS_INFO("publish_hz_: %f", publish_hz_);
  ROS_INFO("P gain: %f", p_gain_);
  ROS_INFO("I gain: %f", i_gain_);
  ROS_INFO("I error threshold: %f", i_error_threshold_);
  ros::Rate rate(publish_hz_);
  double target_velocity = 0.0;
  double dt = 1.0 / publish_hz_;
  uint32_t steering_cmd = 0;
  std_msgs::Float64MultiArray pid_status;
  while (ros::ok()) {
    AuxState aux_state = handler_->GetTransmitterAux();
    if (aux_state == AuxState::kMiddle) {
      handler_->SetMotorCmd(handler_->GetTransmitterThrottleRaw());
      handler_->SetServoCmd(handler_->GetTransmitterSteerRaw());
      i_error_ = 0.0;
      rate.sleep();
      continue;
    }
    // PC control
    if (aux_state == AuxState::kUp) {
      target_velocity = target_velocity_;
      steering_cmd = steer_actual_to_pwm_scale_ * steering_cmd_;
    } else {
      // Transmitter control

      target_velocity = max_velocity_ * handler_->GetTransmitterThrottleRatio();
      steering_cmd = handler_->GetTransmitterSteer();
    }
    double feedforward_velocity =
        velocity_actual_to_pwm_scale_ * target_velocity +
        velocity_actual_to_pwm_offset_;
    double p_error = target_velocity - current_velocity_;
    i_error_ += p_error * dt;
    i_error_ = std::clamp(i_error_, -i_error_threshold_, i_error_threshold_);
    double pid_output =
        p_gain_ * p_error + i_gain_ * i_error_ + feedforward_velocity;
    handler_->SetMotorCmd(
        handler_->ToRawThrottle(static_cast<int>(pid_output)));
    handler_->SetServoCmd(handler_->ToRawSteer(steering_cmd));
    pid_status.data.clear();
    pid_status.data.push_back(target_velocity);
    pid_status.data.push_back(current_velocity_);
    pid_status.data.push_back(p_error);
    pid_status.data.push_back(p_gain_ * p_error);
    pid_status.data.push_back(i_error_);
    pid_status.data.push_back(i_gain_ * i_error_);
    pid_status.data.push_back(feedforward_velocity);
    pid_status.data.push_back(pid_output);
    pid_status.data.push_back(static_cast<double>(
        handler_->ToRawThrottle(static_cast<int>(pid_output))));
    controller_pid_status_pub_.publish(pid_status);
    std::cout << "Target velocity: " << target_velocity << std::endl;
    std::cout << "Current velocity: " << current_velocity_ << std::endl;
    std::cout << "Feedforward: " << feedforward_velocity << std::endl;
    std::cout << "P error: " << p_error << std::endl;
    std::cout << "P term: " << p_gain_ * p_error << std::endl;
    std::cout << "I error: " << i_error_ << std::endl;
    std::cout << "I term: " << i_gain_ * i_error_ << std::endl;
    std::cout << "PID output: " << pid_output << std::endl;
    std::cout << "Raw throttle: "
              << handler_->ToRawThrottle(static_cast<int>(pid_output))
              << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    rate.sleep();
  }
}
