#include "BaseBoardNode.h"

#include <stdexcept>

BaseBoardNode::BaseBoardNode(ros::NodeHandle* nh){
    nh_ = nh;
    ros::NodeHandle pnh("~");
    std::string port;
    int start_seq;

    pnh.param<std::string>("port", port,  "/dev/ttyACM0");
    pnh.param<int>("start_seq", start_seq,  0xAA55);
    pnh.param<double>("publish_hz", publish_hz,  100.0);
    pnh.param<double>("A_actual_to_pwm_velocity", A_actual_to_pwm_velocity, 10.7987);
    pnh.param<double>("b_actual_to_pwm_velocity", b_actual_to_pwm_velocity, 38.8791);

    pnh.param<double>("A_pwm_to_velocity", A_pwm_to_velocity, 0.0925);
    pnh.param<double>("b_pwm_to_velocity", b_pwm_to_velocity, -3.5954);

    pnh.param<double>("A_actual_to_pwm_steer", A_actual_to_pwm_steer, -1229.7202);
    pnh.param<double>("b_actual_to_pwm_steer", b_actual_to_pwm_steer, -9.6785);

    pnh.param<double>("A_pwm_to_steer", A_pwm_to_steer, -0.0008106807517642265);
    pnh.param<double>("b_pwm_to_steer", b_pwm_to_steer, -0.007870494615355378);

    pnh.param<bool>("cmd_mode", cmd_mode_, false);


    phandler_ = new BaseBoardHandler(port, static_cast<uint16_t>(start_seq), publish_hz);
    cmd_sub_ = nh_->subscribe<ackermann_msgs::AckermannDriveStamped>("/base_board/cmd", 1, &BaseBoardNode::cmdCallback, this);
    controller_cmd_pub_ = nh_->advertise<ackermann_msgs::AckermannDriveStamped>("/base_board/controller_cmd", 1);
    controller_raw_cmd_pub_ = nh_->advertise<ackermann_msgs::AckermannDriveStamped>("/base_board/controller_raw_cmd", 1);

    info_thread =  std::thread(&BaseBoardNode::publishBaseInfo, this);
    ROS_INFO("BaseBoardNode initialized with cmd_mode: %d", cmd_mode_);
    phandler_->start();
}

BaseBoardNode::~BaseBoardNode() {
    phandler_->stop();
}

void BaseBoardNode::cmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    int velocity_cmd = msg->drive.speed;
    int steering_cmd = msg->drive.steering_angle;
    
    if(cmd_mode_) {
        velocity_cmd = A_actual_to_pwm_velocity*velocity_cmd + b_actual_to_pwm_velocity;
        steering_cmd = A_actual_to_pwm_steer*steering_cmd + b_actual_to_pwm_steer;
    }
    phandler_->sendPacket(velocity_cmd, steering_cmd);
}

void BaseBoardNode::publishBaseInfo() {
    ackermann_msgs::AckermannDriveStamped raw_response, response;
    ros::Rate r(publish_hz);
    while(ros::ok()) {
        response.header.stamp = ros::Time::now();
        raw_response.header = response.header;
        int pwm_velocity = phandler_->getActualAccel();
        int pwm_steer = phandler_->getActualSteer();
        if(pwm_velocity > -80 && pwm_velocity < 20) {
            response.drive.speed = 0;
        }
        else {
            response.drive.speed = A_pwm_to_velocity * pwm_velocity + b_pwm_to_velocity;
        }
        response.drive.steering_angle = A_pwm_to_steer * pwm_steer + b_pwm_to_steer;
        
        raw_response.drive.speed = pwm_velocity;
        raw_response.drive.steering_angle = pwm_steer;
        controller_cmd_pub_.publish(response);
        controller_raw_cmd_pub_.publish(raw_response);
        r.sleep();
    }

}