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
    pnh.param<double>("A_actual_to_cmd_accel", A_actual_to_cmd_accel, 10.7987);
    pnh.param<double>("b_actual_to_cmd_accel", b_actual_to_cmd_accel, 38.8791);

    pnh.param<double>("A_cmd_to_actual_accel", A_actual_to_cmd_accel, 0.0925);
    pnh.param<double>("b_cmd_to_actual_accel", A_actual_to_cmd_accel, -3.5954);

    pnh.param<double>("A_actual_to_cmd_steer", A_actual_to_cmd_steer, -1229.7202);
    pnh.param<double>("b_actual_to_cmd_steer", b_actual_to_cmd_steer, -9.6785);

    pnh.param<double>("A_cmd_to_actual_steer", A_cmd_to_actual_steer, -0.0008106807517642265);
    pnh.param<double>("b_cmd_to_actual_steer", b_cmd_to_actual_steer, -0.007870494615355378);

    pnh.param<bool>("cmd_mode", cmd_mode_, 1);


    phandler_ = new BaseBoardHandler(port, static_cast<uint16_t>(start_seq), publish_hz);
    cmd_sub_ = nh_->subscribe<ackermann_msgs::AckermannDriveStamped>("/base_board/cmd", 1, &BaseBoardNode::cmdCallback, this);
    controller_cmd_pub_ = nh_->advertise<ackermann_msgs::AckermannDriveStamped>("/base_board/controller_cmd", 1);
    controller_raw_cmd_pub_ = nh_->advertise<ackermann_msgs::AckermannDriveStamped>("/base_board/controller_raw_cmd", 1);

    info_thread =  std::thread(&BaseBoardNode::publishBaseInfo, this);

    phandler_->start();
}

BaseBoardNode::~BaseBoardNode() {
    phandler_->stop();
}

void BaseBoardNode::cmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    int velocity_cmd = msg->drive.speed;
    int steering_cmd = msg->drive.steering_angle;
    
    if(cmd_mode_) {
        velocity_cmd = A_actual_to_cmd_accel*velocity_cmd + b_actual_to_cmd_accel;
        steering_cmd = A_actual_to_cmd_steer*steering_cmd + b_actual_to_cmd_steer;
    }
    phandler_->sendPacket(velocity_cmd, steering_cmd);
}

void BaseBoardNode::publishBaseInfo() {
    ackermann_msgs::AckermannDriveStamped raw_response, response;
    ros::Rate r(publish_hz);
    while(ros::ok()) {
        response.header.stamp = ros::Time::now();
        raw_response.header = raw_response.header;

        response.drive.speed = A_cmd_to_actual_accel * phandler_->getActualAccel() + b_cmd_to_actual_accel;
        response.drive.steering_angle = A_cmd_to_actual_steer * phandler_->getActualSteer() + b_cmd_to_actual_steer;
        
        raw_response.drive.speed = phandler_->getActualAccel();
        raw_response.drive.steering_angle = phandler_->getActualSteer();
        controller_cmd_pub_.publish(response);
        controller_raw_cmd_pub_.publish(raw_response);
        r.sleep();
    }

}