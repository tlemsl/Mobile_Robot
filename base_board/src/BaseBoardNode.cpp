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

    phandler_ = new BaseBoardHandler(port, static_cast<uint16_t>(start_seq), publish_hz);
    cmd_sub_ = nh_->subscribe<geometry_msgs::TwistStamped>("/base_board/cmd", 1, &BaseBoardNode::cmdCallback, this);
    controller_cmd_pub_ = nh_->advertise<geometry_msgs::TwistStamped>("/base_board/controller_cmd", 1);
    info_thread =  std::thread(&BaseBoardNode::publishBaseInfo, this);;

    phandler_->start();
}

BaseBoardNode::~BaseBoardNode() {
    phandler_->stop();
}

void BaseBoardNode::cmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    int accel_cmd = msg->twist.linear.x;
    int steering_cmd = msg->twist.angular.z;
    phandler_->sendPacket(accel_cmd, steering_cmd);
}
void BaseBoardNode::publishBaseInfo() {
    geometry_msgs::TwistStamped response;
    ros::Rate r(publish_hz);
    while(ros::ok()) {
        response.header.stamp = ros::Time::now();
        response.twist.linear.x = phandler_->getActualAccel();
        response.twist.angular.z = phandler_->getActualSteer();
        controller_cmd_pub_.publish(response);
        r.sleep();
    }

}