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
    cmd_sub_ = nh_->subscribe("/base_board/cmd", 1, &BaseBoardNode::cmdCallback, this);
    controller_cmd_pub_ = nh_->advertise<std_msgs::Int32MultiArray>("/base_board/controller_cmd", 1);
    info_thread =  std::thread(&BaseBoardNode::publishBaseInfo, this);;

    phandler_->start();
}

BaseBoardNode::~BaseBoardNode() {
    phandler_->stop();
}

void BaseBoardNode::cmdCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 2) {
        ROS_WARN("Invalid message size");
        return;
    }

    uint32_t accel_cmd = msg->data[0];
    uint32_t steering_cmd = msg->data[1];
    phandler_->sendPacket(accel_cmd, steering_cmd);
}
void BaseBoardNode::publishBaseInfo() {
    std_msgs::Int32MultiArray response;
    ros::Rate r(publish_hz);
    while(ros::ok()) {
        response.data.clear();
        response.data.push_back(phandler_->getActualAccel());
        response.data.push_back(phandler_->getActualSteer());
        controller_cmd_pub_.publish(response);
        r.sleep();
    }

}