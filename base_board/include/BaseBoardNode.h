#ifndef BASE_BOARD_NODE_H
#define BASE_BOARD_NODE_H

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include "BaseBoardHandler.h"
#include <thread>

class BaseBoardNode {
public:
    BaseBoardNode(ros::NodeHandle *nh);
    ~BaseBoardNode();

private:
    ros::NodeHandle *nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher controller_cmd_pub_;
    BaseBoardHandler* phandler_;
    std::thread info_thread;

    double publish_hz;

    void cmdCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void publishBaseInfo();
};

#endif // BASE_BOARD_NODE_H
