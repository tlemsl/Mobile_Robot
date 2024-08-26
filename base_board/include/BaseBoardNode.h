#ifndef BASE_BOARD_NODE_H
#define BASE_BOARD_NODE_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

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

    int accel_ref, steer_ref;
    
    void cmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void publishBaseInfo();
};

#endif // BASE_BOARD_NODE_H
