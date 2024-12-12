#ifndef BASE_BOARD_NODE_H
#define BASE_BOARD_NODE_H

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

#include "BaseBoardHandler.h"
#include <thread>

class BaseBoardNode {
public:
    BaseBoardNode(ros::NodeHandle *nh);
    ~BaseBoardNode();

private:
    ros::NodeHandle *nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher controller_cmd_pub_, controller_raw_cmd_pub_;
    BaseBoardHandler* phandler_;
    std::thread info_thread;

    double publish_hz;

    bool cmd_mode_ = true; //false : raw command, true: physical cammand

    int accel_ref, steer_ref;

    double A_actual_to_cmd_accel, b_actual_to_cmd_accel;
    double A_cmd_to_actual_accel, b_cmd_to_actual_accel;
    double A_actual_to_cmd_steer, b_actual_to_cmd_steer;
    double A_cmd_to_actual_steer, b_cmd_to_actual_steer;

    
    void cmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
    void publishBaseInfo();
};

#endif // BASE_BOARD_NODE_H
