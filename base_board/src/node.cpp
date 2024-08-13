#include <ros/ros.h>
#include "BaseBoardNode.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "base_board_node");
    ros::NodeHandle nh;
    try {
        BaseBoardNode base_board(&nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }
    return 0;
}
