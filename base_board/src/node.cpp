#include <ros/ros.h>

#include <stdexcept>

#include "BaseBoardNode.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_board_node");
  ros::NodeHandle nh;

  try {
    BaseBoardNode base_board(&nh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;  // Return non-zero to indicate error
  }

  return 0;
}
