//
// Created by etienne on 06/02/16.
//
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "proc_mapping");

  //ros::NodeHandle nh("~");
  //TritechMicron tritech_micron(nh);

  ros::spin();
}