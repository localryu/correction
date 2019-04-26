#include <iostream>
#include <ros/ros.h>
#include <string>

#include <lcm/lcm-cpp.hpp>
#include "lcm_to_ros/gga_t.h"
#include "lcm_to_ros/hyundai_mission.h"
#include "eurecar/gga_t.hpp"
#include "eurecar/hyundai_mission.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
using namespace message_filters;


void callback(const lcm_to_ros::gga_t& path)
{
  ROS_INFO("!!!\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compute_correction");

  ros::NodeHandle nh;
  ros::Subscriber path_sub;
  path_sub = nh.subscribe("/lcm_to_ros/eurecar_gga_l2r", 1, &callback);

  ros::spin();

  return 0;
}
