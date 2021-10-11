/*
slam_mapping
 */

/* Author: HoGinhang  */

#include <ros/ros.h>

#include "Fortest_DemoSlamMapping.h"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_mapping");

  SlamMapping gotest;
  gotest.startLiveSlam();
  ros::spin();

  return(0);
}

