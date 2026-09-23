#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "perception_storage/perception_storage.hpp"
#include "perception_storage/pose_tracker/pose_tracker.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "perception_storage_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;
  std::shared_ptr<PerceptionStorage> perception_ptr;
  perception_ptr=std::make_shared<PerceptionStorage>(pnh,nh);
  tf::TransformListener tf(ros::Duration(30.0));
  std::shared_ptr<pose_tracker::pose_tracker> tracker_ptr;
  tracker_ptr=std::make_shared<pose_tracker::pose_tracker>(pnh,&tf);
  ros::spin();
  ros::waitForShutdown();

  return 0;
}
