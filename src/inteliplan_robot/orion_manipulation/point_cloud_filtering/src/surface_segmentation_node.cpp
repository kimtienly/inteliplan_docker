//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <cstdio>
#include <ctime>
#include <iostream>

#include "geometry_msgs/PointStamped.h"
#include "surface_segmenter.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "surface_segmentation_node");
  ros::NodeHandle nh;
  ROS_INFO("Starting surface_segmentation_node");
  point_cloud_filtering::SurfaceSegmenter surface_segmenter(&nh);
  ROS_INFO("Starting surface_segmentation_node 2");
  surface_segmenter.StartServices();

  ros::spin();

  return 0;
}
