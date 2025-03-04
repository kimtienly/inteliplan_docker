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
#include "point_cloud_filtering/SegmentObject.h"
#include "object_segmenter.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "object_segmentation_node");
  ros::NodeHandle nh;

  point_cloud_filtering::ObjectSegmenter object_segmenter(&nh);
  object_segmenter.StartServices();

  ros::spin();

  return 0;
}
