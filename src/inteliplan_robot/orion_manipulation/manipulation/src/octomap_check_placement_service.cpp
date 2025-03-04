//
// Reconstructs STL meshes from Octomap occupancy map input.
//

#include <eigen_conversions/eigen_msg.h>
#include <octomap/octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <cstdio>
#include <ctime>
#include <iostream>
using namespace std;

#include "manipulation/BoundingBox.h"
#include "manipulation/CheckPlacement.h"

#define corner_test_scale 1.5

class OctomapPlacementChecker {
 private:
  ros::ServiceServer check_placement_service;
  ros::ServiceClient get_octomap_client;

 public:
  explicit OctomapPlacementChecker(ros::NodeHandle* nh) {
    check_placement_service = nh->advertiseService(
        "CheckPlacement", &OctomapPlacementChecker::service_callback, this);
    get_octomap_client = nh->serviceClient<octomap_msgs::GetOctomap>("octomap_binary");
  }

  static inline octomap::point3d pointMsgToOctomap(const geometry_msgs::Point& ptMsg) {
    return octomap::point3d(ptMsg.x, ptMsg.y, ptMsg.z);
  }

  static inline tuple<octomap::point3d, octomap::point3d> msgToBBxCorners(const geometry_msgs::Point& center, const geometry_msgs::Point& dims) {
    return {octomap::point3d(center.x + dims.x / 2, center.y + dims.y / 2, center.z + dims.z / 2),
            octomap::point3d(center.x - dims.x / 2, center.y - dims.y / 2, center.z - dims.z / 2)};
  }

  bool checkBbxClearance(octomap::OcTree* octree,
                        octomap::point3d bb_max,
                        octomap::point3d bb_min){
    // Check if given space is occupied
    int num_boxes = 0;
    for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(bb_min, bb_max),
                                            end = octree->end_leafs_bbx();
         it != end; ++it) {
         if (it->getOccupancy() > 0.5){
            num_boxes ++;
        }
    }
    return (num_boxes == 0);
  }

  bool checkSupported(octomap::OcTree* octree,
                    geometry_msgs::Point& _origin,
                    geometry_msgs::Point& dims,
                    double maxRange,
                    bool checkCorners = false){
    bool flag = true;

    if (!checkCorners){
        octomap::point3d origin = pointMsgToOctomap(_origin);
        octomap::point3d end = octomap::point3d(0, 0, 0);
        flag = octree->castRay(origin, octomap::point3d(0, 0, -1), end, true, maxRange);
    }
    else{
        octomap::point3d origin;
        int coef[2] = {1, -1};

        for (int ii: coef){
            for (int jj: coef){
                origin = octomap::point3d(_origin.x + ii * corner_test_scale * dims.x / 2,
                                          _origin.y + jj * corner_test_scale * dims.y / 2,
                                          _origin.z);
                octomap::point3d end = octomap::point3d(0, 0, 0);
                bool tempFlag = octree->castRay(origin, octomap::point3d(0, 0, -1), end, true, maxRange);

                std::cout << "\tii=" << ii << ", jj=" << jj << ", is_ray_supported=" 
                    << (tempFlag==true ? "supported" : "unsupported") << std::endl;

                if (!tempFlag){
                    return false;
                }
            }
        }
    }

    std::cout << "Location is supported." << std::endl;

    return flag;
  }

  bool service_callback(manipulation::CheckPlacement::Request& req,
                        manipulation::CheckPlacement::Response& res) {
    ROS_INFO("Message request received.");

    octomap_msgs::GetOctomap srv_data;

    if (!get_octomap_client.call(srv_data)) {
      ROS_ERROR("Failed to call service octomap_binary");
      return false;
    }

    octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(srv_data.response.map);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

    octomap::point3d bb_max, bb_min;
    tie(bb_max, bb_min) = msgToBBxCorners(req.center, req.dims);

    bool isAvailable = checkBbxClearance(octree, bb_max, bb_min);
    bool isSupported = checkSupported(octree, req.center, req.dims, req.maxHeight, true);

    res.isAvailable = isAvailable;
    res.isSupported = isSupported;

    return true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "octomap_check_placement");
  ros::NodeHandle nh;

  OctomapPlacementChecker srv = OctomapPlacementChecker(&nh);

  ROS_INFO("%s: service ready", ros::this_node::getName().c_str());

  ros::spin();

  return 0;
}
