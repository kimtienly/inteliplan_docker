//
// Reconstructs STL meshes from Octomap occupancy map input.
//

#include <eigen_conversions/eigen_msg.h>
#include <octomap/octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <cstdio>
#include <ctime>
#include <iostream>

#include "../include/stl-creator/mesh.h"
#include "../include/stl-creator/triangle.h"
#include "../include/stl-creator/vec3.h"
#include "manipulation/BoundingBox.h"
#include "manipulation/GetReconstruction.h"

class OctomapToReconstruction {
 private:
  ros::ServiceServer reconstruction_service;
  ros::ServiceClient get_octomap_client;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools;

 public:
  explicit OctomapToReconstruction(ros::NodeHandle* nh) {
    visual_tools.reset(
        new rviz_visual_tools::RvizVisualTools("map", "/collision_bounding_boxes"));
    reconstruction_service = nh->advertiseService(
        "GetReconstruction", &OctomapToReconstruction::service_callback, this);
    get_octomap_client = nh->serviceClient<octomap_msgs::GetOctomap>("octomap_binary");

    visual_tools->trigger();
  }

  static inline octomap::point3d pointMsgToOctomap(const geometry_msgs::Point& ptMsg) {
    return octomap::point3d(ptMsg.x, ptMsg.y, ptMsg.z);
  }

  void publish_bounding_box_visualisations(
      manipulation::BoundingBox& external_bb,
      std::vector<manipulation::BoundingBox>& crop_bbs) {
    Eigen::Isometry3d identity_pose = Eigen::Isometry3d::Identity();
    Eigen::Vector3d vec_bb_min, vec_bb_max;
    tf::pointMsgToEigen(external_bb.min, vec_bb_min);
    tf::pointMsgToEigen(external_bb.max, vec_bb_max);

    visual_tools->deleteAllMarkers();
    visual_tools->publishWireframeCuboid(identity_pose, vec_bb_min, vec_bb_max,
                                         rviz_visual_tools::GREEN);

    for (auto crop_bb : crop_bbs) {
      tf::pointMsgToEigen(crop_bb.min, vec_bb_min);
      tf::pointMsgToEigen(crop_bb.max, vec_bb_max);
      visual_tools->publishWireframeCuboid(identity_pose, vec_bb_min, vec_bb_max,
                                           rviz_visual_tools::RED);
    }

    visual_tools->trigger();
  }

  bool service_callback(manipulation::GetReconstruction::Request& req,
                        manipulation::GetReconstruction::Response& res) {
    ROS_INFO("Message request received.");
    res.flag = false;
    octomap_msgs::GetOctomap srv_data;

    if (!get_octomap_client.call(srv_data)) {
      ROS_ERROR("Failed to call service octomap_binary");
      return false;
    }

    std::vector<manipulation::BoundingBox> crop_bbs = req.crop_bbs;
    publish_bounding_box_visualisations(req.external_bb, req.crop_bbs);

    octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(srv_data.response.map);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    double thresMin = octree->getClampingThresMin();

    // iterating through areas to prune
    for (auto crop_bb : crop_bbs) {
      // extracting the boundary boxes for the area of interest
      octomap::point3d crop_min = pointMsgToOctomap(crop_bb.min);
      octomap::point3d crop_max = pointMsgToOctomap(crop_bb.max);
      // iterate through all boxes inside te pruning region and prune them
      for (octomap::OcTree::leaf_bbx_iterator
               it = octree->begin_leafs_bbx(crop_min, crop_max),
               end = octree->end_leafs_bbx();
           it != end; ++it) {
        it->setLogOdds(octomap::logodds(thresMin));
        // m_octree->updateNode(it.getKey(), -6.0f);
      }
    }
    octree->updateInnerOccupancy();

    // extracting the boundary boxes for the area of interest
    octomap::point3d ex_min = pointMsgToOctomap(req.external_bb.min);
    octomap::point3d ex_max = pointMsgToOctomap(req.external_bb.max);

    Mesh all_mesh, box_mesh;
    int num_boxes = 0;

    // iterate through all existing boxes in the area of interest
    for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(ex_min, ex_max),
                                            end = octree->end_leafs_bbx();
         it != end; ++it) {
      if (it->getOccupancy() > 0.5) {
        // add box
        box_mesh = create_cube();
        box_mesh.scale(Vec3(it.getSize(), it.getSize(), it.getSize()));
        box_mesh.translate(
            Vec3(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));

        all_mesh += box_mesh;
        num_boxes++;
      }
    }
    // save mesh to file
    if (num_boxes > 0) {
      all_mesh.stl_write_binary(req.stl_path);
      res.flag = true;
    } else {
      res.flag = false;
    }

    return true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "octomap_to_reconstruction");
  ros::NodeHandle nh;

  OctomapToReconstruction srv = OctomapToReconstruction(&nh);

  ROS_INFO("%s: service ready", ros::this_node::getName().c_str());

  ros::spin();

  return 0;
}
