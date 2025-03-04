//
// Reconstructs STL meshes from Octomap occupancy map input.
//

#include <eigen_conversions/eigen_msg.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "manipulation/CheckPlacement.h"
#include "manipulation/FindPlacementOnEmptySurface.h"
#include "point_cloud_filtering/SelectSurface.h"
#include "point_cloud_filtering/SampleCloud.h"

class PlacementOnEmptySurfaceFinder {
 private:
  ros::ServiceServer find_placement_service;
  ros::ServiceClient select_surface_client;
  ros::ServiceClient sample_surface_client;
  ros::ServiceClient check_placement_client;

 public:
  explicit PlacementOnEmptySurfaceFinder(ros::NodeHandle* nh) {
    ROS_INFO("Initializing service......");
    find_placement_service = nh->advertiseService(
        "FindPlacementOnEmptySurface", &PlacementOnEmptySurfaceFinder::service_callback, this);
    select_surface_client = nh->serviceClient<point_cloud_filtering::SelectSurface>("select_surface");
    ROS_INFO("select_surface service ready");
    sample_surface_client = nh->serviceClient<point_cloud_filtering::SampleCloud>("sample_cloud");
    ROS_INFO("sample_cloud service ready");
    check_placement_client = nh->serviceClient<manipulation::CheckPlacement>("CheckPlacement");
    ROS_INFO("CheckPlacement service ready");
  }

  bool service_callback(manipulation::FindPlacementOnEmptySurface::Request& req,
                        manipulation::FindPlacementOnEmptySurface::Response& res) {
    res.success = false;

    manipulation::CheckPlacement::Request check_service_req;
    manipulation::CheckPlacement::Response check_service_res;

    point_cloud_filtering::SelectSurface::Request select_service_req;
    point_cloud_filtering::SelectSurface::Response select_service_res;

    select_service_req.search_axis = req.search_axis;
    select_service_req.eps_degrees_tolerance = req.eps_degrees_tolerance;
    select_service_req.search_box_dimension = req.search_box_dimension;
    select_service_req.min_height = req.min_surface_height;

    if (!select_surface_client.call(select_service_req, select_service_res)) {
      ROS_ERROR("Failed to call service select_surface");
      return false;
    }

    point_cloud_filtering::SampleCloud::Request sample_service_req;
    point_cloud_filtering::SampleCloud::Response sample_service_res;

    sample_service_req.cloud = select_service_res.surface;
    sample_service_req.sample_num = 100;

    if (!sample_surface_client.call(sample_service_req, sample_service_res)) {
      ROS_ERROR("Failed to call service sample_cloud");
      return false;
    }

    bool found_flag = false;
    int idx = 0;
    ROS_INFO("Number of sample points received: %ld", sample_service_res.samples.size());
    for (geometry_msgs::Point point: sample_service_res.samples){
        geometry_msgs::Point test_position;
        test_position = point;
        test_position.z = point.z + req.dims.z / 2 + 0.05;

        check_service_req.center = test_position;
        check_service_req.dims = req.dims;
        check_service_req.maxHeight = req.maxHeight;

        if (!check_placement_client.call(check_service_req, check_service_res)) {
          ROS_ERROR("Failed to call service CheckPlacement");
          return false;
        }

        if (check_service_res.isAvailable && check_service_res.isSupported){
            res.success = true;
            res.position = {static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)};
            found_flag = true;
            break;
        }

        ROS_INFO("Testing position %ld", idx);
        if (check_service_res.isAvailable == true){
            ROS_INFO("isAvailable");
        }
        if (check_service_res.isSupported == true){
            ROS_INFO("isSupported");
        }

        idx++;
    }

    if (found_flag == false){
        ROS_ERROR("Couldn't find an appropriate placement option");
        return false;
    }

    ROS_INFO("Successful at finding a placement option");
    return true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "find_placement_on_empty_surface");
  ros::NodeHandle nh;

  PlacementOnEmptySurfaceFinder srv = PlacementOnEmptySurfaceFinder(&nh);

  ROS_INFO("%s: service ready", ros::this_node::getName().c_str());

  ros::spin();

  return 0;
}
