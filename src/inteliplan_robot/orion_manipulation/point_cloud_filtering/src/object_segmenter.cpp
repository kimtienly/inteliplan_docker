//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#include "object_segmenter.h"

#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace point_cloud_filtering {

ObjectSegmenter::ObjectSegmenter(ros::NodeHandle* nh) : SurfaceSegmenter(nh) {
  crop_point_cloud_pub =
      nh->advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);

  object_point_cloud_pub =
      nh->advertise<sensor_msgs::PointCloud2>("object_cloud", 1, true);
}

void ObjectSegmenter::StartServices(void) {
  service_server = ros_node_handle->advertiseService(
      "segment_object", &ObjectSegmenter::ServiceCallback, this);
  ROS_INFO("%s: /segment_object service ready", ros::this_node::getName().c_str());
}

bool ObjectSegmenter::ServiceCallback(
    point_cloud_filtering::SegmentObject::Request& req,
    point_cloud_filtering::SegmentObject::Response& res) {
  res.success = false;

  //------ Reset any visualisation --------
  visual_tools->deleteAllMarkers();
  // PublishTransformMarker(req.search_axis);

  //------ Extract variables from request --------
  float eps_degrees_tolerance = req.eps_degrees_tolerance;
  float crop_box_dimension = req.search_box_dimension;
  Eigen::Vector3d query_point;
  tf::vectorMsgToEigen(req.search_axis.translation, query_point);
  Eigen::Vector3d search_axis = Vector3dFromZRotation(req.search_axis.rotation);

  //------ Process point cloud, expecting a surface --------
  Eigen::Isometry3d plane_pose;
  pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
  PointCloudC::Ptr non_surface_cloud(new PointCloudC());
  bool success = SeparatePointCloudByPlanePipeline(query_point, search_axis,
                                           eps_degrees_tolerance, crop_box_dimension,
                                           plane_pose, plane_coeff, non_surface_cloud);

  if (success == false) {
    return false;
  }

  //------ Process remaining point cloud, which should include the object --------
  ROS_INFO("Surface-removed cloud has %ld points", non_surface_cloud->size());
  sensor_msgs::PointCloud2 msg_cloud_out;

  //------ Crop the point cloud using the plane parameters --------
  // non_surface_cloud may still have anything below the surface still in, e.g. table legs
  // Use the plane to crop out everything below the surface.
  pcl::PointIndices::Ptr filter_indices(new pcl::PointIndices());
  PointCloudC::Ptr plane_cropped_cloud(new PointCloudC());
  FilterByPlane(non_surface_cloud, plane_coeff, filter_indices);
  FilterCloudByIndices(non_surface_cloud, plane_cropped_cloud, filter_indices, false);
  ROS_INFO("Object cloud has %ld points", plane_cropped_cloud->size());
  pcl::toROSMsg(*plane_cropped_cloud, msg_cloud_out);
  crop_point_cloud_pub.publish(msg_cloud_out);

  //------ Cluster the cloud and output the cluster closest to the query point --------
  std::vector<pcl::PointIndices> clusters;
  PointCloudC::Ptr object_cloud(new PointCloudC());
  ClusterCloud(plane_cropped_cloud, &clusters, 0.01, 50, 1.0);
  GetClosestCluster(&clusters, query_point, plane_cropped_cloud, object_cloud);

  //------ Publish the object point cloud --------
  pcl::toROSMsg(*object_cloud, msg_cloud_out);
  object_point_cloud_pub.publish(msg_cloud_out);

  geometry_msgs::Transform transform_msg;
  tf::transformEigenToMsg(plane_pose, transform_msg);
  res.plane_axis = transform_msg;
  res.success = true;
  return true;
}

}  // namespace point_cloud_filtering
