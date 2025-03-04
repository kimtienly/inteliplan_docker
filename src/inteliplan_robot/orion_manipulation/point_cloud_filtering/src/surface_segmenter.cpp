//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#include "surface_segmenter.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <limits>

#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace point_cloud_filtering {

SurfaceSegmenter::SurfaceSegmenter(ros::NodeHandle* nh) {
  ros_node_handle = nh;
  std::snprintf(node_name, 128, "%s", ros::this_node::getName().c_str());

  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("head_rgbd_sensor_rgb_frame",
                                                            "vis_markers"));
  visual_tools->trigger();

  surface_point_cloud_pub =
      nh->advertise<sensor_msgs::PointCloud2>("surface_cloud", 1, true);
}

void SurfaceSegmenter::StartServices(void) {
  service_server = ros_node_handle->advertiseService(
      "detect_surface", &SurfaceSegmenter::ServiceCallback, this);
  ROS_INFO("%s: /detect_surface service ready", ros::this_node::getName().c_str());

  iterative_service_server = ros_node_handle->advertiseService(
      "detect_surface_iterative", &SurfaceSegmenter::IterativeServiceCallback, this);
  ROS_INFO("%s: /detect_surface_iterative service ready", ros::this_node::getName().c_str());

  surface_selection_server = ros_node_handle->advertiseService(
      "select_surface", &SurfaceSegmenter::SurfaceSelectionCallback, this);
  ROS_INFO("%s: /select_surface service ready", ros::this_node::getName().c_str());

  sample_service_server = ros_node_handle->advertiseService(
      "sample_cloud", &SurfaceSegmenter::SampleServiceCallback, this);
  ROS_INFO("%s: /sample_cloud service ready", ros::this_node::getName().c_str());
}

bool SurfaceSegmenter::ServiceCallback(
    point_cloud_filtering::DetectSurface::Request& req,
    point_cloud_filtering::DetectSurface::Response& res) {
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
  bool success = SeparatePointCloudByPlanePipeline(
      query_point, search_axis, eps_degrees_tolerance, crop_box_dimension,
      plane_pose, plane_coeff, non_surface_cloud);

  if (success == true) {
    geometry_msgs::Transform transform_msg;
    tf::transformEigenToMsg(plane_pose, transform_msg);
    res.plane_axis = transform_msg;
    res.success = true;

    ROS_INFO("This is the new version");
    return true;
  } else {
    return false;
  }
}

bool SurfaceSegmenter::IterativeServiceCallback(
    point_cloud_filtering::DetectSurfaceIterative::Request& req,
    point_cloud_filtering::DetectSurfaceIterative::Response& res) {
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
  PointCloudC::Ptr surface_cloud(new PointCloudC());
  PointCloudC::Ptr input_cloud(new PointCloudC());
  PointCloudC::Ptr combined_surface_cloud(new PointCloudC());

  std::vector<sensor_msgs::PointCloud2> surface_vect;

  bool crop_success = GetCroppedRGBDCloud(input_cloud, crop_box_dimension, query_point);

  if (crop_success == false){
    res.success = false;
    return true;
  }
  int total_cloud_size = input_cloud->size();

  ROS_INFO("Total cloud size: %d", total_cloud_size);

  bool flag, success;
  int surface_count = 0;
  flag = true;
  while (flag == true){
    success = SeparatePointCloudByPlane(input_cloud, query_point, search_axis,
                                        eps_degrees_tolerance, crop_box_dimension,
                                        plane_coeff, non_surface_cloud, surface_cloud, false);
    flag = false;
    if (success == true){
        flag = true;
        if (surface_cloud->size() > float(total_cloud_size)*0.04) {
            *combined_surface_cloud += *surface_cloud;
            surface_count ++;

            sensor_msgs::PointCloud2 msg_tmp_cloud;
            PointCloudPtrToMsg(surface_cloud, msg_tmp_cloud);
            surface_vect.push_back(msg_tmp_cloud);
        }
    }
    if (non_surface_cloud->size() < float(total_cloud_size)*0.3){
        flag = false;
    }
    input_cloud = non_surface_cloud;
  }

  ROS_INFO("Surface count: %d", surface_count);
  ROS_INFO("Combined surface size: %ld", combined_surface_cloud->size());

  if (surface_count > 0) {
    if (req.hide_display == false) {
        sensor_msgs::PointCloud2 msg_cloud_out;
        PointCloudPtrToMsg(combined_surface_cloud, msg_cloud_out);
        surface_point_cloud_pub.publish(msg_cloud_out);
    }

    res.success = true;
    res.surfaces = surface_vect;

    ROS_INFO("This is the iterative version");
    return true;
  } else {
    return false;
  }
}

bool SurfaceSegmenter::SurfaceSelectionCallback(
    point_cloud_filtering::SelectSurface::Request& req,
    point_cloud_filtering::SelectSurface::Response& res) {
  res.success = false;

  PointCloudC::Ptr surface_cloud(new PointCloudC());
  PointCloudC::Ptr transformed_cloud(new PointCloudC());
  float min_z, max_z;

  //------ Extract variables from request --------
  point_cloud_filtering::DetectSurfaceIterative::Request iter_service_req;
  point_cloud_filtering::DetectSurfaceIterative::Response iter_service_res;

  iter_service_req.search_axis = req.search_axis;
  iter_service_req.eps_degrees_tolerance = req.eps_degrees_tolerance;
  iter_service_req.search_box_dimension = req.search_box_dimension;
  iter_service_req.hide_display = true;

  float min_height = req.min_height;

  ROS_INFO("%s: performing iterative search for horizontal surfaces", node_name);
  bool iter_success = IterativeServiceCallback(iter_service_req, iter_service_res);

  if (iter_success == false){
    ROS_ERROR("%s: Failed to find surfaces.", node_name);
    return false;
  }

  // https://github.com/stereolabs/zed-ros-wrapper/issues/393
  ROS_INFO("%s: looking up transform from map to camera frame", node_name);
  tf::TransformListener listener;
  tf::StampedTransform to_map_transform;
  try{
    listener.waitForTransform("/map", "/head_rgbd_sensor_rgb_frame", ros::Time(), ros::Duration(5.0));
    listener.lookupTransform("/map", "/head_rgbd_sensor_rgb_frame", ros::Time(), to_map_transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s: %s", node_name, ex.what());
    return false;
  }

  ROS_INFO("%s: selecting prefered area", node_name);
  std::vector<sensor_msgs::PointCloud2> surface_vect = iter_service_res.surfaces;
  float max_area = std::numeric_limits<float>::min();
  float now_area;
  int max_area_idx = -1;
  int idx = 0;

  float sampled_max_height = std::numeric_limits<float>::min();
  int max_height_idx;

  for(sensor_msgs::PointCloud2 surface_cloud_msg : surface_vect){
    MsgToPointCloud(surface_cloud_msg, surface_cloud);
    pcl_ros::transformPointCloud(*surface_cloud, *transformed_cloud, to_map_transform);

    GetCloudMinMaxZ(transformed_cloud, min_z, max_z);

    if (min_z > min_height){
      now_area = transformed_cloud->size();
      if (now_area > max_area){
        max_area = now_area;
        max_area_idx = idx;
      }
    }

    if (sampled_max_height < min_z){
        max_height_idx = idx;
        sampled_max_height = min_z;
    }

    idx++;
  }

  int selection_idx;
  if (max_area_idx >= 0){
    selection_idx = max_area_idx;
  }
  else{
    ROS_ERROR("%s: No surface matches requirement, using highest surface.", node_name);
    selection_idx = max_height_idx;
  }

  ROS_INFO("%s: surface found", node_name);
  sensor_msgs::PointCloud2 msg_cloud_out;
  MsgToPointCloud(surface_vect[selection_idx], surface_cloud);
  PointCloudPtrToMsg(surface_cloud, msg_cloud_out);
  surface_point_cloud_pub.publish(msg_cloud_out);

  sensor_msgs::PointCloud2 msg_transformed_cloud_out;
  pcl_ros::transformPointCloud(*surface_cloud, *transformed_cloud, to_map_transform);
  PointCloudPtrToMsg(transformed_cloud, msg_transformed_cloud_out);

  res.success = true;
  res.surface = msg_transformed_cloud_out;

  return true;
}

bool SurfaceSegmenter::SampleServiceCallback(
    point_cloud_filtering::SampleCloud::Request& req,
    point_cloud_filtering::SampleCloud::Response& res) {
  res.success = false;

  std::vector<geometry_msgs::Point> samples;

  ROS_INFO("%s: Sampling point cloud", node_name);

  PointCloudC::Ptr cloud(new PointCloudC());
  PointCloudC::Ptr sampled_cloud(new PointCloudC());
  point_cloud_filtering::MsgToPointCloud(req.cloud, cloud);
  pcl::RandomSample <pcl::PointXYZRGB> random;
  random.setInputCloud(cloud);
  random.setSeed (std::rand ());
  random.setSample((unsigned int)(req.sample_num));
  random.filter(*sampled_cloud);

  bool found_flag = false;
  for (auto point : sampled_cloud->points){
      geometry_msgs::Point sample_position;
      sample_position.x = point.x;
      sample_position.y = point.y;
      sample_position.z = point.z;

      samples.push_back(sample_position);
  }

  res.samples = samples;
  res.success = true;

  return true;
}


bool SurfaceSegmenter::SeparatePointCloudByPlanePipeline(
    Eigen::Vector3d query_point, Eigen::Vector3d search_axis, float eps_degrees_tolerance,
    float crop_box_dimension, Eigen::Isometry3d& plane_pose,
    pcl::ModelCoefficients::Ptr plane_coeff, PointCloudC::Ptr non_surface_cloud) {

  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  PointCloudC::Ptr surface_cloud(new PointCloudC());

  bool crop_success = GetCroppedRGBDCloud(cropped_cloud, crop_box_dimension, query_point);

  bool success;
  if (crop_success == true) {
    success = SeparatePointCloudByPlane(cropped_cloud, query_point, search_axis,
                                        eps_degrees_tolerance, crop_box_dimension,
                                        plane_coeff, non_surface_cloud, surface_cloud);
  } else {
    success = false;
  }

  if (success == true) {
    GetTransformOnPlane(plane_pose, plane_coeff, query_point, search_axis);
    PublishPlaneMarker(plane_pose, crop_box_dimension);
  }

  return success;
}


bool SurfaceSegmenter::SeparatePointCloudByPlane(
    PointCloudC::Ptr input_cloud, Eigen::Vector3d query_point, Eigen::Vector3d search_axis, float eps_degrees_tolerance,
    float crop_box_dimension,
    pcl::ModelCoefficients::Ptr& plane_coeff, PointCloudC::Ptr& non_surface_cloud, PointCloudC::Ptr& surface_cloud, bool display_flag) {
  // Carry out cropping, plane detection, plane segmentation


  //------ Get the surface in the point cloud --------
  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
  SegmentPlane(input_cloud, plane_inliers, search_axis, eps_degrees_tolerance,
               plane_coeff);

  Eigen::Vector3d plane_normal (plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2]);

  ROS_INFO("Dot product: %f", search_axis.normalized().dot(plane_normal.normalized()));

  if (plane_inliers->indices.empty()) {
    ROS_ERROR("%s: could not estimate a planar model for the given dataset.", node_name);
    return false;
  }

  //------ Extract the plane subset of cropped cloud into surface_cloud --------
  FilterCloudByIndices(input_cloud, surface_cloud, plane_inliers, false);
  ROS_INFO("%s: surface cloud has %ld points", node_name, surface_cloud->size());

  if (display_flag == true){
      sensor_msgs::PointCloud2 msg_cloud_out;
      pcl::toROSMsg(*surface_cloud, msg_cloud_out);
      surface_point_cloud_pub.publish(msg_cloud_out);
  }

  //------ Extract the non-plane subset of cropped cloud into non_surface_cloud --------
  FilterCloudByIndices(input_cloud, non_surface_cloud, plane_inliers, true);

  return true;
}

bool SurfaceSegmenter::GetCroppedRGBDCloud(PointCloudC::Ptr& first_cropped_cloud,
                                           float crop_box_dimension,
                                           Eigen::Vector3d query_point) {
  //------ Receive one point cloud from RGBD sensor --------
  sensor_msgs::PointCloud2ConstPtr msg =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in", ros::Duration(3));
  if (msg == NULL) {
    ROS_ERROR("%s: no point cloud received in timeout", node_name);
    return false;
  }

  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(*msg, *cloud);
  ROS_INFO("%s: received point cloud with %ld points", node_name, cloud->size());

  //------ Crop the point cloud roughly 15 cm around the target --------
  Eigen::Vector3d min_crop_pt = query_point.array() - crop_box_dimension;
  Eigen::Vector3d max_crop_pt = query_point.array() + crop_box_dimension;
  ROS_INFO_STREAM(node_name << ": crop points:  " << min_crop_pt << ", " << max_crop_pt);
  PublishCropBoundingBoxMarker(min_crop_pt, max_crop_pt);

  CropCloud(cloud, first_cropped_cloud, ToHomogeneousCoordsVector(min_crop_pt),
            ToHomogeneousCoordsVector(max_crop_pt));

  ROS_INFO("%s: box cropped cloud has %ld points", node_name,
           first_cropped_cloud->size());

  return true;
}

void SurfaceSegmenter::GetTransformOnPlane(Eigen::Isometry3d& plane_pose,
                                                        pcl::ModelCoefficients::Ptr plane_coeff,
                                                        Eigen::Vector3d query_point,
                                                        Eigen::Vector3d search_axis) {
  Eigen::Vector3d plane_projection;
  CalculatePlaneProjection(plane_coeff, query_point, plane_projection);

  ROS_INFO_STREAM(node_name << ": calculated plane point:  " << plane_projection);
  ROS_INFO_STREAM(node_name << ": plane parameters:  " << *plane_coeff);
  plane_pose = GetPlanePose(plane_coeff, plane_projection, search_axis);
}

void SurfaceSegmenter::CalculatePlaneProjection(pcl::ModelCoefficients::Ptr plane_coeff,
                                                Eigen::Vector3d point,
                                                Eigen::Vector3d& closest_point) {
  // Calculate the closest point to a point on a plane.
  // ax + by + cz + d = 0
  float a = plane_coeff->values[0];
  float b = plane_coeff->values[1];
  float c = plane_coeff->values[2];
  float d = plane_coeff->values[3];
  Eigen::Vector3d plane_abc = Eigen::Vector3d(a, b, c);

  float k = (-a * point[0] - b * point[1] - c * point[2] - d) /
            static_cast<float>(a * a + b * b + c * c);

  closest_point = point + (plane_abc * k);
}

void SurfaceSegmenter::PublishTransformMarker(geometry_msgs::Transform transform) {
  Eigen::Isometry3d transformed_pose = Eigen::Isometry3d::Identity();
  tf::transformMsgToEigen(transform, transformed_pose);
  visual_tools->publishAxis(transformed_pose, rviz_visual_tools::SMALL);
  visual_tools->trigger();
}

void SurfaceSegmenter::PublishCropBoundingBoxMarker(Eigen::Vector3d min_crop_pt,
                                                    Eigen::Vector3d max_crop_pt) {
  Eigen::Isometry3d identity_pose = Eigen::Isometry3d::Identity();
  visual_tools->publishWireframeCuboid(identity_pose, min_crop_pt, max_crop_pt,
                                       rviz_visual_tools::BLUE);
  visual_tools->trigger();
}

void SurfaceSegmenter::PublishPlaneMarker(Eigen::Isometry3d plane_pose, float plane_size,
                                          float arrow_size) {
  // Publish plane and normal vector markers
  visual_tools->publishXYPlane(plane_pose, rviz_visual_tools::BLUE, plane_size);
  visual_tools->publishZArrow(plane_pose, rviz_visual_tools::BLUE,
                              rviz_visual_tools::SMALL, arrow_size);
  visual_tools->trigger();
}

Eigen::Vector3d Vector3dFromZRotation(const geometry_msgs::Quaternion& m) {
  // Rotate the Z unit vector by a given rotation
  Eigen::Quaterniond rotQ;
  tf::quaternionMsgToEigen(m, rotQ);
  Eigen::Quaterniond z;
  z.w() = 0;
  z.vec() = Eigen::Vector3d::UnitZ();
  Eigen::Quaterniond rotatedP = rotQ * z * rotQ.inverse();
  Eigen::Vector3d rotated = rotatedP.vec();
  return rotated;
}

Eigen::Isometry3d GetPlanePose(pcl::ModelCoefficients::Ptr plane_coeff,
                               Eigen::Vector3d plane_projection,
                               Eigen::Vector3d search_axis) {
  // Generate a pose representing the plane. Enforce that the normal is in the direction
  // of search_axis.
  Eigen::Vector3d plane_normal = Eigen::Vector3d(
      plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2]);

  double dot_product = plane_normal.dot(search_axis);
  if (dot_product < 0) {
    // Flip the plane normal to match search_axis direction
    plane_normal = -1 * plane_normal;
  }

  Eigen::Quaterniond rotQ =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), plane_normal);

  Eigen::Isometry3d plane_pose;
  plane_pose.setIdentity();
  plane_pose.rotate(rotQ);
  plane_pose.translation() = plane_projection;

  return plane_pose;
}

void GetCloudMinMaxX(const PointCloudC::Ptr cloud, float& min_x, float& max_x) {
  // Get the min and max x/y/z values of a point cloud
  max_x = std::numeric_limits<float>::min();
  min_x = std::numeric_limits<float>::max();

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (cloud->points[i].x > max_x) {
      max_x = cloud->points[i].x;
    }
    if (cloud->points[i].x < min_x) {
      min_x = cloud->points[i].x;
    }
  }
}

void GetCloudMinMaxY(const PointCloudC::Ptr cloud, float& min_y, float& max_y) {
  // Get the min and max x/y/z values of a point cloud
  max_y = std::numeric_limits<float>::min();
  min_y = std::numeric_limits<float>::max();

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (cloud->points[i].y > max_y) {
      max_y = cloud->points[i].y;
    }
    if (cloud->points[i].y < min_y) {
      min_y = cloud->points[i].y;
    }
  }
}

void GetCloudMinMaxZ(const PointCloudC::Ptr cloud, float& min_z, float& max_z) {
  // Get the min and max x/z/z values of a point cloud
  max_z = std::numeric_limits<float>::min();
  min_z = std::numeric_limits<float>::max();

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (cloud->points[i].z > max_z) {
      max_z = cloud->points[i].z;
    }
    if (cloud->points[i].z < min_z) {
      min_z = cloud->points[i].z;
    }
  }
}

void SegmentPlane(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices,
                  Eigen::Vector3d axis, float eps_degrees_tolerance,
                  pcl::ModelCoefficients::Ptr coeff) {
  // NOTE that the model coefficients returned by this function may be *either direction*:
  // i.e. they plane normal may point the opposite direction to the search axis.
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);

  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);

  // Set the distance to the plane for a point to be a plane inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  seg.setMaxIterations(500);

  // Make sure that the plane is perpendicular to given axis, given some degree tolerance.
  seg.setAxis(axis.cast<float>());
  seg.setEpsAngle(pcl::deg2rad(eps_degrees_tolerance));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  pcl::ModelCoefficients plane_coeff;
  seg.segment(*indices, *coeff);
}

void FilterByPlane(PointCloudC::Ptr in_cloud, pcl::ModelCoefficients::Ptr plane_coeff,
                   pcl::PointIndices::Ptr plane_side_points) {
  float a = plane_coeff->values[0];
  float b = plane_coeff->values[1];
  float c = plane_coeff->values[2];
  float d = plane_coeff->values[3];
  Eigen::Vector4f plane_abcd = Eigen::Vector4f(a, b, c, d);

  pcl::PlaneClipper3D<PointC> crop = pcl::PlaneClipper3D<PointC>(plane_abcd);
  crop.clipPointCloud3D(*in_cloud, plane_side_points->indices);

  return;
}

Eigen::Vector4f ToHomogeneousCoordsVector(const Eigen::Vector3d in) {
  Eigen::Vector4f out =
      Eigen::Vector4f(static_cast<double>(in[0]), static_cast<double>(in[1]),
                      static_cast<double>(in[2]), 1.0);
  return out;
}

void FilterCloudByIndices(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud,
                          pcl::PointIndices::Ptr indices, bool invert) {
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(indices);
  extract.setNegative(invert);
  extract.filter(*out_cloud);
}

void CropCloud(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud,
               Eigen::Vector4f min_p, Eigen::Vector4f max_p) {
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(in_cloud);
  crop.setMin(min_p);
  crop.setMax(max_p);
  crop.filter(*out_cloud);
}

void ClusterCloud(PointCloudC::Ptr cloud, std::vector<pcl::PointIndices>* clusters,
                  double cluster_tolerance, int min_cluster_points,
                  double max_cluster_proportion) {
  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(min_cluster_points);
  euclid.setMaxClusterSize(std::round(max_cluster_proportion * cloud->size()));
  euclid.extract(*clusters);
}

void GetLargestCluster(std::vector<pcl::PointIndices>* clusters,
                       PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud) {
  // Get the cluster with the largest number of points in the cluster

  int clust_size, cluster_index;
  clust_size = 0;
  cluster_index = 0;
  for (size_t i = 0; i < clusters->size(); ++i) {
    if (clusters->at(i).indices.size() > clust_size) {
      cluster_index = i;
    }
  }

  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  pcl::ExtractIndices<PointC> indices_extracter;
  *indices = clusters->at(cluster_index);
  indices_extracter.setInputCloud(in_cloud);
  indices_extracter.setIndices(indices);
  indices_extracter.filter(*out_cloud);
}

void GetClosestCluster(std::vector<pcl::PointIndices>* clusters, Eigen::Vector3d query_p,
                       PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud) {
  // Get the clustered points closest to the point query_p

  int cluster_index;
  float closest_squared_dist = std::numeric_limits<float>::max();

  PointC* point;
  float squared_dist;
  float query_x = static_cast<float>(query_p(0));
  float query_y = static_cast<float>(query_p(1));
  float query_z = static_cast<float>(query_p(2));

  for (size_t i = 0; i < clusters->size(); ++i) {
    for (std::vector<int>::const_iterator point_it = clusters->at(i).indices.begin();
         point_it != clusters->at(i).indices.end(); ++point_it) {
      point = &(in_cloud->points[*point_it]);
      squared_dist = (point->x - query_x) * (point->x - query_x) +
             (point->y - query_y) * (point->y - query_y) +
             (point->z - query_z) * (point->z - query_z);
      if (squared_dist < closest_squared_dist) {
        cluster_index = i;
        closest_squared_dist = squared_dist;
      }
    }
  }

  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  pcl::ExtractIndices<PointC> indices_extracter;
  *indices = clusters->at(cluster_index);
  indices_extracter.setInputCloud(in_cloud);
  indices_extracter.setIndices(indices);
  indices_extracter.filter(*out_cloud);
}

void PointCloudPtrToMsg(PointCloudC::Ptr cloud, sensor_msgs::PointCloud2& msg){
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "head_rgbd_sensor_link";
}

void MsgToPointCloud(sensor_msgs::PointCloud2 msg, PointCloudC::Ptr& cloud_ptr){
    pcl::fromROSMsg(msg, *cloud_ptr);
}

}  // namespace point_cloud_filtering
