//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#ifndef POINT_CLOUD_FILTERING_INCLUDE_POINT_CLOUD_FILTERING_OBJECT_SEGMENTER_H_
#define POINT_CLOUD_FILTERING_INCLUDE_POINT_CLOUD_FILTERING_OBJECT_SEGMENTER_H_

#include "point_cloud_filtering/SegmentObject.h"
#include "surface_segmenter.h"

namespace point_cloud_filtering {

class ObjectSegmenter : public SurfaceSegmenter {
 public:
  explicit ObjectSegmenter(ros::NodeHandle* nh);
  void StartServices(void);

 protected:
  ros::Publisher crop_point_cloud_pub;
  ros::Publisher object_point_cloud_pub;

  bool ServiceCallback(
      point_cloud_filtering::SegmentObject::Request& req,
      point_cloud_filtering::SegmentObject::Response& res);
};

}  // namespace point_cloud_filtering

#endif  // POINT_CLOUD_FILTERING_INCLUDE_POINT_CLOUD_FILTERING_OBJECT_SEGMENTER_H_
