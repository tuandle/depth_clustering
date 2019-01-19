#ifndef SRC_ROS_BRIDGE_GROUND_PUBLISHER_H_
#define SRC_ROS_BRIDGE_GROUND_PUBLISHER_H_

#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "communication/abstract_client.h"
#include "projections/projection_params.h"
#include "utils/cloud.h"
#include "utils/radians.h"

namespace depth_clustering {

using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;
typedef pcl::PointXYZL PointT;

class GroundPublisher : public AbstractClient<Cloud> {
 public:
  GroundPublisher(ros::NodeHandle* node_handle, const std::string& frame_id,
                  const std::string& topic_ground);
  virtual ~GroundPublisher(){};

  void OnNewObjectReceived(const Cloud& ground_cloud, const int id) override;
  void GroundImageToPcl(const Cloud& ground_cloud,
                        pcl::PointCloud<PointT>& pcl_cloud);
  void PublishGround(const pcl::PointCloud<PointT>& pcl_cloud);

 protected:
  ros::NodeHandle* _node_handle;
  std::string _frame_id, _topic_ground;
  ros::Publisher _ground_pub;
};
}  // namespace depth_clustering
#endif  // SRC_ROS_BRIDGE_GROUND_PUBLISHER_H_