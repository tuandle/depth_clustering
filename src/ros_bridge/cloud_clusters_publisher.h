#ifndef SRC_ROS_BRIDGE_CLOUD_CLUSTERS_PUBLISHER_H_
#define SRC_ROS_BRIDGE_CLOUD_CLUSTERS_PUBLISHER_H_

#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_map>

#include "communication/abstract_client.h"
#include "projections/projection_params.h"
#include "utils/cloud.h"
#include "utils/radians.h"

namespace depth_clustering {
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;
typedef pcl::PointXYZL PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;

class CloudClustersRosPublisher
    : public AbstractClient<std::unordered_map<uint16_t, Cloud>> {
 public:
  CloudClustersRosPublisher(ros::NodeHandle* node_handle,
                            const std::string& frame_id,
                            const std::string& topic_cluster_clouds);
  virtual ~CloudClustersRosPublisher(){};
  void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds,
                           const int id) override;

  void ImageToPcl(const std::unordered_map<uint16_t, Cloud>& clouds,
                  pcl::PointCloud<PointT>& pcl_cloud);

  void PublishCloud(const pcl::PointCloud<PointT>& pcl_cloud);

 protected:
  ros::NodeHandle* _node_handle;
  std::string _frame_id, _topic_cluster_clouds;
  ros::Publisher _cloud_pub;
};
}  // namespace depth_clustering

#endif  // SRC_ROS_BRIDGE_CLOUD_CLUSTERS_PUBLISHER_H_
