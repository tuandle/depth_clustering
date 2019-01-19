#include "ros_bridge/cloud_clusters_publisher.h"
#include <algorithm>
#include <cmath>
#include <memory>

#include "image_labelers/diff_helpers/angle_diff.h"
#include "image_labelers/diff_helpers/simple_diff.h"
#include "image_labelers/linear_image_labeler.h"
#include "pcl/common/common.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"

#include <ros/console.h>

namespace depth_clustering {
using std::abs;

using cv::DataType;
using cv::Mat;
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;
using std::to_string;
using time_utils::Timer;

CloudClustersRosPublisher::CloudClustersRosPublisher(
    ros::NodeHandle* node_handle, const std::string& frame_id,
    const std::string& topic_cluster_clouds)
    : _node_handle{node_handle},
      _frame_id{frame_id},
      _topic_cluster_clouds{topic_cluster_clouds},
      _cloud_pub{
          _node_handle->advertise<PointCloud2>(_topic_cluster_clouds, 1)} {}

void CloudClustersRosPublisher::OnNewObjectReceived(
    const std::unordered_map<uint16_t, Cloud>& clouds, const int id) {
  Timer total_timer;
  pcl::PointCloud<PointT> pcl_cloud;
  ImageToPcl(clouds, pcl_cloud);
  PublishCloud(pcl_cloud);
}

void CloudClustersRosPublisher::ImageToPcl(
    const std::unordered_map<uint16_t, Cloud>& clouds,
    pcl::PointCloud<PointT>& pcl_cloud) {
  int i = 0;
  for (const auto& kv : clouds) {
    const auto& cluster = kv.second;
    if (cluster.empty()) {
      fprintf(stderr, "cluster is empty\n");
      continue;
    }
    /*fprintf(stderr, "cluster is NOT empty\n");*/
    pcl::PointCloud<PointT> pcl_temp;
    PointT min_p;
    PointT max_p;
    for (const auto& point : cluster.points()) {
      PointT p;
      p.x = point.x(); 
      p.y = point.y();
      p.z = point.z();
      p.label = kv.first; // change back to i for clusterers
      pcl_temp.push_back(p);
    }
    pcl_cloud += pcl_temp;
    ++i;
    
    /*pcl::getMinMax3D(pcl_temp, min_p, max_p);

    double dif_x = max_p.x - min_p.x;
    double dif_y = max_p.y - min_p.y;
    double dif_z = max_p.z - min_p.z;

    if ((dif_x) * (dif_y) < 5 ||
        (dif_x * dif_x + dif_y * dif_y + dif_z * dif_z) < 3) {
      pcl_cloud += pcl_temp;
      i++;
    }*/
  }
}

void CloudClustersRosPublisher::PublishCloud(
    const pcl::PointCloud<PointT>& pcl_cloud) {
  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(pcl_cloud, cloud2);
  cloud2.header.frame_id = _frame_id;
  cloud2.header.stamp = ros::Time::now();
  _cloud_pub.publish(cloud2);
}

}  // namespace depth_clustering
