#include "ros_bridge/ground_publisher.h"
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

GroundPublisher::GroundPublisher(ros::NodeHandle* node_handle,
                                 const std::string& frame_id,
                                 const std::string& topic_ground)
    : _node_handle{node_handle},
      _frame_id{frame_id},
      _topic_ground{topic_ground},
      _ground_pub{_node_handle->advertise<PointCloud2>(_topic_ground, 1)} {}

void GroundPublisher::OnNewObjectReceived(const Cloud& ground_cloud,
                                          const int id) {
  Timer total_timer;
  pcl::PointCloud<PointT> pcl_cloud;
  GroundImageToPcl(ground_cloud, pcl_cloud);
  PublishGround(pcl_cloud);
}

void GroundPublisher::GroundImageToPcl(const Cloud& ground_cloud,
                                       pcl::PointCloud<PointT>& pcl_cloud) {
  int i = 1;  // conform with loam's label for ground
  for (const auto& point : ground_cloud.points()) {
    PointT p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    p.label = i;
    pcl_cloud.push_back(p);
  }
}

void GroundPublisher::PublishGround(const pcl::PointCloud<PointT>& pcl_cloud) {
  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(pcl_cloud, cloud2);
  cloud2.header.frame_id = _frame_id;
  cloud2.header.stamp = ros::Time::now();
  _ground_pub.publish(cloud2);
}
}  // namespace depth_clustering