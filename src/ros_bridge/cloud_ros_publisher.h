#ifndef SRC_ROS_BRIDGE_CLOUD_ROS_PUBLISHER_H_
#define SRC_ROS_BRIDGE_CLOUD_ROS_PUBLISHER_H_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <string>
#include <unordered_map>

#include "communication/abstract_sender.h"
#include "utils/cloud.h"
#include "utils/pose.h"
#include "utils/useful_typedefs.h"

namespace depth_clustering {
/**
 * @brief      Class for cloud ros publisher.
 */
class CloudRosPublisher : public AbstractSender<Cloud> {
  using PointCloudT = sensor_msgs::PointCloud2;

 public:
  CloudRosPublisher(const ros::NodeHandle& node_handle,
                    const ProjectionParams& params,
                    const std::string& topic_clouds);
  virtual ~CloudRosPublisher() {}
  /**
   * @brief      Get point cloud from ROS
   *
   * @param[in]  msg_cloud  The message cloud
   */
  void CallbackVelodyne(const PointCloudT::ConstPtr& msg_cloud);
  /**
   * @brief      Starts listening to ros.
   */
  void StartListeningToRos();
  void PrintMsgStats(const sensor_msgs::PointCloud2ConstPtr& msg);

 protected:
  Cloud::Ptr RosCloudToCloud(const PointCloudT::ConstPtr& msg);

  ros::NodeHandle _node_handle;
  ros::Subscriber _subscriber_clouds;
  image_transport::Publisher _depth_pub;
  image_transport::ImageTransport _it;
  std::string _topic_clouds;
  std::string _topic_odom;

  ProjectionParams _params;
};

}  // namespace depth_clustering

#endif  // SRC_ROS_BRIDGE_CLOUD_ROS_PUBLISHER_H_