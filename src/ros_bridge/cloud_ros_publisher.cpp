#include "ros_bridge/cloud_ros_publisher.h"

#include <algorithm>
#include <string>
#include <vector>

namespace depth_clustering {
using ros::NodeHandle;
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

using std::map;
using std::string;
using std::vector;
template <class T>
T BytesTo(const vector<uint8_t>& data, uint32_t start_idx) {
  const size_t kNumberOfBytes = sizeof(T);
  uint8_t byte_array[kNumberOfBytes];
  // forward bit order (it is a HACK. We do not account for bigendianes)
  for (size_t i = 0; i < kNumberOfBytes; ++i) {
    byte_array[i] = data[start_idx + i];
  }
  T result;
  std::copy(reinterpret_cast<const uint8_t*>(&byte_array[0]),
            reinterpret_cast<const uint8_t*>(&byte_array[kNumberOfBytes]),
            reinterpret_cast<uint8_t*>(&result));
  return result;
}

CloudRosPublisher::CloudRosPublisher(const ros::NodeHandle& node_handle,
                                     const ProjectionParams& params,
                                     const std::string& topic_clouds)
    : AbstractSender{SenderType::STREAMER}, _it(node_handle), _params{params} {
  _node_handle = node_handle;
  _topic_clouds = topic_clouds;
}

void CloudRosPublisher::CallbackVelodyne(
    const PointCloudT::ConstPtr& msg_cloud) {
  // PrintMsgStats(msg_cloud);
  Cloud::Ptr cloud_ptr = RosCloudToCloud(msg_cloud);
  cloud_ptr->InitProjection(_params);
  ShareDataWithAllClients(*cloud_ptr);

  cv::Mat _depth_to_pub = cloud_ptr->projection_ptr()->depth_image();
  cv_bridge::CvImage projected_img;
  std_msgs::Header img_header;
  img_header.stamp = msg_cloud->header.stamp;
  img_header.frame_id = "base_link";
  projected_img = cv_bridge::CvImage(
      img_header, sensor_msgs::image_encodings::TYPE_32FC1, _depth_to_pub);
  _depth_pub.publish(projected_img.toImageMsg());
}

void CloudRosPublisher::StartListeningToRos() {
  _subscriber_clouds = _node_handle.subscribe(
      _topic_clouds, 100, &CloudRosPublisher::CallbackVelodyne, this);
  _depth_pub = _it.advertise("/depth_img_from_lidar", 1);
}

Cloud::Ptr CloudRosPublisher::RosCloudToCloud(
    const PointCloudT::ConstPtr& msg) {
  uint32_t x_offset = msg->fields[0].offset;
  uint32_t y_offset = msg->fields[1].offset;
  uint32_t z_offset = msg->fields[2].offset;
  uint32_t ring_offset = msg->fields[4].offset;

  Cloud cloud;
  for (uint32_t point_start_byte = 0, counter = 0;
       point_start_byte < msg->data.size();
       point_start_byte += msg->point_step, ++counter) {
    RichPoint point;
    point.x() = BytesTo<float>(msg->data, point_start_byte + x_offset);
    point.y() = BytesTo<float>(msg->data, point_start_byte + y_offset);
    point.z() = BytesTo<float>(msg->data, point_start_byte + z_offset);
    point.ring() = BytesTo<uint16_t>(msg->data, point_start_byte + ring_offset);
    // point.z *= -1;  // hack
    cloud.push_back(point);
  }

  return make_shared<Cloud>(cloud);
}

}  // namespace depth_clustering