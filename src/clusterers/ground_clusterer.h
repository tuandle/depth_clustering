#ifndef SRC_CLUSTERERS_GROUND_BASED_CLUSTERER_H_
#define SRC_CLUSTERERS_GROUND_BASED_CLUSTERER_H_

#include <opencv/cv.h>
#include <chrono>
#include <ctime>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "utils/cloud.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/useful_typedefs.h"

#include "clusterers/abstract_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"
#include "image_labelers/linear_image_labeler.h"
#include "projections/cloud_projection.h"

namespace depth_clustering {

/**
 * @brief      Class for image based clusterer.
 *
 * @tparam     LabelerT  A Labeler class to be used for labeling.
 */
template <typename LabelerT>
class GroundClusterer : public AbstractClusterer {
 public:
  using Receiver = AbstractClient<Cloud>;
  using Sender = AbstractSender<std::unordered_map<uint16_t, Cloud>>;

  /**
   * @brief      Construct an image-based clusterer.
   *
   * @param[in]  angle_tollerance  The angle tollerance to separate objects
   * @param[in]  min_cluster_size  The minimum cluster size to send
   * @param[in]  max_cluster_size  The maximum cluster size to send
   */
  explicit GroundClusterer(Radians angle_tollerance = 8_deg,
                           uint16_t min_cluster_size = 100,
                           uint16_t max_cluster_size = 25000)
      : AbstractClusterer(0.0, min_cluster_size, max_cluster_size),
        _counter(0),
        _angle_tollerance(angle_tollerance),
        _label_client{nullptr} {}

  virtual ~GroundClusterer() {}

  /**
   * @brief      Sets the label image client.
   *
   * @param      client  The client to receive color images with labels
   */
  void SetLabelImageClient(AbstractClient<cv::Mat>* client) {
    this->_label_client = client;
  }

  /**
   * @brief      Gets called when clusterer receives a cloud to cluster ground
   *
   * @param[in]  cloud      The cloud to cluster
   * @param[in]  sender_id  The sender identifier
   */
  void OnNewObjectReceived(const Cloud& cloud, const int sender_id) override {
    // generate a projection from a point cloud
    if (!cloud.projection_ptr()) {
      fprintf(stderr, "ERROR: projection not initialized in cloud.\n");
      fprintf(stderr, "INFO: cannot label this cloud.\n");
      return;
    }
    time_utils::Timer timer;
    // create 3d clusters from image labels
    std::unordered_map<uint16_t, Cloud> clusters;
    // assign an arbitrary ground label
    uint16_t label = 2;
    const cv::Mat& img_ground = cloud.projection_ptr()->depth_image();
    for (int row = 0; row < img_ground.rows; ++row) {
      for (int col = 0; col < img_ground.cols; ++col) {
        const auto& point_container = cloud.projection_ptr()->at(row, col);
        if (point_container.IsEmpty()) {
          // this is ok, just continue, nothing interesting here, no points.
          continue;
        }
        if (img_ground.at<float>(row, col) == 0) {
          // skip non-ground points
          continue;
        }
        for (const auto& point_idx : point_container.points()) {
          const auto& point = cloud.points()[point_idx];
          clusters[label].push_back(point);
        }
      }
    }

    fprintf(stderr, "INFO: prepared ground clusters in: %lu us\n",
            timer.measure());

    this->ShareDataWithAllClients(clusters);
    fprintf(stderr, "INFO: ground clusters shared: %lu us\n", timer.measure());
  }

 private:
  int _counter;
  Radians _angle_tollerance;

  AbstractClient<cv::Mat>* _label_client;
};

}  // namespace depth_clustering

#endif  // SRC_CLUSTERERS_GROUND_BASED_CLUSTERER_H_
