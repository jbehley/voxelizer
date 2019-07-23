#ifndef SRC_WIDGET_KITTIREADER_H_
#define SRC_WIDGET_KITTIREADER_H_

#include <stdint.h>
#include <QtCore/QString>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include "data/common.h"
#include "data/kitti_utils.h"

/** \brief
 *
 *
 *
 *  \author behley
 */

class KittiReader {
 public:
  void initialize(const QString& sequence_dir);

  uint32_t count() const { return velodyne_filenames_.size(); }

  void setNumPriorScans(int32_t count) { numPriorScans_ = count; }
  void setNumPastScans(int32_t count) { numPastScans_ = count; }

  /** \brief get points and labels for given index. **/
  //  void retrieve(uint32_t index, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
  //                std::vector<LabelsPtr>& labels);

  void retrieve(int32_t idx, std::vector<PointcloudPtr>& priorPoints, std::vector<LabelsPtr>& pirorLabels,
                std::vector<PointcloudPtr>& pastPoints, std::vector<LabelsPtr>& pastLabels);

  std::vector<Eigen::Matrix4f> poses() const { return poses_; }

 protected:
  void readPoints(const std::string& filename, Laserscan& scan);
  void readLabels(const std::string& filename, std::vector<uint32_t>& labels);
  void readPoses(const std::string& filename, std::vector<Eigen::Matrix4f>& poses);

  KITTICalibration calib_;
  std::vector<Eigen::Matrix4f> poses_;
  std::vector<std::string> velodyne_filenames_;
  std::vector<std::string> label_filenames_;

  // cache reads from before.
  std::map<uint32_t, PointcloudPtr> pointsCache_;
  std::map<uint32_t, LabelsPtr> labelCache_;

  float maxDistance_{15.0f};
  int32_t numPriorScans_{0};
  int32_t numPastScans_{10};
};

#endif /* SRC_WIDGET_KITTIREADER_H_ */
