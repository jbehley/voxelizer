#include <stdint.h>
#include <widget/KittiReader.h>
#include <QtCore/QDir>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include "rv/string_utils.h"

void KittiReader::initialize(const QString& sequence_dir, const QString& input_label_dir) {
  velodyne_filenames_.clear();

  QDir base_dir(sequence_dir);
  QDir velodyne_dir(base_dir.filePath("velodyne"));
  if (!velodyne_dir.exists()) throw std::runtime_error("Missing velodyne files.");
  QStringList entries = velodyne_dir.entryList(QDir::Files, QDir::Name);
  for (int32_t i = 0; i < entries.size(); ++i) {
    velodyne_filenames_.push_back(velodyne_dir.filePath(entries.at(i)).toStdString());
  }

  if (!base_dir.exists("calib.txt"))
    throw std::runtime_error("Missing calibration file: " + base_dir.filePath("calib.txt").toStdString());

  calib_.initialize(base_dir.filePath("calib.txt").toStdString());
  readPoses(base_dir.filePath("poses.txt").toStdString(), poses_);

  for (uint32_t i = 0; i < velodyne_filenames_.size(); ++i) {
    std::ifstream in(velodyne_filenames_[i].c_str());
    in.seekg(0, std::ios::end);
    uint32_t num_points = in.tellg() / (4 * sizeof(float));
    in.close();

    QString filename = QFileInfo(QString::fromStdString(velodyne_filenames_[i])).baseName() + ".label";
    if (!input_label_dir.exists(filename)) {
      std::ofstream out(input_label_dir.filePath(filename).toStdString().c_str());
      std::vector<uint32_t> labels(num_points, 0);
      out.write(reinterpret_cast<const char*>(labels.data()), num_points * sizeof(uint32_t));
      out.close();
    }
    label_filenames_.push_back(input_label_dir.filePath(filename).toStdString());
  }
}

void KittiReader::retrieve(int32_t idx, std::vector<PointcloudPtr>& priorPoints, std::vector<LabelsPtr>& priorLabels,
                           std::vector<PointcloudPtr>& pastPoints, std::vector<LabelsPtr>& pastLabels) {
  priorPoints.clear();
  priorLabels.clear();
  pastPoints.clear();
  pastLabels.clear();

  if (idx >= int32_t(velodyne_filenames_.size()) || idx < 0) return;

  std::vector<int32_t> indexesBefore;
  for (auto it = pointsCache_.begin(); it != pointsCache_.end(); ++it) indexesBefore.push_back(it->first);
  std::vector<int32_t> indexesAfter;

  uint32_t scansRead = 0;

  for (int32_t t = std::max<int32_t>(0, idx - numPriorScans_); t <= idx; ++t) {
    indexesAfter.push_back(t);
    if (pointsCache_.find(t) == pointsCache_.end()) {
      scansRead += 1;

      priorPoints.push_back(std::shared_ptr<Laserscan>(new Laserscan));
      readPoints(velodyne_filenames_[t], *priorPoints.back());
      pointsCache_[t] = priorPoints.back();
      priorPoints.back()->pose = poses_[t];

      priorLabels.push_back(std::shared_ptr<std::vector<uint32_t>>(new std::vector<uint32_t>()));
      readLabels(label_filenames_[t], *priorLabels.back());
      labelCache_[t] = priorLabels.back();

      if (priorPoints.back()->size() != priorLabels.back()->size()) {
        std::cout << "Filename: " << velodyne_filenames_[t] << std::endl;
        std::cout << "Filename: " << label_filenames_[t] << std::endl;
        std::cout << "num. points = " << priorPoints.back()->size()
                  << " vs. num. labels = " << priorLabels.back()->size() << std::endl;
        throw std::runtime_error("Inconsistent number of labels.");
      }

    } else {
      priorPoints.push_back(pointsCache_[t]);
      priorLabels.push_back(labelCache_[t]);
    }
  }

  for (int32_t t = int32_t(std::min<int32_t>(velodyne_filenames_.size() - 1, idx + 1));
       t < int32_t(std::min<int32_t>(velodyne_filenames_.size(), idx + numPastScans_ + 1)); ++t) {
    indexesAfter.push_back(t);
    if (pointsCache_.find(t) == pointsCache_.end()) {
      scansRead += 1;

      pastPoints.push_back(std::shared_ptr<Laserscan>(new Laserscan));
      readPoints(velodyne_filenames_[t], *pastPoints.back());
      pointsCache_[t] = pastPoints.back();
      pastPoints.back()->pose = poses_[t];

      pastLabels.push_back(std::shared_ptr<std::vector<uint32_t>>(new std::vector<uint32_t>()));
      readLabels(label_filenames_[t], *pastLabels.back());
      labelCache_[t] = pastLabels.back();

      if (pastPoints.back()->size() != pastLabels.back()->size()) {
        std::cout << "Filename: " << velodyne_filenames_[t] << std::endl;
        std::cout << "Filename: " << label_filenames_[t] << std::endl;
        std::cout << "num. points = " << pastPoints.back()->size() << " vs. num. labels = " << pastLabels.back()->size()
                  << std::endl;
        throw std::runtime_error("Inconsistent number of labels.");
      }

    } else {
      pastPoints.push_back(pointsCache_[t]);
      pastLabels.push_back(labelCache_[t]);
    }
  }

  std::cout << scansRead << " point clouds read." << std::endl;

  // FIXME: keep more scans in cache. not only remove unloaded scans.

  std::sort(indexesBefore.begin(), indexesBefore.end());
  std::sort(indexesAfter.begin(), indexesAfter.end());

  std::vector<int32_t> needsDelete(indexesBefore.size());
  std::vector<int32_t>::iterator end = std::set_difference(
      indexesBefore.begin(), indexesBefore.end(), indexesAfter.begin(), indexesAfter.end(), needsDelete.begin());

  for (auto it = needsDelete.begin(); it != end; ++it) {
    pointsCache_.erase(*it);
    labelCache_.erase(*it);
  }
}

void KittiReader::readPoints(const std::string& filename, Laserscan& scan) {
  std::ifstream in(filename.c_str(), std::ios::binary);
  if (!in.is_open()) return;

  scan.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (4 * sizeof(float));
  in.seekg(0, std::ios::beg);

  std::vector<float> values(4 * num_points);
  in.read((char*)&values[0], 4 * num_points * sizeof(float));

  in.close();
  std::vector<Point3f>& points = scan.points;
  std::vector<float>& remissions = scan.remissions;

  points.resize(num_points);
  remissions.resize(num_points);

  for (uint32_t i = 0; i < num_points; ++i) {
    points[i].x = values[4 * i];
    points[i].y = values[4 * i + 1];
    points[i].z = values[4 * i + 2];
    remissions[i] = values[4 * i + 3];
  }
}

void KittiReader::readLabels(const std::string& filename, std::vector<uint32_t>& labels) {
  std::ifstream in(filename.c_str(), std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "Unable to open label file. " << std::endl;
    return;
  }

  labels.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (sizeof(uint32_t));
  in.seekg(0, std::ios::beg);

  labels.resize(num_points);
  in.read((char*)&labels[0], num_points * sizeof(uint32_t));

  for(uint32_t i = 0; i < labels.size(); ++i)
  {
    labels[i] = labels[i] & 0xFFFF; // extract label from (instance + label)
  }

  in.close();
}

void KittiReader::readPoses(const std::string& filename, std::vector<Eigen::Matrix4f>& poses) {
  poses = KITTI::Odometry::loadPoses(filename);

  // convert from camera to velodyne coordinate system.
  Eigen::Matrix4f Tr = calib_["Tr"];
  Eigen::Matrix4f Tr_inv = Tr.inverse();
  for (uint32_t i = 0; i < poses.size(); ++i) {
    poses[i] = Tr_inv * poses[i] * Tr;
  }
}
