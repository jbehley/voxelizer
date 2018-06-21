#ifndef SRC_DATA_VOXELGRID_H_
#define SRC_DATA_VOXELGRID_H_

#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>

/** \brief simple Voxelgrid data structure with a label historgram per voxel.
 *
 *  Instead of averaging of points per voxel, we simply record the label of the point.
 *
 *
 *  \author behley
 */

class VoxelGrid {
 public:
  class Voxel {
   public:
    std::map<uint32_t, uint32_t> labels;
    uint32_t count{0};
  };

  void initialize(float resolution, const Eigen::Vector4f& min, const Eigen::Vector4f& max) {
    resolution_ = resolution;
    sizex_ = std::ceil((max.x() - min.x()) / resolution_);
    sizey_ = std::ceil((max.y() - min.y()) / resolution_);
    sizez_ = std::ceil((max.z() - min.z()) / resolution_);

    voxels_.resize(sizex_ * sizey_ * sizez_);
    // ensure that min, max are always inside the voxel grid.
    float ox = min.x() - 0.5 * (sizex_ * resolution - (max.x() - min.x()));
    float oy = min.y() - 0.5 * (sizey_ * resolution - (max.y() - min.y()));
    float oz = min.z() - 0.5 * (sizez_ * resolution - (max.z() - min.z()));
    offset_ = Eigen::Vector4f(ox, oy, oz, 1);

    //    center_.head(3) = 0.5 * (max - min) + min;
    //    center_[3] = 0;
  }

  void clear() {
    for (auto idx : occupied_) {
      voxels_[idx].count = 0;
      voxels_[idx].labels.clear();
    }
  }

  void insert(const Eigen::Vector4f& p, uint32_t label) {
    Eigen::Vector4f tp = p - offset_;
    uint32_t i = std::floor(tp.x() / resolution_);
    uint32_t j = std::floor(tp.y() / resolution_);
    uint32_t k = std::floor(tp.z() / resolution_);

    if ((i >= sizex_) || (j >= sizey_) || (k >= sizez_)) return;

    int32_t gidx = i + j * sizex_ + k * sizex_ * sizey_;
    if (gidx < 0) return;

    occupied_.push_back(gidx);

//    float n = voxels_[gidx].count;
    voxels_[gidx].labels[label] += 1;  //(1. / (n + 1)) * (n * voxels_[gidx].point + p);
    voxels_[gidx].count += 1;
  }

  const std::vector<Voxel>& voxels() const { return voxels_; }

  const Eigen::Vector4f& offset() const { return offset_; }

 protected:
  float resolution_;

  uint32_t sizex_, sizey_, sizez_;
  std::vector<Voxel> voxels_;
  std::vector<uint32_t> occupied_;

  Eigen::Vector4f offset_;
};

#endif /* SRC_DATA_VOXELGRID_H_ */
