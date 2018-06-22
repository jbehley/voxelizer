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
    clear();

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

  Voxel& operator()(uint32_t i, uint32_t j, uint32_t k) { return voxels_[i + j * sizex_ + k * sizex_ * sizey_]; }
  const Voxel& operator()(uint32_t i, uint32_t j, uint32_t k) const {
    return voxels_[i + j * sizex_ + k * sizex_ * sizey_];
  }

  void clear() {
    for (auto idx : occupied_) {
      voxels_[idx].count = 0;
      voxels_[idx].labels.clear();
    }
    occupied_.clear();
  }

  void insert(const Eigen::Vector4f& p, uint32_t label) {
    Eigen::Vector4f tp = p - offset_;
    int32_t i = std::floor(tp.x() / resolution_);
    int32_t j = std::floor(tp.y() / resolution_);
    int32_t k = std::floor(tp.z() / resolution_);

    if ((i >= int32_t(sizex_)) || (j >= int32_t(sizey_)) || (k >= int32_t(sizez_))) return;
    if ((i < 0) || (j < 0) || (k < 0)) return;

    int32_t gidx = i + j * sizex_ + k * sizex_ * sizey_;
    if (gidx < 0 || gidx >= int32_t(voxels_.size())) return;

    occupied_.push_back(gidx);

    //    float n = voxels_[gidx].count;
    voxels_[gidx].labels[label] += 1;  //(1. / (n + 1)) * (n * voxels_[gidx].point + p);
    voxels_[gidx].count += 1;
  }

  const std::vector<Voxel>& voxels() const { return voxels_; }

  const Eigen::Vector4f& offset() const { return offset_; }

  uint32_t size(uint32_t dim) const { return (&sizex_)[std::max<uint32_t>(std::min<uint32_t>(dim, 3), 0)]; }

  float resolution() const { return resolution_; }

 protected:
  float resolution_;

  uint32_t sizex_, sizey_, sizez_;
  std::vector<Voxel> voxels_;
  std::vector<uint32_t> occupied_;

  Eigen::Vector4f offset_;
};

#endif /* SRC_DATA_VOXELGRID_H_ */
