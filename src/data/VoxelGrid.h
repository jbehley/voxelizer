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

  /** \brief set parameters of voxel grid and compute internal offsets,  etc. **/
  void initialize(float resolution, const Eigen::Vector4f& min, const Eigen::Vector4f& max);

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

  /** \brief add label for specific point to voxel grid **/
  void insert(const Eigen::Vector4f& p, uint32_t label);
  /** \brief get  all voxels. **/
  const std::vector<Voxel>& voxels() const { return voxels_; }

  const Eigen::Vector4f& offset() const { return offset_; }

  /** \brief get size in specific dimension **/
  uint32_t size(uint32_t dim) const { return (&sizex_)[std::max<uint32_t>(std::min<uint32_t>(dim, 3), 0)]; }

  /** \brief resolutions aka sidelength of a voxel **/
  float resolution() const { return resolution_; }

  /** \brief check for each voxel if  there is voxel occluding the voxel. **/
  void updateOcclusions();



 protected:
  /** \brief check if given voxel is occluded.
   *
   *  \return index of voxel that occludes given voxel, -1  if voxel is not occluded.
   **/
  int32_t occludedBy(const Voxel& start) const;

  float resolution_;

  uint32_t sizex_, sizey_, sizez_;
  std::vector<Voxel> voxels_;
  std::vector<uint32_t> occupied_;

  Eigen::Vector4f offset_;
};

#endif /* SRC_DATA_VOXELGRID_H_ */
