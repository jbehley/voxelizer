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

  Voxel& operator()(uint32_t i, uint32_t j, uint32_t k) { return voxels_[index(i, j, k)]; }
  const Voxel& operator()(uint32_t i, uint32_t j, uint32_t k) const { return voxels_[index(i, j, k)]; }

  /** \brief cleanup voxelgrid. **/
  void clear();

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

  /** \brief fill occluded areas with labels. **/
  void insertOcclusionLabels();

  /** \brief get if voxel at (i,j,k) is occluded.
   *  Assumes that updateOcclusions has been called before,
   *
   *  \see updateOcclusions
   **/
  bool isOccluded(int32_t i, int32_t j, int32_t k) const;
  /** \brief get if voxel at (i,j,k) is free.
   *  Assumes that updateOcclusions has been called before,
   *
   *  \see updateOcclusions
   **/
  bool isFree(int32_t i, int32_t j, int32_t k) const;

  /** \brief check if given voxel is occluded.
   *
   *
   *  \return index of voxel that occludes given voxel, -1  if voxel is not occluded.
   **/
  int32_t occludedBy(int32_t i, int32_t j, int32_t k, std::vector<Eigen::Vector3i>* visited = nullptr) const;

 protected:
  /** \brief get position of voxel center. **/
  inline Eigen::Vector3f voxel2position(int32_t i, int32_t j, int32_t k) const {
    return Eigen::Vector3f(offset_[0] + i * resolution_ + 0.5 * resolution_,
                           offset_[1] + j * resolution_ + 0.5 * resolution_,
                           offset_[2] + k * resolution_ + 0.5 * resolution_);
  }

  inline Eigen::Vector3i position2voxel(const Eigen::Vector3f& pos) const {
    return Eigen::Vector3i((pos[0] - offset_[0]) / resolution_, (pos[1] - offset_[1]) / resolution_,
                           (pos[2] - offset_[2]) / resolution_);
  }

  inline int32_t index(int32_t i, int32_t j, int32_t k) const { return i + j * sizex_ + k * sizex_ * sizey_; }

  float resolution_;

  uint32_t sizex_, sizey_, sizez_;
  std::vector<Voxel> voxels_;
  std::vector<uint32_t> occupied_;  // filled by updateOcclusions.

  Eigen::Vector4f offset_;

  std::vector<int32_t> occlusions_;
  bool occlusionsValid_{false};
};

#endif /* SRC_DATA_VOXELGRID_H_ */
