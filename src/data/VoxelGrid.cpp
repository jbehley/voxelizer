#include <data/VoxelGrid.h>

#include <iostream>

void VoxelGrid::initialize(float resolution, const Eigen::Vector4f& min, const Eigen::Vector4f& max) {
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

  occlusions_.resize(sizex_ * sizey_ * sizez_);
}

void VoxelGrid::clear() {
  for (auto idx : occupied_) {
    voxels_[idx].count = 0;
    voxels_[idx].labels.clear();
  }

  for (auto idx : occluded_) {
    voxels_[idx].count = 0;
    voxels_[idx].labels.clear();
  }

  occupied_.clear();
  occluded_.clear();
}

void VoxelGrid::insert(const Eigen::Vector4f& p, uint32_t label) {
  Eigen::Vector4f tp = p - offset_;
  int32_t i = std::floor(tp.x() / resolution_);
  int32_t j = std::floor(tp.y() / resolution_);
  int32_t k = std::floor(tp.z() / resolution_);

  if ((i >= int32_t(sizex_)) || (j >= int32_t(sizey_)) || (k >= int32_t(sizez_))) return;
  if ((i < 0) || (j < 0) || (k < 0)) return;

  int32_t gidx = index(i, j, k);
  if (gidx < 0 || gidx >= int32_t(voxels_.size())) return;

  occupied_.push_back(gidx);

  //    float n = voxels_[gidx].count;
  voxels_[gidx].labels[label] += 1;  //(1. / (n + 1)) * (n * voxels_[gidx].point + p);
  voxels_[gidx].count += 1;
  occlusionsValid_ = false;
}

void VoxelGrid::insertMat(uint32_t i, uint32_t j, uint32_t k , uint32_t label) {

  if ((i >= int32_t(sizex_)) || (j >= int32_t(sizey_)) || (k >= int32_t(sizez_))) return;
  if ((i < 0) || (j < 0) || (k < 0)) return;
  int32_t gidx = index(i, j, k);
  if (gidx < 0 || gidx >= int32_t(voxels_.size())) return;

  if (label == 0) return; // TODO: The handling of empty "0" voxels is handled differently in the original voxelizer.cpp

  occupied_.push_back(gidx);
  voxels_[gidx].labels[label] += 1;  //(1. / (n + 1)) * (n * voxels_[gidx].point + p);
  voxels_[gidx].count += 1;
  occlusionsValid_ = false;
}

bool VoxelGrid::isOccluded(int32_t i, int32_t j, int32_t k) const { return occlusions_[index(i, j, k)] > -1; }

bool VoxelGrid::isFree(int32_t i, int32_t j, int32_t k) const { return occlusions_[index(i, j, k)] == -1; }

bool VoxelGrid::isInvalid(int32_t i, int32_t j, int32_t k) const {
  if (int32_t(invalid_.size()) < index(i, j, k)) return true;

  return (invalid_[index(i, j, k)] > -1) && (invalid_[index(i, j, k)] != index(i, j, k));
}

void VoxelGrid::insertOcclusionLabels() {
  if (!occlusionsValid_) updateOcclusions();

  for (uint32_t i = 0; i < sizex_; ++i) {
    for (uint32_t j = 0; j < sizey_; ++j) {
      for (uint32_t k = 0; k < sizez_; ++k) {
        // heuristic: find label from above.
        if (occlusions_[index(i, j, k)] != index(i, j, k)) {
          int32_t n = 1;
          while ((k + n < sizez_) && isOccluded(i, j, k + n) && voxels_[index(i, j, k + n)].count == 0) n += 1;
          if (k + n < sizez_ && voxels_[index(i, j, k + n)].count > 0) {
            voxels_[index(i, j, k)].count = voxels_[index(i, j, k + n)].count;
            voxels_[index(i, j, k)].labels = voxels_[index(i, j, k + n)].labels;
          }
        }
      }
    }
  }
}

void VoxelGrid::updateOcclusions() {
  for (uint32_t i = 0; i < sizex_; ++i) {
    for (uint32_t j = 0; j < sizey_; ++j) {
      for (uint32_t k = 0; k < sizez_; ++k) {
        occlusions_[index(i, j, k)] = occludedBy(i, j, k);
        if (occlusions_[index(i, j, k)] != index(i, j, k)) occluded_.push_back(index(i, j, k));
      }
    }
  }

  invalid_ = occlusions_;
  occlusionsValid_ = true;
}

void VoxelGrid::updateInvalid(const Eigen::Vector3f& position) {
  if (!occlusionsValid_) updateOcclusions();

  for (uint32_t x = 0; x < sizex_; ++x) {
    for (uint32_t y = 0; y < sizey_; ++y) {
      for (uint32_t z = 0; z < sizez_; ++z) {
        int32_t idx = index(x, y, z);
        // idea: if voxel is not occluded, the value should be -1.
        invalid_[idx] = std::min<int32_t>(invalid_[idx], occludedBy(x, y, z, position));
      }
    }
  }
}

int32_t VoxelGrid::occludedBy(int32_t i, int32_t j, int32_t k, const Eigen::Vector3f& endpoint,
                              std::vector<Eigen::Vector3i>* visited) const {
  float NextCrossingT[3], DeltaT[3]; /** t for next intersection with voxel boundary of axis, t increment for axis
  **/
  int32_t Step[3], Out[3], Pos[3];   /** voxel increment for axis, index of of outside voxels, current position **/
  float dir[3];                      /** ray direction **/

  if (visited != nullptr) visited->clear();

  Pos[0] = i;
  Pos[1] = j;
  Pos[2] = k;

  /** calculate direction vector assuming sensor at (0,0,_heightOffset) **/
  Eigen::Vector3f startpoint = voxel2position(Pos[0], Pos[1], Pos[2]);

  double halfResolution = 0.5 * resolution_;

  dir[0] = endpoint[0] - startpoint[0];
  dir[1] = endpoint[1] - startpoint[1];
  dir[2] = endpoint[2] - startpoint[2];

  /** initialize variables for traversal **/
  for (uint32_t axis = 0; axis < 3; ++axis) {
    if (dir[axis] < 0) {
      NextCrossingT[axis] = -halfResolution / dir[axis];
      DeltaT[axis] = -resolution_ / dir[axis];
      Step[axis] = -1;
      Out[axis] = 0;
    } else {
      NextCrossingT[axis] = halfResolution / dir[axis];
      DeltaT[axis] = resolution_ / dir[axis];
      Step[axis] = 1;
      Out[axis] = size(axis);
    }
  }

  Eigen::Vector3i endindexes = position2voxel(endpoint);
  int32_t i_end = endindexes[0];
  int32_t j_end = endindexes[1];
  int32_t k_end = endindexes[2];

  const int32_t cmpToAxis[8] = {2, 1, 2, 1, 2, 2, 0, 0};
  int32_t iteration = 0;
  for (;;)  // loop infinitely...
  {
    if (Pos[0] < 0 || Pos[1] < 0 || Pos[2] < 0) break;
    if (Pos[0] >= int32_t(sizex_) || Pos[1] >= int32_t(sizey_) || Pos[2] >= int32_t(sizez_)) break;

    int32_t idx = index(Pos[0], Pos[1], Pos[2]);
    bool occupied = voxels_[idx].count > 0;
    if (visited != nullptr) visited->push_back(Eigen::Vector3i(Pos[0], Pos[1], Pos[2]));

    if (occupied) return idx;

    int32_t bits = ((NextCrossingT[0] < NextCrossingT[1]) << 2) + ((NextCrossingT[0] < NextCrossingT[2]) << 1) +
                   ((NextCrossingT[1] < NextCrossingT[2]));
    int32_t stepAxis = cmpToAxis[bits]; /* branch-free looping */

    Pos[stepAxis] += Step[stepAxis];
    NextCrossingT[stepAxis] += DeltaT[stepAxis];

    /** note the first condition should never happen, since we want to reach a point inside the grid **/
    if (Pos[stepAxis] == Out[stepAxis]) break;                         //... until outside, and leaving the loop here!
    if (Pos[0] == i_end && Pos[1] == j_end && Pos[2] == k_end) break;  // .. or the sensor origin is reached.

    ++iteration;
  }

  return -1;
}
