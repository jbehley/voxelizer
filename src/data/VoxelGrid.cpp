#include <data/VoxelGrid.h>


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
  }

void VoxelGrid::insert(const Eigen::Vector4f& p, uint32_t label) {
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

int32_t VoxelGrid::occludedBy(const Voxel& start) const
{

  return -1;
  //
  //
  //  /** this voxel will get an occlusion value, so store this in the occluded voxels list **/
  //  foo.push_back(start);
  //
  //  float NextCrossingT[3], DeltaT[3]; /** t for next intersection with voxel boundary of axis, t increment for axis
  //  **/
  //  int Step[3], Out[3], Pos[3]; /** voxel increment for axis, index of of outside voxels, current position **/
  //  float dir[3]; /** ray direction **/
  //
  //  Pos[0] = start->getICoord();
  //  Pos[1] = start->getJCoord();
  //  Pos[2] = start->getKCoord();
  //
  //  /** calculate direction vector assuming sensor at (0,0,_heightOffset) **/
  //  RoSe::Vector startpoint = voxelToPos(Pos[0], Pos[1], Pos[2]);
  //
  //  double voxelLength2 = 0.5 * voxelLength;
  //
  //  startpoint[0] += voxelLength2;
  //  startpoint[1] += voxelLength2;
  //  startpoint[2] += voxelLength2;
  //
  //  /** a little hack, we are not following the ray exact to the sensor, but to a voxel near to the sensor **/
  //  dir[0] = voxelLength2 - startpoint[1];
  //  dir[1] = voxelLength2 - startpoint[0];
  //  dir[2] = _heightOffset + voxelLength2 - startpoint[2];
  //  /** initialize variables for traversal **/
  //  for (unsigned int axis = 0; axis < 3; ++axis)
  //  {
  //    if (dir[axis] < 0)
  //    {
  //      NextCrossingT[axis] = -0.5 * voxelLength / dir[axis];
  //      DeltaT[axis] = -voxelLength / dir[axis];
  //      Step[axis] = -1;
  //      Out[axis] = 0;
  //    }
  //    else
  //    {
  //      NextCrossingT[axis] = 0.5 * voxelLength / dir[axis];
  //      DeltaT[axis] = voxelLength / dir[axis];
  //      Step[axis] = 1;
  //      Out[axis] = nVoxels[axis];
  //    }
  //  }
  //
  //  int i_end, j_end, k_end;
  //  RoSe::Vector3 endpoint(voxelLength2, voxelLength2, _heightOffset + voxelLength2);
  //  posToVoxel(endpoint, &i_end, &j_end, &k_end);
  //
  //  const int cmpToAxis[8] = { 2, 1, 2, 1, 2, 2, 0, 0 };
  //  int iteration = 0;
  //  for (;;) // loop infinitely...
  //  {
  //    const Voxel* voxel = &voxels[offset(Pos[0], Pos[1], Pos[2])];
  //    /* if the current position is occupied, return the obstacle id. */
  //    if (voxel->mIsOccupied && voxel->getKCoord() > 0)
  //    {
  //      /** FIXME: this is very bad code: casting away the constness. **/
  //      Voxel* bla = const_cast<Voxel*> (voxel);
  //      start->mOccludedBy = bar.markVoxelAsObject(bla);
  //      start->mIsOccluded = true;
  //
  //      return voxel->mObstacleIndex;
  //    }
  ////    else if (voxel->mIsOccupied) /** occlusion by ground voxels **/
  ////    {
  ////      start->mOccludedBy = -1;
  ////      return -1;
  ////    }
  //
  //    int bits = ((NextCrossingT[0] < NextCrossingT[1]) << 2)
  //        + ((NextCrossingT[0] < NextCrossingT[2]) << 1) + ((NextCrossingT[1] < NextCrossingT[2]));
  //    int stepAxis = cmpToAxis[bits]; /* branch-free looping */
  //
  //    Pos[stepAxis] += Step[stepAxis];
  //    NextCrossingT[stepAxis] += DeltaT[stepAxis];
  //
  //    /** note the first condition should never happen, since we want to reach a point inside the grid **/
  //    if (Pos[stepAxis] == Out[stepAxis]) break; //... until outside, and leaving the loop here!
  //    if (Pos[0] == i_end && Pos[1] == j_end && Pos[2] == k_end) break; // .. or the sensor origin is reached.
  //
  //    ++iteration;
  //  }
  //
  //  start->mOccludedBy = -1;
  //  start->mIsOccluded = false;
  //  return -1;
}
