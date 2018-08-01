#ifndef SRC_DATA_VOXELIZE_UTILS_H_
#define SRC_DATA_VOXELIZE_UTILS_H_

#include <matio.h>
#include "Pointcloud.h"
#include "VoxelGrid.h"
#include "common.h"

class Config {
 public:
  float minRange{2.5f};
  float maxRange{25.0f};
  std::vector<uint32_t> filteredLabels;
};

void fillVoxelGrid(const Eigen::Matrix4f& anchor_pose, const std::vector<PointcloudPtr>& points,
                   const std::vector<LabelsPtr>& labels, VoxelGrid& grid, const Config& config);

void saveVoxelGrid(const VoxelGrid& grid, const std::string& filename);

#endif /* SRC_DATA_VOXELIZE_UTILS_H_ */
