#include "voxelize_utils.h"

void fillVoxelGrid(const Eigen::Matrix4f& anchor_pose, const std::vector<PointcloudPtr>& points,
                   const std::vector<LabelsPtr>& labels, VoxelGrid& grid, const Config& config) {
  for (uint32_t t = 0; t < points.size(); ++t) {
    const Eigen::Matrix4f& pose = points[t]->pose;
    for (uint32_t i = 0; i < points[t]->points.size(); ++i) {
      const Point3f& pp = points[t]->points[i];
      float range = Eigen::Vector3f(pp.x, pp.y, pp.z).norm();
      if (range < config.minRange || range > config.maxRange) continue;
      bool is_car_point = (pp.x < 3.0 && pp.x > -2.0 && std::abs(pp.y) < 2.0);
      if (is_car_point) continue;

      Eigen::Vector4f p = anchor_pose.inverse() * pose * Eigen::Vector4f(pp.x, pp.y, pp.z, 1);
      if (std::find(config.filteredLabels.begin(), config.filteredLabels.end(), (*labels[t])[i]) ==
          config.filteredLabels.end()) {
        grid.insert(p, (*labels[t])[i]);
      }
    }
  }
}

void saveVoxelGrid(const VoxelGrid& grid, const std::string& filename) {
  //  Eigen::Vector4f offset = grid.offset();
  //  float voxelSize = grid.resolution();

  uint32_t Nx = grid.size(0);
  uint32_t Ny = grid.size(1);
  uint32_t Nz = grid.size(2);

  //  std::cout << "Nx = " << Nx << std::endl;
  //  std::cout << "Ny = " << Ny << std::endl;
  //  std::cout << "Nz = " << Nz << std::endl;

  size_t numElements = grid.num_elements();
  std::vector<uint32_t> outputTensor(numElements, 0);
  std::vector<uint32_t> outputTensorOccluded(numElements, 0);
  std::vector<uint32_t> outputTensorInvalid(numElements, 0);

  int32_t counter = 0;
  for (uint32_t x = 0; x < Nx; ++x) {
    for (uint32_t y = 0; y < Ny; ++y) {
      for (uint32_t z = 0; z < Nz; ++z) {
        const VoxelGrid::Voxel& v = grid(x, y, z);

        uint32_t isOccluded = (uint32_t)grid.isOccluded(x, y, z);

        uint32_t maxCount = 0;
        uint32_t maxLabel = 0;

        for (auto it = v.labels.begin(); it != v.labels.end(); ++it) {
          if (it->second > maxCount) {
            maxCount = it->second;
            maxLabel = it->first;
          }
        }

        // Write maxLabel appropriately to file.
        counter = counter + 1;
        outputTensor[counter] = maxLabel;
        outputTensorOccluded[counter] = isOccluded;
        outputTensorInvalid[counter] = (uint32_t)grid.isInvalid(x, y, z);
      }
    }
  }
  //  std::cout << "Counter = " << counter << std::endl;

  // Save 1D-outputTensor as mat file
  mat_t* matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT5);  // or MAT_FT_MAT4 / MAT_FT_MAT73
  size_t dim[1] = {numElements};

  matvar_t* variable_data = Mat_VarCreate("data", MAT_C_INT32, MAT_T_INT32, 1, dim, &outputTensor[0], 0);
  Mat_VarWrite(matfp, variable_data, MAT_COMPRESSION_NONE);  // or MAT_COMPRESSION_ZLIB

  matvar_t* variable_mask = Mat_VarCreate("mask", MAT_C_INT32, MAT_T_INT32, 1, dim, &outputTensorOccluded[0], 0);
  Mat_VarWrite(matfp, variable_mask, MAT_COMPRESSION_NONE);  // or MAT_COMPRESSION_ZLIB

  matvar_t* variable_invalid = Mat_VarCreate("invalid", MAT_C_INT32, MAT_T_INT32, 1, dim, &outputTensorInvalid[0], 0);
  Mat_VarWrite(matfp, variable_invalid, MAT_COMPRESSION_NONE);  // or MAT_COMPRESSION_ZLIB

  Mat_VarFree(variable_data);
  Mat_VarFree(variable_mask);
  Mat_VarFree(variable_invalid);

  Mat_Close(matfp);
  //  std::cout << "Done" << std::endl;
}
