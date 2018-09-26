#include "voxelize_utils.h"

#include <matio.h>

#include <rv/string_utils.h>
#include <boost/lexical_cast.hpp>
#include <fstream>

using namespace rv;

std::vector<std::string> parseDictionary(std::string str) {
  std::vector<std::string> tokens;
  str = trim(str);
  if (str[0] != '{' || str[str.size() - 1] != '}') {
    std::cerr << "Parser Error: " << str << " is not a valid dictionary token!" << std::endl;
    return tokens;
  }

  tokens = split(str.substr(1, str.size() - 2), ":");

  return tokens;
}

template <class T>
std::vector<T> parseList(std::string str) {
  str = trim(str);
  std::vector<T> list;

  str = trim(str);
  if (str[0] != '[' || str[str.size() - 1] != ']') {
    std::cerr << "Parser Error: " << str << " is not a valid list token!" << std::endl;
    return list;
  }

  auto entry_tokens = split(str.substr(1, str.size() - 2), ",");

  for (const auto& token : entry_tokens) {
    T value = boost::lexical_cast<T>(trim(token));
    list.push_back(value);
  }

  return list;
}

template <>
std::vector<std::string> parseList(std::string str) {
  str = trim(str);
  if (str[0] != '[' || str[str.size() - 1] != ']') {
    std::cerr << "Parser Error: " << str << " is not a valid list token!" << std::endl;
  }

  std::vector<std::string> list;
  auto entry_tokens = split(str.substr(1, str.size() - 2), ",");

  for (const auto& token : entry_tokens) list.push_back(token);

  return list;
}

Config parseConfiguration(const std::string& filename) {
  Config config;
  std::ifstream in(filename);

  if (!in.is_open()) return config;

  std::string line;
  in.peek();
  while (in.good() && !in.eof()) {
    std::getline(in, line);

    if (trim(line)[0] == '#') continue;  // ignore comments.

    auto tokens = split(line, ":");
    if (tokens.size() < 2) continue;
    if (tokens.size() > 2) {
      for (uint32_t i = 2; i < tokens.size(); ++i) {
        tokens[1] += ":" + tokens[i];
      }
      tokens.resize(2);
    }

    if (tokens[0] == "max scans") {
      config.maxNumScans = boost::lexical_cast<uint32_t>(trim(tokens[1]));
      continue;
    }
    if (tokens[0] == "max range") {
      config.maxRange = boost::lexical_cast<float>(trim(tokens[1]));
      continue;
    }
    if (tokens[0] == "voxel size") {
      config.voxelSize = boost::lexical_cast<float>(trim(tokens[1]));
      continue;
    }
    if (tokens[0] == "min range") {
      config.minRange = boost::lexical_cast<float>(trim(tokens[1]));
      continue;
    }
    if (tokens[0] == "prior scans") {
      config.priorScans = boost::lexical_cast<uint32_t>(trim(tokens[1]));
      continue;
    }
    if (tokens[0] == "past scans") {
      config.pastScans = boost::lexical_cast<uint32_t>(trim(tokens[1]));
      continue;
    }
    if (tokens[0] == "past distance") {
      config.pastScans = boost::lexical_cast<float>(trim(tokens[1]));
      continue;
    }

    if (tokens[0] == "stride num") {
      config.stride_num = boost::lexical_cast<uint32_t>(trim(tokens[1]));
      continue;
    }
    if (tokens[0] == "stride distance") {
      config.stride_distance = boost::lexical_cast<float>(trim(tokens[1]));
      continue;
    }

    if (tokens[0] == "min extent") {
      auto coords = parseList<float>(tokens[1]);
      config.minExtent = Eigen::Vector4f(coords[0], coords[1], coords[2], 1.0f);
      continue;
    }

    if (tokens[0] == "max extent") {
      auto coords = parseList<float>(tokens[1]);
      config.maxExtent = Eigen::Vector4f(coords[0], coords[1], coords[2], 1.0f);

      continue;
    }

    if (tokens[0] == "ignore") {
      config.filteredLabels = parseList<uint32_t>(tokens[1]);

      continue;
    }

    if (tokens[0] == "join") {
      auto join_tokens = parseList<std::string>(tokens[1]);

      for (const auto& token : join_tokens) {
        auto mapping = parseDictionary(token);
        uint32_t label = boost::lexical_cast<uint32_t>(trim(mapping[0]));
        config.joinedLabels[label] = parseList<uint32_t>(mapping[1]);
      }

      continue;
    }

    std::cout << "unknown parameter: " << tokens[0] << std::endl;
  }

  in.close();

  return config;
}

void fillVoxelGrid(const Eigen::Matrix4f& anchor_pose, const std::vector<PointcloudPtr>& points,
                   const std::vector<LabelsPtr>& labels, VoxelGrid& grid, const Config& config) {
  std::map<uint32_t, uint32_t> mappedLabels;  // replace key with value.
  for (auto joins : config.joinedLabels) {
    for (auto label : joins.second) {
      mappedLabels[label] = joins.first;
    }
  }
  const auto configminRange = config.minRange;
  const auto configmaxRange = config.maxRange;
  for (uint32_t t = 0; t < points.size(); ++t) {
    const auto points_t = points[t];
    const Eigen::Matrix4f& pose = points_t->pose;
    //std::cout<<pose<<std::endl;
    const uint32_t points_t_size=points_t->points.size();
    int outpts=0;
    Eigen::Matrix4f ap = anchor_pose.inverse() * pose;
    const auto labels_t=(*labels[t]);
    for (uint32_t i = 0; i < points_t_size; ++i) {
      const Point3f& pp = points_t->points[i];
      const auto ppx = pp.x;
      const auto ppy = pp.y;
      const auto ppz = pp.z;
      float range = Eigen::Vector3f(ppx, ppy, ppz).norm();
      if (range < configminRange || range > configmaxRange) continue;
      bool is_car_point = (config.hidecar && ppx < 3.0 && ppx > -2.0 && std::abs(ppy) < 2.0);
      if (is_car_point) continue;

      Eigen::Vector4f p = ap * Eigen::Vector4f(ppx, ppy, ppz, 1);

      uint32_t label = labels_t[i];
      if (mappedLabels.find(label) != mappedLabels.end()) label = mappedLabels[label];

      if (std::find(config.filteredLabels.begin(), config.filteredLabels.end(), label) == config.filteredLabels.end()) {
        grid.insert(p, labels_t[i]);
        //std::cout<<"."<<std::flush;
        outpts++;
      }
    }
    //std::cout<<outpts<<"/"<<points_t->points.size()<<std::endl;

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
