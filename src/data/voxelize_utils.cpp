#include "voxelize_utils.h"

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

  for (uint32_t i = 0; i < entry_tokens.size(); ++i) {
    std::string token = entry_tokens[i];

    if (token.find("[") != std::string::npos) {
      if (token.find("]", token.find("[")) == std::string::npos) {
        // found a nested unterminated list token
        std::string next_token;
        do {
          if (i >= entry_tokens.size()) break;
          next_token = entry_tokens[i + 1];
          token += "," + next_token;
          ++i;
        } while (next_token.find("]") == std::string::npos);
      }
    }
    list.push_back(token);
  }

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
      auto join_tokens = parseList<std::string>(trim(tokens[1]));

      for (const auto& token : join_tokens) {
        auto mapping = parseDictionary(token);
        uint32_t label = boost::lexical_cast<uint32_t>(trim(mapping[0]));
        config.joinedLabels[label] = parseList<uint32_t>(trim(mapping[1]));
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

  for (uint32_t t = 0; t < points.size(); ++t) {
    Eigen::Matrix4f ap = anchor_pose.inverse() * points[t]->pose;

    for (uint32_t i = 0; i < points[t]->points.size(); ++i) {
      const Point3f& pp = points[t]->points[i];

      float range = Eigen::Vector3f(pp.x, pp.y, pp.z).norm();
      if (range < config.minRange || range > config.maxRange) continue;
      bool is_car_point = (config.hidecar && pp.x < 3.0 && pp.x > -2.0 && std::abs(pp.y) < 2.0);
      if (is_car_point) continue;

      Eigen::Vector4f p = ap * Eigen::Vector4f(pp.x, pp.y, pp.z, 1);

      uint32_t label = (*labels[t])[i];
      if (mappedLabels.find(label) != mappedLabels.end()) label = mappedLabels[label];

      if (std::find(config.filteredLabels.begin(), config.filteredLabels.end(), label) == config.filteredLabels.end()) {
        grid.insert(p, (*labels[t])[i]);
      }
    }
  }
}

template <typename T>
std::vector<uint8_t> pack(const std::vector<T>& vec) {
  std::vector<uint8_t> packed(vec.size() / 8);

  for (uint32_t i = 0; i < vec.size(); i += 8) {
    packed[i / 8] = (vec[i] > 0) << 7 | (vec[i + 1] > 0) << 6 | (vec[i + 2] > 0) << 5 | (vec[i + 3] > 0) << 4 |
                    (vec[i + 4] > 0) << 3 | (vec[i + 5] > 0) << 2 | (vec[i + 6] > 0) << 1 | (vec[i + 7] > 0);
    ;
  }

  return packed;
}

void saveVoxelGrid(const VoxelGrid& grid, const std::string& directory, const std::string& basename,
                   const std::string& mode) {
  uint32_t Nx = grid.size(0);
  uint32_t Ny = grid.size(1);
  uint32_t Nz = grid.size(2);

  size_t numElements = grid.num_elements();
  std::vector<uint16_t> outputLabels(numElements, 0);
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
        outputLabels[counter] = maxLabel;
        outputTensorOccluded[counter] = isOccluded;
        outputTensorInvalid[counter] = (uint32_t)grid.isInvalid(x, y, z);
      }
    }
  }

  if (mode == "target") {
    // for target we just generate label, invalid, occluded.
    {
      std::string output_filename = directory + "/" + basename + ".label";

      std::ofstream out(output_filename.c_str());
      out.write((const char*)&outputLabels[0], outputLabels.size() * sizeof(uint16_t));
      out.close();
    }

    {
      std::string output_filename = directory + "/" + basename + ".occluded";

      std::ofstream out(output_filename.c_str());
      std::vector<uint8_t> packed = pack(outputTensorOccluded);
      out.write((const char*)&packed[0], packed.size() * sizeof(uint8_t));
      out.close();
    }

    {
      std::string output_filename = directory + "/" + basename + ".invalid";

      std::ofstream out(output_filename.c_str());
      std::vector<uint8_t> packed = pack(outputTensorInvalid);
      out.write((const char*)&packed[0], packed.size() * sizeof(uint8_t));
      out.close();
    }

  } else {
    // for input we just generate the ".bin" file.
    {
      std::string output_filename = directory + "/" + basename + ".bin";

      std::ofstream out(output_filename.c_str());
      std::vector<uint8_t> packed = pack(outputLabels);
      out.write((const char*)&packed[0], packed.size() * sizeof(uint8_t));
      out.close();
    }
  }
}
