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
  std::vector<T> list;

  str = trim(str);
  if (str[0] != '[' || str[str.size() - 1] != ']') {
    std::cerr << "Parser Error: " << str << " is not a valid list token!" << std::endl;
    return list;
  }

  auto entry_tokens = split(str.substr(1, str.size() - 2), ",");

  for (const auto& token : entry_tokens) {
    uint32_t label = boost::lexical_cast<T>(trim(token));
    list.push_back(label);
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

    if (tokens[0] == "max scans") config.maxNumScans = boost::lexical_cast<uint32_t>(trim(tokens[1]));
    if (tokens[0] == "max range") config.maxRange = boost::lexical_cast<float>(trim(tokens[1]));
    if (tokens[0] == "min range") config.minRange = boost::lexical_cast<float>(trim(tokens[1]));
    if (tokens[0] == "prior scans") config.priorScans = boost::lexical_cast<uint32_t>(trim(tokens[1]));
    if (tokens[0] == "past scans") config.pastScans = boost::lexical_cast<uint32_t>(trim(tokens[1]));
    if (tokens[0] == "past distance") config.pastScans = boost::lexical_cast<float>(trim(tokens[1]));

    if (tokens[0] == "stride num") config.stride_num = boost::lexical_cast<uint32_t>(trim(tokens[1]));
    if (tokens[0] == "stride distance") config.stride_distance = boost::lexical_cast<float>(trim(tokens[1]));

    if (tokens[0] == "min extent") {
      auto coords = parseList<int32_t>(tokens[1]); // "auto" is of type "vector<int32_t>"
      config.minExtent = Eigen::Vector4f(coords[0], coords[1], coords[2], 1);  
      std::cout << "minExtent: " << coords[0] << "," << coords[1] << "," << coords[2] << std::endl;
    }

    if (tokens[0] == "max extent") {
      auto coords = parseList<int32_t>(tokens[1]); // "auto" is of type "vector<int32_t>"
      config.maxExtent = Eigen::Vector4f(coords[0], coords[1], coords[2], 1);  
      std::cout << "maxExtent: " << coords[0] << "," << coords[1] << "," << coords[2] << std::endl;
    }

    if (tokens[0] == "voxel size") {
      config.voxelSize = boost::lexical_cast<float>(trim(tokens[1]));
      std::cout << "voxelSize = " << config.voxelSize << std::endl;
    }

    if (tokens[0] == "ignore") {
      config.filteredLabels = parseList<uint32_t>(tokens[1]);
    }

    if (tokens[0] == "join") {
      auto join_tokens = parseList<std::string>(tokens[1]);

      for (const auto& token : join_tokens) {
        auto mapping = parseDictionary(token);
        uint32_t label = boost::lexical_cast<uint32_t>(trim(mapping[0]));
        config.joinedLabels[label] = parseList<uint32_t>(mapping[1]);
      }
    }
  }

  in.close();

  return config;
}

void fillVoxelGridMat(int32_t*& inVoxels, 
                      VoxelGrid& grid) {

  uint32_t Nx = grid.size(0);
  uint32_t Ny = grid.size(1);
  uint32_t Nz = grid.size(2);

  size_t numElements = grid.num_elements();

  int32_t counter = 0;
  for (uint32_t x = 0; x < Nx; ++x) {
    for (uint32_t y = 0; y < Ny; ++y) {
      for (uint32_t z = 0; z < Nz; ++z) {
        grid.insertMat(x,y,z, inVoxels[counter]);
        counter = counter + 1;
      }
    }
  }                      

}

