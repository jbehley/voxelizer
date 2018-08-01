#include <matio.h>
#include <stdint.h>
#include <fstream>
#include <iostream>

#include <QtCore/QDir>
#include "data/voxelize_utils.h"
#include "widget/KittiReader.h"

Config config;

void parseConfiguration(const std::string& filename) {}

int32_t main(int32_t argc, char** argv) {
  if (argc < 3) {
    std::cout << "Usage: ./gen_data <config> <directory> [output_directory]" << std::endl;

    return 1;
  }

  parseConfiguration(argv[1]);

  std::string input_directory = argv[2];
  std::string output_dirname = "extracted";
  if (argc > 3) output_dirname = argv[3];

  QDir output_dir = QString::fromStdString(output_dirname);
  if (!output_dir.exists()) {
    std::cout << "Creating ouput directory" << std::endl;
    // TODO implement!
    return 1;
  }

  KittiReader reader;
  reader.initialize(QString::fromStdString(input_directory));
  //  uint32_t current = 0;

  VoxelGrid priorGrid;
  VoxelGrid pastGrid;

  uint32_t current = 0;

  while (current < reader.count()) {
    std::vector<PointcloudPtr> priorPoints;
    std::vector<LabelsPtr> priorLabels;
    std::vector<PointcloudPtr> pastPoints;
    std::vector<LabelsPtr> pastLabels;

    reader.retrieve(current, priorPoints, priorLabels, pastPoints, pastLabels);

    priorGrid.clear();
    pastGrid.clear();

    Eigen::Matrix4f anchor_pose = priorPoints.back()->pose;

    fillVoxelGrid(anchor_pose, priorPoints, priorLabels, priorGrid, config);

    fillVoxelGrid(anchor_pose, priorPoints, priorLabels, pastGrid, config);
    fillVoxelGrid(anchor_pose, pastPoints, pastLabels, pastGrid, config);

    // updating occlusions.
    //    std::cout << "updating occlusions." << std::endl;
    priorGrid.updateOcclusions();
    pastGrid.updateOcclusions();

    priorGrid.insertOcclusionLabels();
    pastGrid.insertOcclusionLabels();

    for (uint32_t i = 0; i < pastPoints.size(); ++i) {
      Eigen::Vector3f endpoint = (anchor_pose.inverse() * pastPoints[i]->pose).col(3).head(3);
      pastGrid.updateInvalid(endpoint);
    }

    // store grid in mat file.

    //    current += config.skip;
  }

  return 0;
}
