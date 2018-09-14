#include <matio.h>
#include <stdint.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <QtCore/QDir>
#include "data/voxelize_utils.h"
#include "rv/Stopwatch.h"
#include "rv/string_utils.h"
#include "widget/KittiReader.h"

using namespace rv;

float poseDistance(const Eigen::Matrix4f& A, const Eigen::Matrix4f& B) {
  return (A.col(3).head(3) - B.col(3).head(3)).norm();
}

int32_t main(int32_t argc, char** argv) {
  if (argc < 3) {
    std::cout << "Usage: ./gen_data <config> <directory> [output_directory]" << std::endl;

    return 1;
  }

  Config config = parseConfiguration(argv[1]);

  std::string input_directory = argv[2];
  std::string output_dirname = "extracted";
  if (argc > 3) output_dirname = argv[3];

  QDir output_dir(QString::fromStdString(output_dirname));

  if (!output_dir.exists()) {
    std::cout << "Creating output directory: " << output_dir.absolutePath().toStdString() << std::endl;
    if (!output_dir.mkpath(output_dir.absolutePath())) {
      throw std::runtime_error("Unable to create output directory.");
    }

    output_dir.mkdir("input");
    output_dir.mkdir("label");
  }

  std::string seq = QDir(QString::fromStdString(input_directory)).dirName().toStdString();

  KittiReader reader;
  reader.initialize(QString::fromStdString(input_directory));

  reader.setNumPriorScans(config.priorScans);
  reader.setNumPastScans(config.pastScans);

  VoxelGrid priorGrid;
  VoxelGrid pastGrid;

  priorGrid.initialize(config.voxelSize, config.minExtent, config.maxExtent);
  pastGrid.initialize(config.voxelSize, config.minExtent, config.maxExtent);

  std::vector<Eigen::Matrix4f> poses = reader.poses();

  uint32_t current = 0;

  while (current < reader.count()) {
    std::cout << current << std::endl;

    std::vector<PointcloudPtr> priorPoints;
    std::vector<LabelsPtr> priorLabels;
    std::vector<PointcloudPtr> pastPoints;
    std::vector<LabelsPtr> pastLabels;

    reader.retrieve(current, priorPoints, priorLabels, pastPoints, pastLabels);

    // ensure that labels are present, only then store data.
    uint32_t labelCount = 0;
    uint32_t pointCount = 0;

    for (uint32_t i = 0; i < pastLabels.size(); ++i) {
      pointCount += pastLabels[i]->size();
      for (uint32_t j = 0; j < pastLabels[i]->size(); ++j) {
        uint32_t label = (*pastLabels[i])[j];
        if (label > 0) labelCount += 1;
      }
    }

    float percentageLabeled = 100.0f * labelCount / pointCount;
    std::cout << percentageLabeled << "% points labeled." << std::endl;

    priorGrid.clear();
    pastGrid.clear();

    if (percentageLabeled > 90.0f) {
      Eigen::Matrix4f anchor_pose = priorPoints.back()->pose;

      Stopwatch::tic();
      fillVoxelGrid(anchor_pose, priorPoints, priorLabels, priorGrid, config);

      fillVoxelGrid(anchor_pose, priorPoints, priorLabels, pastGrid, config);
      fillVoxelGrid(anchor_pose, pastPoints, pastLabels, pastGrid, config);
      std::cout << "fill voxelgrid took " << Stopwatch::toc() << std::endl;

      Stopwatch::tic();
      // updating occlusions.
      //    std::cout << "updating occlusions." << std::endl;
      priorGrid.updateOcclusions();
      pastGrid.updateOcclusions();
      std::cout << "update occlusions took " << Stopwatch::toc() << std::endl;

      Stopwatch::tic();
      priorGrid.insertOcclusionLabels();
      pastGrid.insertOcclusionLabels();
      std::cout << "occlusion labels took " << Stopwatch::toc() << std::endl;

      Stopwatch::tic();
      for (uint32_t i = 0; i < pastPoints.size(); ++i) {
        Eigen::Vector3f endpoint = (anchor_pose.inverse() * pastPoints[i]->pose).col(3).head(3);
        pastGrid.updateInvalid(endpoint);
      }
      std::cout << "update invalid took " << Stopwatch::toc() << std::endl;

      Stopwatch::tic();
      // store grid in mat file.
      std::stringstream outname;
      outname << seq << "_" << std::setfill('0') << std::setw(6) << current << ".mat";
      saveVoxelGrid(priorGrid, output_dirname + "/input/" + outname.str());
      saveVoxelGrid(pastGrid, output_dirname + "/label/" + outname.str());
      std::cout << "saving took " << Stopwatch::toc() << std::endl;
    } else {
      std::cout << "skipped." << std::endl;
    }

    // get index of next scan.
    float distance = 0.0f;
    uint32_t count = 0;
    while ((count < config.stride_num || distance < config.stride_distance || count == 0) &&
           current + 1 < poses.size()) {
      distance += poseDistance(poses[current], poses[current + 1]);
      count += 1;
      current += 1;
    }

    if ((count < config.stride_num || distance < config.stride_distance || count == 0) && current + 1 >= poses.size()) {
      // no further scan can be extracted possible.
      break;
    }
  }

  return 0;
}
