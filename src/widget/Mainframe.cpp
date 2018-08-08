#include "Mainframe.h"

#include <fstream>
#include <iostream>
#include <map>

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtWidgets/QFileDialog>
#include <QtGui/QClipboard>

#include "../data/label_utils.h"
#include "../data/misc.h"

#include <QtWidgets/QMessageBox>

#include "../data/voxelize_utils.h"

using namespace glow;

Mainframe::Mainframe(VoxelGrid inVoxelGrid) : mChangesSinceLastSave(false) {
  ui.setupUi(this);

  /** load labels and colors **/
  std::map<uint32_t, glow::GlColor> label_colors;
  getLabelColors("labels.xml", label_colors);
  config = parseConfiguration("settings.cfg");

  priorVoxelGrid_ = inVoxelGrid;
  extractLabeledVoxels(priorVoxelGrid_, priorVoxels_);

  // updateVoxelGrids();

  ui.mViewportXYZ->setLabelColors(label_colors);
  ui.mViewportXYZ->setFilteredLabels(config.filteredLabels);
  // ui.mViewportXYZ->setDrawingOption("highlight voxels", true);
  ui.mViewportXYZ->setVoxelGridProperties(config.voxelSize, priorVoxelGrid_.offset());
  ui.mViewportXYZ->setVoxels(priorVoxels_);

  ui.mViewportXYZ->setDrawingOption("show occluded", false);
  ui.mViewportXYZ->setDrawingOption("highlight voxels", false);
  

  // priorVoxelGrid_.

  // for(int i = 0; i < 22445; ++i)
  // {
  //   if (priorVoxelGrid_.label[i] > 0) std::cout << "\tx[" << i <<"] = "<< priorVoxelGrid_.label[i] << "\n" ;
  // }



//     VoxelGrid_ = voxelGrid;
//     ui.mViewportXYZ->setVoxelGridProperties(voxelSize, VoxelGrid_.offset());


}

Mainframe::~Mainframe() {}


void Mainframe::saveScreenshot(){

  QImage img = ui.mViewportXYZ->grabFrameBuffer();
  img.save("screenshot.png");
  QApplication::clipboard()->setImage(img);

}


// void Mainframe::buildVoxelGrids() {
//   emit buildVoxelgridStarted();

//   priorVoxelGrid_.clear();
//   // pastVoxelGrid_.clear();

//   fillVoxelGridMat(priorLabels_, priorVoxelGrid_, config);
//   // fillVoxelGridMat(pastLabels_, pastVoxelGrid_, config);

//   // only visualization code.
//   priorVoxels_.clear();
//   // pastVoxels_.clear();
//   // extract voxels and labels.
//   extractLabeledVoxels(priorVoxelGrid_, priorVoxels_);
//   // extractLabeledVoxels(pastVoxelGrid_, pastVoxels_);

//   emit buildVoxelgridFinished();
// }

// void Mainframe::updateVoxelGrids() {
//   // ui.mViewportXYZ->setVoxelGridProperties(std::max<float>(0.01, ui.spinVoxelSize->value()), priorVoxelGrid_.offset());
//   // ui.mViewportXYZ->setVoxels(priorVoxels_);

//   // if (visited_.size() > 0) {
//     // std::cout << "visited_ > 0" << std::endl;
//     // std::vector<LabeledVoxel> voxels;
//     // float voxelSize = priorVoxelGrid_.resolution();
//     // Eigen::Vector4f offset = priorVoxelGrid_.offset();

//     // for (uint32_t i = 0; i < visited_.size(); ++i) {
//     //   LabeledVoxel lv;
//     //   Eigen::Vector4f pos = offset + Eigen::Vector4f(visited_[i].x() * voxelSize, visited_[i].y() * voxelSize,
//     //                                                  visited_[i].z() * voxelSize, 0.0f);
//     //   lv.position = vec3(pos.x(), pos.y(), pos.z());
//     //   lv.label = 11;
//     //   voxels.push_back(lv);
//     // }

//     // ui.mViewportXYZ->highlightVoxels(voxels);
//   // } else{
//   //   std::cout << "Error: visited_ == 0" << std::endl;
//   // }

// }



void Mainframe::extractLabeledVoxels(const VoxelGrid& grid, std::vector<LabeledVoxel>& labeledVoxels) {
  labeledVoxels.clear();
  // fixme: iterate only over occuppied voxels.
  Eigen::Vector4f offset = grid.offset();
  float voxelSize = grid.resolution();

  for (uint32_t x = 0; x < grid.size(0); ++x) {
    for (uint32_t y = 0; y < grid.size(1); ++y) {
      for (uint32_t z = 0; z < grid.size(2); ++z) {
        const VoxelGrid::Voxel& v = grid(x, y, z);
        if (v.count > 0) {
          LabeledVoxel lv;
          Eigen::Vector4f pos = offset + Eigen::Vector4f(x * voxelSize, y * voxelSize, z * voxelSize, 0.0f);
          lv.position = vec3(pos.x(), pos.y(), pos.z());

          uint32_t maxCount = 0;
          uint32_t maxLabel = 0;

          for (auto it = v.labels.begin(); it != v.labels.end(); ++it) {
            if (it->second > maxCount) {
              maxCount = it->second;
              maxLabel = it->first;
            }
          }

          lv.label = maxLabel;
          labeledVoxels.push_back(lv);
        }
      }
    }
  }
}

// void Mainframe::updateVoxelSize(float voxelSize) {
//   voxelSize = std::max<float>(0.01, voxelSize);
//   priorVoxelGrid_.initialize(voxelSize, config.minExtent, config.maxExtent);
//   // pastVoxelGrid_.initialize(voxelSize, config.minExtent, config.maxExtent);

//   readerFuture_ = std::async(std::launch::async, &Mainframe::buildVoxelGrids, this);
// }


