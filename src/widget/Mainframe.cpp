#include "Mainframe.h"

#include <fstream>
#include <iostream>
#include <map>

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtWidgets/QFileDialog>

#include "../data/label_utils.h"
#include "../data/misc.h"

#include <QtWidgets/QMessageBox>

#include "../data/voxelize_utils.h"

using namespace glow;

Mainframe::Mainframe() : mChangesSinceLastSave(false) {
  ui.setupUi(this);

  connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(open()));
  connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(save()));

  /** initialize the paint button mapping **/

  connect(ui.spinPointSize, SIGNAL(valueChanged(int)), ui.mViewportXYZ, SLOT(setPointSize(int)));

  connect(ui.sldTimeline, &QSlider::valueChanged, [this](int value) { setCurrentScanIdx(value); });
  connect(ui.btnForward, &QToolButton::released, [this]() { forward(); });

  connect(ui.btnBackward, &QToolButton::released, [this]() { backward(); });

  connect(ui.chkShowPoints, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("show points", value); });

  connect(ui.chkShowVoxels, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("show voxels", value); });

  connect(this, &Mainframe::readerFinshed, this, &Mainframe::updateScans);
  connect(this, &Mainframe::readerStarted, this, &Mainframe::activateSpinner);
  connect(this, &Mainframe::buildVoxelgridFinished, this, &Mainframe::updateVoxelGrids);

  connect(ui.rdoTrainVoxels, &QRadioButton::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("show train", value); });

  connect(ui.spinVoxelSize, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this](double value) {
            config.voxelSize = value;
            updateVoxelSize(value);
          });

  connect(this, &Mainframe::buildVoxelgridStarted, this, &Mainframe::disableGui);
  connect(this, &Mainframe::buildVoxelgridFinished, this, &Mainframe::enableGui);

  connect(ui.spinPriorScans, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [this](int32_t value) {
    reader_.setNumPriorScans(value);
    setCurrentScanIdx(ui.sldTimeline->value());
  });

  connect(ui.spinPastScans, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [this](int32_t value) {
    reader_.setNumPastScans(value);
    setCurrentScanIdx(ui.sldTimeline->value());
  });

  connect(ui.spinMaxRange, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this](double value) {
            config.maxRange = value;
            ui.mViewportXYZ->setMaxRange(value);
            setCurrentScanIdx(ui.sldTimeline->value());
          });

  connect(ui.chkShowOccluded, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("show occluded", value); });
  connect(ui.chkShowInvalid, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("show invalid", value); });

  /** load labels and colors **/
  std::map<uint32_t, std::string> label_names;
  std::map<uint32_t, glow::GlColor> label_colors;

  getLabelNames("labels.xml", label_names);
  getLabelColors("labels.xml", label_colors);

  ui.mViewportXYZ->setLabelColors(label_colors);

  config = parseConfiguration("settings.cfg");

  ui.mViewportXYZ->setFilteredLabels(config.filteredLabels);
  ui.mViewportXYZ->setDrawingOption("highlight voxels", false);

  ui.spinPriorScans->setValue(config.priorScans);
  ui.spinPastScans->setValue(config.pastScans);

  reader_.setNumPastScans(ui.spinPastScans->value());
  reader_.setNumPriorScans(ui.spinPriorScans->value());

  //  // TODO: find reasonable voxel volume size.
  //  minExtent = Eigen::Vector4f(0, -20, -2, 1);
  //  maxExtent = Eigen::Vector4f(40, 20, 1, 1);

  //  float voxelSize = ui.spinVoxelSize->value();
  ui.spinVoxelSize->setValue(config.voxelSize);
  ui.mViewportXYZ->setMaximumScans(config.maxNumScans);
  ui.mViewportXYZ->setMaxRange(config.maxRange);
  ui.spinMaxRange->setValue(config.maxRange);
  ui.mViewportXYZ->setMinRange(config.minRange);

  ui.spinMinExtentX->setValue(config.minExtent.x());
  ui.spinMinExtentY->setValue(config.minExtent.y());
  ui.spinMinExtentZ->setValue(config.minExtent.z());
  ui.spinMaxExtentX->setValue(config.maxExtent.x());
  ui.spinMaxExtentY->setValue(config.maxExtent.y());
  ui.spinMaxExtentZ->setValue(config.maxExtent.z());

  // extent update.

  connect(ui.spinMaxExtentX, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this](int32_t value) {
            config.maxExtent.x() = value;
            updateExtent();
          });

  connect(ui.spinMaxExtentY, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this](int32_t value) {
            config.maxExtent.y() = value;
            updateExtent();
          });

  connect(ui.spinMaxExtentZ, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this](int32_t value) {
            config.maxExtent.z() = value;
            updateExtent();
          });

  connect(ui.spinMinExtentX, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this](int32_t value) {
            config.minExtent.x() = value;
            updateExtent();
          });

  connect(ui.spinMinExtentY, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this](int32_t value) {
            config.minExtent.y() = value;
            updateExtent();
          });

  connect(ui.spinMinExtentZ, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this](int32_t value) {
            config.minExtent.z() = value;
            updateExtent();
          });

  priorVoxelGrid_.initialize(config.voxelSize, config.minExtent, config.maxExtent);
  pastVoxelGrid_.initialize(config.voxelSize, config.minExtent, config.maxExtent);

  ui.mViewportXYZ->setVoxelGridProperties(config.voxelSize, priorVoxelGrid_.offset());

  connect(ui.actionRecord, &QAction::triggered, [this]() { startRecording(); });
  connect(ui.actionSnapshot, &QAction::triggered, [this]() { snap(); });

  QStringList screenshotFiles = QDir(".").entryList();

  for (auto entry : screenshotFiles) {
    if (entry.startsWith("screenshot")) {
      nextScreenshot_ += 1;
    }
  }
}

Mainframe::~Mainframe() {}

void Mainframe::closeEvent(QCloseEvent* event) {
  //  if (mChangesSinceLastSave) {
  //    int ret = QMessageBox::warning(this, tr("Unsaved changes."), tr("The annotation has been modified.\n"
  //                                                                    "Do you want to save your changes?"),
  //                                   QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
  //                                   QMessageBox::Save);
  //    if (ret == QMessageBox::Save)
  //      save();
  //    else if (ret == QMessageBox::Cancel) {
  //      event->ignore();
  //      return;
  //    }
  //  }
  statusBar()->showMessage("Writing labels...");

  event->accept();
}

void Mainframe::open() {
  //  if (mChangesSinceLastSave) {
  //    int ret = QMessageBox::warning(this, tr("Unsaved changes."), tr("The annotation has been modified.\n"
  //                                                                    "Do you want to save your changes?"),
  //                                   QMessageBox::Save | QMessageBox::Cancel | QMessageBox::Discard,
  //                                   QMessageBox::Save);
  //    if (ret == QMessageBox::Save)
  //      save();
  //    else if (ret == QMessageBox::Cancel)
  //      return;
  //  }

  QString retValue =
      QFileDialog::getExistingDirectory(this, "Select scan directory", lastDirectory, QFileDialog::ShowDirsOnly);

  if (!retValue.isNull()) {
    QDir base_dir(retValue);

    if (!base_dir.exists("velodyne") || !base_dir.exists("poses.txt")) {
      std::cout << "[ERROR] velodyne or poses.txt missing." << std::endl;
      return;
    }

    reader_.initialize(retValue);
    carReader_.initialize(retValue, Car::loadCarModels("../cars"));

    //    ui.sldTimeline->setMaximum(reader_.count());
    ui.btnBackward->setEnabled(false);
    ui.btnForward->setEnabled(false);
    if (reader_.count() > 0) ui.btnForward->setEnabled(true);

    //    if (ui.sldTimeline->value() == 0) setCurrentScanIdx(0);
    //    ui.sldTimeline->setValue(0);

    readerFuture_ = std::async(std::launch::async, &Mainframe::readAsync, this, -1);

    ui.sldTimeline->setEnabled(false);
    ui.sldTimeline->setMaximum(reader_.count());
    ui.sldTimeline->setValue(0);

    lastDirectory = base_dir.absolutePath();

    QString title = "Voxelizer - ";
    title += QFileInfo(retValue).completeBaseName();
    setWindowTitle(title);

    mChangesSinceLastSave = false;
  }
}

void Mainframe::save() {
  saveVoxelGrid(priorVoxelGrid_, "input.mat");
  saveVoxelGrid(pastVoxelGrid_, "labels.mat");
}

void Mainframe::unsavedChanges() { mChangesSinceLastSave = true; }

void Mainframe::setCurrentScanIdx(int32_t idx) {
  std::cout << "setCurrentScanIdx(" << idx << ")" << std::endl;
  if (reader_.count() == 0) return;
  readerFuture_ = std::async(std::launch::async, &Mainframe::readAsync, this, idx);

  ui.sldTimeline->setEnabled(false);
}

void Mainframe::readAsync(int32_t idx_) {
  // TODO progress indicator.
  uint32_t idx;
  emit readerStarted();
  if (idx_ == -1) {
    idx = 0;
  } else {
    idx = idx_;
  }
  if (idx_ == -1) {
    std::cout << "init carPoints" << std::endl;
    carPoints_.clear();
    carLabels_.clear();

    std::vector<Car> c;
    carReader_.load(c);
    int car_no = 0;
    for (const auto& i : c) {
      carPoints_.push_back(std::make_shared<Laserscan>());
      carLabels_.push_back(std::make_shared<std::vector<uint32_t>>());
      std::cout << i.getModel() << std::endl;
      auto cpts = i.getPoints();
      carPoints_[car_no]->pose = i.getPosition();
      // carPoints_[car_no]->pose = Eigen::Matrix4f::Identity();
      for (const auto& pt : (*cpts)) {
        // std::cout<<pt.x<<" "<<pt.y<<" "<<pt.z<<" "<<std::endl;
        Point3f p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        carPoints_[car_no]->points.push_back(p);
        carLabels_[car_no]->push_back(110);
      }
      std::cout << carPoints_[car_no]->points.size() << " car points loaded" << std::endl;
      car_no++;
    }
  }

  ui.sldTimeline->setEnabled(false);

  std::vector<PointcloudPtr> priorPoints;
  std::vector<LabelsPtr> priorLabels;
  std::vector<PointcloudPtr> pastPoints;
  std::vector<LabelsPtr> pastLabels;

  reader_.retrieve(idx, priorPoints, priorLabels, pastPoints, pastLabels);

  priorPoints_ = priorPoints;
  priorLabels_ = priorLabels;
  pastPoints_ = pastPoints;
  pastLabels_ = pastLabels;

  emit readerFinshed();

  buildVoxelGrids();

  ui.sldTimeline->setEnabled(true);
}

void Mainframe::disableGui() {
  ui.spinVoxelSize->setEnabled(false);
  ui.spinPastScans->setEnabled(false);
  ui.spinPriorScans->setEnabled(false);
  ui.sldTimeline->setEnabled(false);
  ui.spinMaxRange->setEnabled(false);

  ui.spinMaxExtentX->setEnabled(false);
  ui.spinMaxExtentY->setEnabled(false);
  ui.spinMaxExtentZ->setEnabled(false);
  ui.spinMinExtentX->setEnabled(false);
  ui.spinMinExtentY->setEnabled(false);
  ui.spinMinExtentZ->setEnabled(false);
}

void Mainframe::enableGui() {
  ui.spinVoxelSize->setEnabled(true);
  ui.spinPastScans->setEnabled(true);
  ui.spinPriorScans->setEnabled(true);
  ui.sldTimeline->setEnabled(true);
  ui.spinMaxRange->setEnabled(true);

  ui.spinMaxExtentX->setEnabled(true);
  ui.spinMaxExtentY->setEnabled(true);
  ui.spinMaxExtentZ->setEnabled(true);
  ui.spinMinExtentX->setEnabled(true);
  ui.spinMinExtentY->setEnabled(true);
  ui.spinMinExtentZ->setEnabled(true);
}

void Mainframe::buildVoxelGrids() {
  emit buildVoxelgridStarted();

  priorVoxelGrid_.clear();
  pastVoxelGrid_.clear();

  if (priorPoints_.size() > 0) {
    Eigen::Matrix4f anchor_pose = priorPoints_.back()->pose;

    std::cout << "fill grids..." << std::flush;
    fillVoxelGrid(anchor_pose, priorPoints_, priorLabels_, priorVoxelGrid_, config);

    // fill test Grid
    fillVoxelGrid(anchor_pose, priorPoints_, priorLabels_, pastVoxelGrid_, config);
    fillVoxelGrid(anchor_pose, pastPoints_, pastLabels_, pastVoxelGrid_, config);
    std::cout << "finished." << std::endl;

    // create a slightly different config for AutoAuto models
//    Config carconf;
//    carconf.minRange = 0.;
//    carconf.maxRange = 64.;
//    carconf.hidecar = false;
//    carconf.maxExtent = config.maxExtent;
//    carconf.minExtent = config.minExtent;
//    carconf.maxNumScans = config.maxNumScans;
//    carconf.pastScans = config.pastScans;
//    carconf.pastDistance = config.pastDistance;
//    carconf.filteredLabels = config.filteredLabels;
//    carconf.joinedLabels = config.joinedLabels;
//    carconf.stride_num = config.stride_num;
//    carconf.stride_distance = config.stride_distance;

//    fillVoxelGrid(anchor_pose, carPoints_, carLabels_, pastVoxelGrid_, carconf);
    std::cout << "carPoints_.size() = " << carPoints_.size() << std::endl;

    // updating occlusions.
    std::cout << "updating occlusions..." << std::flush;

    priorVoxelGrid_.updateOcclusions();
    pastVoxelGrid_.updateOcclusions();
    std::cout << "finished." << std::endl;

    std::cout << "insert occlusion labels..." << std::flush;
    priorVoxelGrid_.insertOcclusionLabels();
    pastVoxelGrid_.insertOcclusionLabels();
    std::cout << std::endl;

    std::cout << "update invalid..." << std::flush;
    for (uint32_t i = 0; i < pastPoints_.size(); ++i) {
      Eigen::Vector3f endpoint = (anchor_pose.inverse() * pastPoints_[i]->pose).col(3).head(3);
      pastVoxelGrid_.updateInvalid(endpoint);
    }
    std::cout << "finished." << std::endl;

    std::cout << "Updating visualized voxels..." << std::flush;
    // only visualization code.
    priorVoxels_.clear();
    pastVoxels_.clear();
    // extract voxels and labels.
    extractLabeledVoxels(priorVoxelGrid_, priorVoxels_);
    extractLabeledVoxels(pastVoxelGrid_, pastVoxels_);
    std::cout << "finished." << std::endl;

    std::cout << "Used " << (priorVoxelGrid_.voxels().size() * sizeof(LabeledVoxel)) / (1000 * 1000)
              << " MB for visualization." << std::endl;
    std::cout << "Used " << (pastVoxelGrid_.voxels().size() * sizeof(LabeledVoxel)) / (1000 * 1000)
              << " MB for visualization." << std::endl;

    //    std::cout << "end" << std::endl;
  }

  emit buildVoxelgridFinished();
}

void Mainframe::updateVoxelGrids() {
  ui.mViewportXYZ->setVoxelGridProperties(std::max<float>(0.01, ui.spinVoxelSize->value()), priorVoxelGrid_.offset());
  ui.mViewportXYZ->setVoxels(priorVoxels_, pastVoxels_);

  if (visited_.size() > 0) {
    std::vector<LabeledVoxel> voxels;
    float voxelSize = priorVoxelGrid_.resolution();
    Eigen::Vector4f offset = priorVoxelGrid_.offset();

    for (uint32_t i = 0; i < visited_.size(); ++i) {
      LabeledVoxel lv;
      Eigen::Vector4f pos = offset + Eigen::Vector4f(visited_[i].x() * voxelSize, visited_[i].y() * voxelSize,
                                                     visited_[i].z() * voxelSize, 0.0f);
      lv.position = vec3(pos.x(), pos.y(), pos.z());
      lv.label = 11;
      voxels.push_back(lv);
    }

    ui.mViewportXYZ->highlightVoxels(voxels);
  }

  updateOccludedVoxels();
  updateInvalidVoxels();

  if (recording_) {
    std::string out_filename = outputDirectory_;
    out_filename += QString("/%1.png").arg((int)ui.sldTimeline->value(), 5, 10, (QChar)'0').toStdString();

    ui.mViewportXYZ->grabFrameBuffer().save(QString::fromStdString(out_filename));
    std::cout << "Writing to " << out_filename << std::endl;

    forward();
  }
}

void Mainframe::updateOccludedVoxels() {
  VoxelGrid& grid = (ui.rdoTrainVoxels->isChecked()) ? priorVoxelGrid_ : pastVoxelGrid_;
  std::vector<LabeledVoxel> voxels;
  float voxelSize = grid.resolution();
  Eigen::Vector4f offset = grid.offset();

  for (uint32_t x = 0; x < grid.size(0); ++x) {
    for (uint32_t y = 0; y < grid.size(1); ++y) {
      for (uint32_t z = 0; z < grid.size(2); ++z) {
        if (!grid.isOccluded(x, y, z)) continue;
        LabeledVoxel lv;
        Eigen::Vector4f pos =
            offset + Eigen::Vector4f(x * voxelSize + 0.1, y * voxelSize + 0.1, z * voxelSize + 0.1, 0.0f);
        lv.position = vec3(pos.x(), pos.y(), pos.z());
        lv.label = 11;
        voxels.push_back(lv);
      }
    }
  }

  ui.mViewportXYZ->setOcclusionVoxels(voxels);
}

void Mainframe::updateInvalidVoxels() {
  std::vector<LabeledVoxel> voxels;

  float voxelSize = pastVoxelGrid_.resolution();
  Eigen::Vector4f offset = pastVoxelGrid_.offset();

  for (uint32_t x = 0; x < pastVoxelGrid_.size(0); ++x) {
    for (uint32_t y = 0; y < pastVoxelGrid_.size(1); ++y) {
      for (uint32_t z = 0; z < pastVoxelGrid_.size(2); ++z) {
        if (!pastVoxelGrid_.isInvalid(x, y, z)) continue;

        LabeledVoxel lv;
        Eigen::Vector4f pos =
            offset + Eigen::Vector4f(x * voxelSize + 0.1, y * voxelSize + 0.1, z * voxelSize + 0.1, 0.0f);
        lv.position = vec3(pos.x(), pos.y(), pos.z());
        lv.label = 11;
        voxels.push_back(lv);
      }
    }
  }

  ui.mViewportXYZ->setInvalidVoxels(voxels);
}

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

void Mainframe::activateSpinner() { statusBar()->showMessage("     Reading scans..."); }

void Mainframe::updateScans() {
  statusBar()->clearMessage();
  glow::_CheckGlError(__FILE__, __LINE__);
  ui.mViewportXYZ->setPoints(priorPoints_, priorLabels_, pastPoints_, pastLabels_);
  glow::_CheckGlError(__FILE__, __LINE__);
}

void Mainframe::updateVoxelSize(float voxelSize) {
  voxelSize = std::max<float>(0.01, voxelSize);
  priorVoxelGrid_.initialize(voxelSize, config.minExtent, config.maxExtent);
  pastVoxelGrid_.initialize(voxelSize, config.minExtent, config.maxExtent);

  readerFuture_ = std::async(std::launch::async, &Mainframe::buildVoxelGrids, this);
}

void Mainframe::updateExtent() {
  priorVoxelGrid_.initialize(config.voxelSize, config.minExtent, config.maxExtent);
  pastVoxelGrid_.initialize(config.voxelSize, config.minExtent, config.maxExtent);

  readerFuture_ = std::async(std::launch::async, &Mainframe::buildVoxelGrids, this);
}

void Mainframe::forward() {
  int32_t value = ui.sldTimeline->value() + 1;
  if (value < int32_t(reader_.count())) ui.sldTimeline->setValue(value);
  ui.btnBackward->setEnabled(true);
  if (value == int32_t(reader_.count()) - 1) ui.btnForward->setEnabled(false);
}

void Mainframe::backward() {
  int32_t value = ui.sldTimeline->value() - 1;
  if (value >= 0) ui.sldTimeline->setValue(value);
  ui.btnForward->setEnabled(true);
  if (value == 0) ui.btnBackward->setEnabled(false);
}
void Mainframe::readConfig() {}

void Mainframe::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_D || event->key() == Qt::Key_Right) {
    if (ui.btnForward->isEnabled() && ui.sldTimeline->isEnabled()) forward();

  } else if (event->key() == Qt::Key_A || event->key() == Qt::Key_Left) {
    if (ui.btnBackward->isEnabled() && ui.sldTimeline->isEnabled()) backward();
  }
}

void Mainframe::startRecording() {
  if (!ui.actionRecord->isChecked()) {
    recording_ = false;
  } else {
    QString retValue = QFileDialog::getExistingDirectory(this, "Select output directory", "");
    if (!retValue.isNull()) {
      outputDirectory_ = retValue.toStdString();
      recording_ = true;
    }
  }
}

void Mainframe::snap() {
  std::string out_filename = "./";
  out_filename += QString("screenshot%1.png").arg((int)nextScreenshot_++, 5, 10, (QChar)'0').toStdString();

  ui.mViewportXYZ->grabFrameBuffer().save(QString::fromStdString(out_filename));
  std::cout << "Writing to " << out_filename << std::endl;
}
