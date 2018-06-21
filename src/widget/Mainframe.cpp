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

#include <boost/lexical_cast.hpp>

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

  connect(ui.chkShowRemission, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("remission", value); });

  connect(ui.chkShowColor, &QCheckBox::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("color", value); });

  connect(this, &Mainframe::readerFinshed, this, &Mainframe::updateScans);
  connect(this, &Mainframe::readerStarted, this, &Mainframe::activateSpinner);

  connect(ui.rdoTrainVoxels, &QRadioButton::toggled,
          [this](bool value) { ui.mViewportXYZ->setDrawingOption("show train", value); });

  /** load labels and colors **/
  std::map<uint32_t, std::string> label_names;
  std::map<uint32_t, glow::GlColor> label_colors;

  getLabelNames("labels.xml", label_names);
  getLabelColors("labels.xml", label_colors);

  ui.mViewportXYZ->setLabelColors(label_colors);

  readConfig();

  ui.mViewportXYZ->setFilteredLabels(filteredLabels);

  reader_.setNumPastScans(ui.spinPastScans->value());
  reader_.setNumPriorScans(ui.spinPriorScans->value());

  // TODO: find reasonable voxel volume size.
  priorVoxels_.initialize(ui.spinVoxelSize->value(), Eigen::Vector4f(0, -20, -2, 1), Eigen::Vector4f(40, 20, 1, 1));
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

    //    ui.sldTimeline->setMaximum(reader_.count());
    ui.btnBackward->setEnabled(false);
    ui.btnForward->setEnabled(false);
    if (reader_.count() > 0) ui.btnForward->setEnabled(true);

    //    if (ui.sldTimeline->value() == 0) setCurrentScanIdx(0);
    //    ui.sldTimeline->setValue(0);

    readerFuture_ = std::async(std::launch::async, &Mainframe::readAsync, this, 0);

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
  // TODO: write appropriate
}

void Mainframe::unsavedChanges() { mChangesSinceLastSave = true; }

void Mainframe::setCurrentScanIdx(int32_t idx) {
  readerFuture_ = std::async(std::launch::async, &Mainframe::readAsync, this, idx);

  ui.sldTimeline->setEnabled(false);
}

void Mainframe::readAsync(uint32_t idx) {
  // TODO progress indicator.
  emit readerStarted();

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

  priorVoxels_.clear();
  pastVoxels_.clear();
  if (priorPoints_.size() > 0) {
    Eigen::Matrix4f anchor_pose = priorPoints_.back()->pose.inverse();

    fillVoxelGrid(anchor_pose, priorPoints_, priorLabels_, priorVoxels_);

    fillVoxelGrid(anchor_pose, priorPoints_, priorLabels_, pastVoxels_);
    fillVoxelGrid(anchor_pose, pastPoints_, pastLabels_, pastVoxels_);
  }
  ui.sldTimeline->setEnabled(true);
}

void Mainframe::fillVoxelGrid(const Eigen::Matrix4f& anchor_pose, const std::vector<PointcloudPtr>& points,
                              const std::vector<LabelsPtr>& labels, VoxelGrid& grid) {
  for (uint32_t t = 0; t < points.size(); ++t) {
    const Eigen::Matrix4f& pose = points[t]->pose;
    for (uint32_t i = 0; i < points[t]->points.size(); ++i) {
      const Point3f& pp = points[t]->points[i];
      Eigen::Vector4f p = anchor_pose * pose * Eigen::Vector4f(pp.x, pp.y, pp.z, 1);

      grid.insert(p, (*labels[t])[i]);
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
void Mainframe::readConfig() {
  std::ifstream in("settings.cfg");

  if (!in.is_open()) return;

  std::string line;
  in.peek();
  while (in.good() && !in.eof()) {
    std::getline(in, line);

    auto tokens = split(line, ":");
    if (tokens[0] == "max scans") {
      uint32_t numScans = boost::lexical_cast<uint32_t>(trim(tokens[1]));
      ui.mViewportXYZ->setMaximumScans(numScans);
      std::cout << "-- Setting 'max scans' to " << numScans << std::endl;
    }

    if (tokens[0] == "max range") {
      float range = boost::lexical_cast<float>(trim(tokens[1]));
      ui.mViewportXYZ->setMaxRange(range);

      std::cout << "-- Setting 'max range' to " << range << std::endl;
    }

    if (tokens[0] == "min range") {
      float range = boost::lexical_cast<float>(trim(tokens[1]));
      ui.mViewportXYZ->setMinRange(range);
      std::cout << "-- Setting 'min range' to " << range << std::endl;
    }

    if (tokens[0] == "ignore") {
      tokens = split(tokens[1], ",");
      for (const auto& token : tokens) {
        uint32_t label = boost::lexical_cast<uint32_t>(trim(token));
        filteredLabels.push_back(label);
      }
    }
  }

  in.close();
}

void Mainframe::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_D || event->key() == Qt::Key_Right) {
    if (ui.btnForward->isEnabled() && ui.sldTimeline->isEnabled()) forward();

  } else if (event->key() == Qt::Key_A || event->key() == Qt::Key_Left) {
    if (ui.btnBackward->isEnabled() && ui.sldTimeline->isEnabled()) backward();
  }
}
