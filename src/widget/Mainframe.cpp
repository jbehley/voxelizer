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

  /** load labels and colors **/
  std::map<uint32_t, std::string> label_names;
  std::map<uint32_t, glow::GlColor> label_colors;

  getLabelNames("labels.xml", label_names);
  getLabelColors("labels.xml", label_colors);

  ui.mViewportXYZ->setLabelColors(label_colors);

  readConfig();
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
    const auto& tile = reader_.getTile(Eigen::Vector3f::Zero());
    readerFuture_ = std::async(std::launch::async, &Mainframe::readAsync, this, tile.i, tile.j);

    lastDirectory = base_dir.absolutePath();

    QString title = "Point Labeler - ";
    title += QFileInfo(retValue).completeBaseName();
    setWindowTitle(title);

    mChangesSinceLastSave = false;
  }
}

void Mainframe::save() {
  // TODO: write appropriate
}

void Mainframe::unsavedChanges() { mChangesSinceLastSave = true; }

void Mainframe::setCurrentScanIdx(int32_t idx) { ui.mViewportXYZ->setScanIndex(idx); }

void Mainframe::readAsync(uint32_t i, uint32_t j) {
  // TODO progress indicator.
  emit readerStarted();

  std::vector<uint32_t> indexes;
  std::vector<PointcloudPtr> points;
  std::vector<LabelsPtr> labels;
  std::vector<ColorsPtr> colors;

  std::vector<uint32_t> oldIndexes = indexes_;
  std::vector<LabelsPtr> oldLabels = labels_;

  reader_.retrieve(i, j, indexes, points, labels, colors);

  indexes_ = indexes;
  points_ = points;
  labels_ = labels;
  colors_ = colors;

  // find difference.
  std::vector<uint32_t> diff_indexes;
  index_difference(oldLabels, labels_, diff_indexes);

  std::vector<uint32_t> removedIndexes;
  std::vector<LabelsPtr> removedLabels;

  for (auto index : diff_indexes) {
    removedIndexes.push_back(oldIndexes[index]);
    removedLabels.push_back(oldLabels[index]);
  }
  // only update really needed label files.
  reader_.update(removedIndexes, removedLabels);

  const auto& tile = reader_.getTile(i, j);


  emit readerFinshed();
}

void Mainframe::activateSpinner() {
  statusBar()->showMessage("     Reading scans...");
}

void Mainframe::updateScans() {
  statusBar()->clearMessage();
  glow::_CheckGlError(__FILE__, __LINE__);
  ui.mViewportXYZ->setPoints(points_, labels_);
  glow::_CheckGlError(__FILE__, __LINE__);

  ui.sldTimeline->setMaximum(indexes_.size());
  ui.sldTimeline->setValue(0);
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

    if (tokens[0] == "tile size") {
      float tileSize = boost::lexical_cast<float>(trim(tokens[1]));
      reader_.setTileSize(tileSize);
      std::cout << "-- Setting 'tile size' to " << tileSize << std::endl;
    }

    if (tokens[0] == "max range") {
      float range = boost::lexical_cast<float>(trim(tokens[1]));
      ui.mViewportXYZ->setMaxRange(range);
      reader_.setMaximumDistance(range);
      std::cout << "-- Setting 'max range' to " << range << std::endl;
    }

    if (tokens[0] == "min range") {
      float range = boost::lexical_cast<float>(trim(tokens[1]));
      ui.mViewportXYZ->setMinRange(range);
      std::cout << "-- Setting 'min range' to " << range << std::endl;
    }
  }

  in.close();
}

void Mainframe::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_D || event->key() == Qt::Key_Right) {
    if (ui.btnForward->isEnabled()) forward();

  } else if (event->key() == Qt::Key_A || event->key() == Qt::Key_Left) {
    if (ui.btnBackward->isEnabled()) backward();
  }
}
