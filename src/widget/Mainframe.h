#ifndef MAINFRAME_H_
#define MAINFRAME_H_

#include <stdint.h>
#include <QtCore/QSignalMapper>
#include <QtWidgets/QMainWindow>
#include <future>
#include "KittiReader.h"
#include "common.h"
#include "data/geometry.h"
#include "ui_VoxelizerMainFrame.h"

#include "data/VoxelGrid.h"

// TODO: undo.

/** \brief main widget showing the point cloud and tools to label a point cloud/multiple point clouds. **/
class Mainframe : public QMainWindow {
  Q_OBJECT
 public:
  Mainframe();
  ~Mainframe();

 public slots:
  void open();
  void save();

 signals:
  void readerStarted();
  void readerFinshed();

  void buildVoxelgridStarted();
  void buildVoxelgridFinished();

 protected:
  void readAsync(uint32_t idx);

  void updateScans();
  void activateSpinner();

  void forward();
  void backward();

  void setCurrentScanIdx(int32_t idx);

  void closeEvent(QCloseEvent* event);

  void readConfig();

  void fillVoxelGrid(const Eigen::Matrix4f& anchor_pose, const std::vector<PointcloudPtr>& points,
                     const std::vector<LabelsPtr>& labels, VoxelGrid& grid);

  std::vector<PointcloudPtr> priorPoints_;
  std::vector<LabelsPtr> priorLabels_;

  std::vector<PointcloudPtr> pastPoints_;
  std::vector<LabelsPtr> pastLabels_;

  std::vector<uint32_t> filteredLabels;
  std::string filename;

  void keyPressEvent(QKeyEvent* event);

 protected slots:
  void unsavedChanges();

 private:
  Ui::MainWindow ui;

  std::map<uint32_t, std::string> label_names;
  bool mChangesSinceLastSave;
  QString lastDirectory;

  Point3f midpoint;

  KittiReader reader_;
  std::future<void> readerFuture_;

  VoxelGrid priorVoxels_;
  VoxelGrid pastVoxels_;
};

#endif /* MAINFRAME_H_ */
