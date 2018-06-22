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

  void buildVoxelGrids();

  /** \brief update scans in the viewport.
   *  note: must happen in the thread the OpenGL context resides.
   **/
  void updateScans();

  void disableGui();
  void enableGui();

  /** \brief update voxelgrid in the viewport.
   *  note: must happen in the thread the OpenGL context resides.
   **/
  void updateVoxelGrids();

  /** \brief update voxel size / resolution of voxel grid. **/
  void updateVoxelSize(float value);

  void activateSpinner();

  void forward();
  void backward();

  void setCurrentScanIdx(int32_t idx);

  void closeEvent(QCloseEvent* event);

  void readConfig();

  void fillVoxelGrid(const Eigen::Matrix4f& anchor_pose, const std::vector<PointcloudPtr>& points,
                     const std::vector<LabelsPtr>& labels, VoxelGrid& grid);

  void extractLabeledVoxels(const VoxelGrid& grid, std::vector<LabeledVoxel>& voxels);

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

  VoxelGrid priorVoxelGrid_;
  VoxelGrid pastVoxelGrid_;

  std::vector<LabeledVoxel> priorVoxels_;
  std::vector<LabeledVoxel> pastVoxels_;

  Eigen::Vector4f minExtent;
  Eigen::Vector4f maxExtent;


  float minRange, maxRange;
};

#endif /* MAINFRAME_H_ */
