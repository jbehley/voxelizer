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

 protected:
  void readAsync(uint32_t i, uint32_t j);

  void updateScans();
  void activateSpinner();

  void forward();
  void backward();

  void setCurrentScanIdx(int32_t idx);

  void closeEvent(QCloseEvent* event);

  void readConfig();

  std::vector<uint32_t> indexes_;
  std::vector<PointcloudPtr> points_;
  std::vector<LabelsPtr> labels_;
  std::vector<ColorsPtr> colors_;

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
};

#endif /* MAINFRAME_H_ */
