/* \brief a view, which can be used to label points.
 *
 *
 */
#ifndef POINTVIEW_H_
#define POINTVIEW_H_

#include <stdint.h>
#include <list>
#include <vector>

#include <glow/glbase.h>

#include <QtCore/QTime>
#include <QtCore/QTimer>
#include <QtGui/QMouseEvent>
#include <QtOpenGL/QGLWidget>

#include <GL/gl.h>
#include <GL/glu.h>

#include <glow/GlBuffer.h>
#include <glow/GlColor.h>
#include <glow/GlFramebuffer.h>
#include <glow/GlProgram.h>
#include <glow/GlRenderbuffer.h>
#include <glow/GlShaderCache.h>
#include <glow/GlTexture.h>
#include <glow/GlVertexArray.h>
#include <glow/util/RoSeCamera.h>

#include "common.h"
#include "data/geometry.h"

class Viewport : public QGLWidget {
  Q_OBJECT
 public:
  enum AXIS { XYZ, X, Y, Z };

  enum MODE { NONE, PAINT, POLYGON };

  enum FLAGS { FLAG_OVERWRITE = 1, FLAG_OTHER = 2 };

  Viewport(QWidget* parent = 0, Qt::WindowFlags f = 0);
  virtual ~Viewport();

  void setMaximumScans(uint32_t numScans);

  void setPoints(const std::vector<PointcloudPtr>& priorPoints, std::vector<LabelsPtr>& priorLabels,
                 const std::vector<PointcloudPtr>& pastPoints, std::vector<LabelsPtr>& pastLabels);

  void setDrawingOption(const std::string& name, bool value);

  void setMinRange(float range);
  void setMaxRange(float range);

  void setScanIndex(uint32_t idx);

  void setLabelColors(const std::map<uint32_t, glow::GlColor>& colors);

  void setVoxels(const std::vector<LabeledVoxel>& priorVoxels, const std::vector<LabeledVoxel>& pastVoxels);

  void setVoxelGridProperties(float voxelSize, const Eigen::Vector4f& offset);

  void highlightVoxels(const std::vector<LabeledVoxel>& voxels);

  void setOcclusionVoxels(const std::vector<LabeledVoxel>& voxels);
 signals:
  void labelingChanged();

 public slots:
  void setPointSize(int value);

  void setMode(MODE mode);
  void setFlags(int32_t flags);

  void setFilteredLabels(const std::vector<uint32_t>& labels);

 protected:
  struct BufferInfo {
    uint32_t index;
    uint32_t size;
    Laserscan* scan;
  };

  bool initContext() {
    // enabling core profile
    QGLFormat corefmt;
    corefmt.setVersion(5, 0);  // getting highest compatible format...
    corefmt.setProfile(QGLFormat::CoreProfile);
    setFormat(corefmt);

    // version info.
    QGLFormat fmt = this->format();
    std::cout << "OpenGL Context Version " << fmt.majorVersion() << "." << fmt.minorVersion() << " "
              << ((fmt.profile() == QGLFormat::CoreProfile) ? "core profile" : "compatibility profile") << std::endl;

    makeCurrent();
    glow::inititializeGLEW();

    return true;
  }

  void initPrograms();
  void initVertexBuffers();

  void initializeGL();
  void resizeGL(int width, int height);
  void paintGL();

  void drawPoints(const Eigen::Matrix4f& anchor_pose, const std::vector<BufferInfo>& indexes);

  void mousePressEvent(QMouseEvent*);
  void mouseReleaseEvent(QMouseEvent*);
  void mouseMoveEvent(QMouseEvent*);
  void keyPressEvent(QKeyEvent*);

  glow::GlCamera::KeyboardModifier resolveKeyboardModifier(Qt::KeyboardModifiers modifiers);

  glow::GlCamera::MouseButton resolveMouseButton(Qt::MouseButtons button);

  //  void drawPoints(const std::vector<Point3f>& points, const std::vector<uint32_t>& labels);
  //  void labelPoints(int32_t x, int32_t y, float radius, uint32_t label);

  void fillBuffers(const std::vector<PointcloudPtr>& points, const std::vector<LabelsPtr>& labels,
                   std::vector<BufferInfo>& indexes);

  bool contextInitialized_;
  std::map<uint32_t, glow::GlColor> mLabelColors;

  std::vector<PointcloudPtr> priorPoints_;
  std::vector<LabelsPtr> priorLabels_;
  std::vector<BufferInfo> priorIndexes_;

  std::vector<PointcloudPtr> pastPoints_;
  std::vector<LabelsPtr> pastLabels_;
  std::vector<BufferInfo> pastIndexes_;

  glow::RoSeCamera mCamera;
  bool mChangeCamera{false};

  AXIS mAxis;
  MODE mMode;
  int32_t mFlags;

  std::vector<uint32_t> mFilteredLabels;

  /** selected endpoint **/

  bool buttonPressed;
  QTimer timer_;

  // shaders, etc.
  uint32_t maxScans_{50};
  uint32_t maxPointsPerScan_{150000};
  std::vector<Eigen::Matrix4f> bufPoses_;
  glow::GlBuffer<Point3f> bufPoints_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<float> bufRemissions_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<uint32_t> bufLabels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<uint32_t> bufVisible_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};

  glow::GlBuffer<LabeledVoxel> bufPriorVoxels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<LabeledVoxel> bufPastVoxels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<LabeledVoxel> bufHighlightedVoxels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<LabeledVoxel> bufOccludedVoxels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};

  glow::GlTextureRectangle texLabelColors_;

  glow::GlVertexArray vao_no_points_;
  glow::GlVertexArray vao_points_;
  glow::GlVertexArray vao_prior_voxels_;
  glow::GlVertexArray vao_past_voxels_;
  glow::GlVertexArray vao_highlighted_voxels_;
  glow::GlVertexArray vao_occluded_voxels_;

  glow::GlProgram prgDrawPose_;
  glow::GlProgram prgDrawPoints_;
  glow::GlProgram prgDrawVoxels_;

  int32_t pointSize_{1};

  glow::GlUniform<Eigen::Matrix4f> mvp_{"mvp", Eigen::Matrix4f::Identity()};
  glow::GlUniform<Eigen::Matrix4f> mvp_inv_t_{"mvp_inv_t", Eigen::Matrix4f::Identity()};

  Eigen::Matrix4f model_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f view_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f projection_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f conversion_{glow::RoSe2GL::matrix};

  std::map<Laserscan*, BufferInfo> bufferContent_;

  std::map<std::string, bool> drawingOption_;

  float minRange_{0.0f}, maxRange_{100.0f};

  bool removeGround_{false};
  float groundThreshold_{-1.6f};

  uint32_t singleScanIdx_{0};

  std::vector<int32_t> freeIndexes;
  uint32_t nextFree{0};
  uint32_t loadedScans{0};

  float voxelSize_;
};

#endif /* POINTVIEW_H_ */
